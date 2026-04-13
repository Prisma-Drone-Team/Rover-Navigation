#include <memory>
#include <string>
#include <vector>
#include <map>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_loader.hpp"

#include "rover_manager/primitive_base.hpp"

namespace rover_manager
{

class RoverManager : public rclcpp::Node
{
public:
  RoverManager() : Node("rover_manager")
  {
    // --- Parameters ---
    this->declare_parameter("robot_namespace", "robot1");
    this->declare_parameter("primitives", std::vector<std::string>{});
    this->declare_parameter("command_topic", "seed_pdt_rover/command");
    this->declare_parameter("feedback_topic", "seed_pdt_rover/state");

    this->get_parameter("robot_namespace", robot_ns_);
    this->get_parameter("command_topic", command_topic_);
    this->get_parameter("feedback_topic", feedback_topic_);

    // --- Publishers / Subscribers ---
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      command_topic_, 10,
      std::bind(&RoverManager::command_callback, this, std::placeholders::_1));

    feedback_pub_ = this->create_publisher<std_msgs::msg::String>(feedback_topic_, 10);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/" + robot_ns_ + "/cmd_vel", 10);

    // --- Load primitives via pluginlib ---
    primitive_loader_ = std::make_shared<pluginlib::ClassLoader<PrimitiveBase>>(
      "rover_manager", "rover_manager::PrimitiveBase");

    std::vector<std::string> primitive_names;
    this->get_parameter("primitives", primitive_names);

    auto shared_this = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});

    for (const auto & prim_name : primitive_names) {
      std::string plugin_type;
      try { this->declare_parameter(prim_name + ".plugin", ""); }
      catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {}
      this->get_parameter(prim_name + ".plugin", plugin_type);

      if (plugin_type.empty()) {
        RCLCPP_ERROR(this->get_logger(),
          "Primitive '%s' has no 'plugin' parameter, skipping", prim_name.c_str());
        continue;
      }

      try {
        auto primitive = primitive_loader_->createSharedInstance(plugin_type);
        primitive->initialize(shared_this, prim_name, robot_ns_);
        primitives_[prim_name] = primitive;

        RCLCPP_WARN(this->get_logger(),
          "Loaded primitive '%s' (type: %s)", prim_name.c_str(), plugin_type.c_str());
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to load primitive '%s' (type: %s): %s",
          prim_name.c_str(), plugin_type.c_str(), ex.what());
      }
    }

    RCLCPP_WARN(this->get_logger(),
      "RoverManager initialized with %zu primitive(s), ns='%s'",
      primitives_.size(), robot_ns_.c_str());

    // --- Timer for tick loop ---
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&RoverManager::tick_callback, this));
  }

  ~RoverManager()
  {
    primitives_.clear();
    primitive_loader_.reset();
  }

private:
  // ============================================================
  // Command handling
  // ============================================================

  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    new_command_ = msg->data;
    RCLCPP_WARN(this->get_logger(), "Received command: %s", new_command_.c_str());
  }

  // ============================================================
  // Tick loop — called every 200ms
  // ============================================================

  void tick_callback()
  {
    // If there's an active primitive, tick it
    if (active_primitive_) {
      active_primitive_->tick();

      auto status = active_primitive_->getStatus();

      // Publish feedback
      auto fb_msg = std_msgs::msg::String();
      fb_msg.data = active_primitive_name_ + ": " +
                    statusToString(status) + " - " +
                    active_primitive_->getFeedback();
      feedback_pub_->publish(fb_msg);

      // If primitive finished, clean up
      if (status == PrimitiveStatus::SUCCEEDED ||
          status == PrimitiveStatus::FAILED ||
          status == PrimitiveStatus::CANCELLED)
      {
        RCLCPP_WARN(this->get_logger(), "Primitive '%s' finished with status: %s",
                    active_primitive_name_.c_str(), statusToString(status).c_str());
        active_primitive_->reset();
        active_primitive_ = nullptr;
        active_primitive_name_ = "";
        current_command_ = "";
      }
    }

    // Process new command
    if (!new_command_.empty() && new_command_ != current_command_) {
      process_command(new_command_);
      current_command_ = new_command_;
      new_command_ = "";
    } else if (!new_command_.empty()) {
      new_command_ = "";
    }
  }

  // ============================================================
  // Command processing
  // ============================================================

  void process_command(const std::string & cmd)
  {
    // Parse command: "op(arg1,arg2,...)" or "op"
    auto tokens = instance2vector(cmd);
    if (tokens.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Empty command");
      return;
    }

    std::string op = tokens[0];
    std::vector<std::string> args(tokens.begin() + 1, tokens.end());

    // Handle built-in commands
    if (op == "stop" || op == "emergency_stop") {
      handle_stop();
      return;
    }
    if (op == "cancel" || op == "cancel_goal") {
      handle_cancel();
      return;
    }
    if (op == "wait") {
      handle_cancel();
      return;
    }

    // Find the primitive for this command
    auto it = primitives_.find(op);
    if (it == primitives_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Unknown command / no primitive for '%s'", op.c_str());
      return;
    }

    // Cancel current primitive if switching
    if (active_primitive_ && active_primitive_name_ != op) {
      RCLCPP_WARN(this->get_logger(), "Switching from '%s' to '%s'",
                  active_primitive_name_.c_str(), op.c_str());
      active_primitive_->cancel();
      active_primitive_->reset();
    }

    // Execute new primitive
    active_primitive_ = it->second;
    active_primitive_name_ = op;

    if (!active_primitive_->execute(args)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to execute primitive '%s'", op.c_str());
      active_primitive_->reset();
      active_primitive_ = nullptr;
      active_primitive_name_ = "";
    }
  }

  void handle_stop()
  {
    if (active_primitive_) {
      active_primitive_->cancel();
      active_primitive_->reset();
      active_primitive_ = nullptr;
      active_primitive_name_ = "";
    }
    // Emergency stop: zero velocity
    geometry_msgs::msg::Twist tw;
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    tw.angular.z = 0.0;
    cmd_vel_pub_->publish(tw);
    current_command_ = "";
    RCLCPP_WARN(this->get_logger(), "Emergency stop!");
  }

  void handle_cancel()
  {
    if (active_primitive_) {
      active_primitive_->cancel();
      active_primitive_->reset();
      active_primitive_ = nullptr;
      active_primitive_name_ = "";
    }
    current_command_ = "";
    RCLCPP_WARN(this->get_logger(), "Command cancelled");
  }

  // ============================================================
  // Command parsing
  // ============================================================

  // std::vector<std::string> parse_command(const std::string & cmd)
  // {
  //   std::vector<std::string> result;

  //   // Find the operation name (before '(' or the whole string)
  //   auto paren_pos = cmd.find('(');
  //   if (paren_pos == std::string::npos) {
  //     // Simple command like "stop" or "wait"
  //     result.push_back(cmd);
  //     return result;
  //   }

  //   // Operation name
  //   result.push_back(cmd.substr(0, paren_pos));

  //   // Arguments between parentheses
  //   auto close_paren = cmd.rfind(')');
  //   if (close_paren == std::string::npos || close_paren <= paren_pos) {
  //     return result;
  //   }

  //   std::string args_str = cmd.substr(paren_pos + 1, close_paren - paren_pos - 1);

  //   // Split by comma, but respect nested parentheses
  //   int depth = 0;
  //   std::string current_arg;
  //   for (char c : args_str) {
  //     if (c == '(' || c == '[') {
  //       depth++;
  //       current_arg += c;
  //     } else if (c == ')' || c == ']') {
  //       depth--;
  //       current_arg += c;
  //     } else if (c == ',' && depth == 0) {
  //       if (!current_arg.empty()) {
  //         result.push_back(current_arg);
  //         current_arg = "";
  //       }
  //     } else {
  //       current_arg += c;
  //     }
  //   }
  //   if (!current_arg.empty()) {
  //     result.push_back(current_arg);
  //   }

  //   return result;
  // }

  std::vector<std::string> instance2vector(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    ss >> std::noskipws;
    ss>>c;
    while(!ss.eof())
    {
        if(c=='"' && !isString){ isString=true; app=app+c; }
        else if(c=='"' && isString){ isString=false; app=app+c; }
        else if(isString){ app=app+c; }
        else if(c=='(' && isAtom){ isAtom=false; result.push_back(app); app=""; }
        else if(c=='(' || c=='['){ count++; app=app+c; }
        else if((c==')' || c==']') && count!=0){ count--; app=app+c; }
        else if(c!=',' || count!=0) app=app+c;
        else { result.push_back(app); app=""; }
        ss>>c;
    }
    if(isAtom) {
        if( app.find('\\') != std::string::npos ){
            std::stringstream ss2(app);
            std::string substr;
            while(std::getline(ss2, substr, '\\')) result.push_back(substr);
        }
        else result.push_back(app);
    }
    else{
        if(!app.empty()) app.erase(app.size()-1);
        result.push_back(app);
    }
    return result;
}

  // ============================================================
  // Member variables
  // ============================================================

  // Parameters
  std::string robot_ns_;
  std::string command_topic_;
  std::string feedback_topic_;

  // Plugin loader
  std::shared_ptr<pluginlib::ClassLoader<PrimitiveBase>> primitive_loader_;

  // Loaded primitives (name -> instance)
  std::map<std::string, std::shared_ptr<PrimitiveBase>> primitives_;

  // Active primitive
  std::shared_ptr<PrimitiveBase> active_primitive_;
  std::string active_primitive_name_;

  // Command state
  std::string current_command_;
  std::string new_command_;

  // ROS2 interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_manager

// ============================================================
// Main
// ============================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rover_manager::RoverManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
