#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <ros_gz_interfaces/msg/Pose_V.hpp>
//#include <pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <chrono>

class OdomSim : public rclcpp::Node {
public:
    OdomSim() : Node("odom_sim") {
        // Parametri del nodo
        // this->declare_parameter("robot_name", "robot");
        // this->get_parameter("robot_name", robot_name_);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Sottoscrizione alla pose di Gazebo
        pose_sub_ = this->create_subscription<ros_gz_interfaces::msg::Pose_V>(
            "/world/default/dynamic_pose/info", 10,
            [this](const ros_gz_interfaces::msg::Pose_V::SharedPtr msg) { 
            this->poseCallback(msg);
            }
        );

        last_time_ = this->now();
    }

private:
    void poseCallback(const ros_gz_interfaces::msg::Pose_V::SharedPtr msg) {
        // Trova la pose del robot
        ros_gz_interfaces::msg::Pose robot_pose;
        bool found = false;
        for (const auto &pose : msg->poses) {
            if (pose.name == robot_name_) {
                robot_pose = pose;
                found = true;
                break;
            }
        }

        if (!found) {
            RCLCPP_WARN(this->get_logger(), "Pose del robot '%s' non trovata!", robot_name_.c_str());
            return;
        }

        // Estrai posizione e orientazione
        double x = robot_pose.position.x;
        double y = robot_pose.position.y;
        double z = robot_pose.position.z;
        double qx = robot_pose.orientation.x;
        double qy = robot_pose.orientation.y;
        double qz = robot_pose.orientation.z;
        double qw = robot_pose.orientation.w;

        // Calcola il tempo trascorso
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt == 0) return;  // Evita divisioni per zero

        // Calcola velocità lineari
        double vx = (x - last_x_) / dt;
        double vy = (y - last_y_) / dt;

        // Calcola velocità angolare (approssimata usando la quaternione)
        double v_yaw = 2.0 * (qw * qz + qx * qy) / (1.0 - 2.0 * (qy * qy + qz * qz));
        v_yaw = (v_yaw - last_yaw_) / dt;

        // Salva lo stato attuale
        last_x_ = x;
        last_y_ = y;
        last_yaw_ = v_yaw;

        // Costruisci il messaggio di odometria
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;
        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = v_yaw;

        // Pubblica l'odometria
        odom_pub_->publish(odom_msg);

        // Pubblica la trasformazione TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = z;
        tf_msg.transform.rotation.x = qx;
        tf_msg.transform.rotation.y = qy;
        tf_msg.transform.rotation.z = qz;
        tf_msg.transform.rotation.w = qw;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    // Variabili di stato
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Pose_V>::SharedPtr pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::string robot_name_;
    rclcpp::Time last_time_;
    double last_x_ = 0.0, last_y_ = 0.0, last_yaw_ = 0.0;
};

// Main
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomSim>());
    rclcpp::shutdown();
    return 0;
}

