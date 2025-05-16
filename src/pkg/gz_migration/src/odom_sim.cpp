#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

// using std::placeholders::_1;
using namespace std::chrono_literals;

class OdomSim : public rclcpp::Node {
public:
    OdomSim() : Node("odom_sim"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        this->declare_parameter<std::string>("parent_frame", "rover/map");
        this->declare_parameter<std::string>("child_frame", "base_footprint");

        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&OdomSim::publish_odometry, this));
    }

private:
    void publish_odometry() {
        try {
            auto transform = tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);
            auto now = this->get_clock()->now();

            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = now;
            odom_msg.header.frame_id = parent_frame_;
            odom_msg.child_frame_id = child_frame_;

            // Copio posizione e orientamento da tf2 a odom
            odom_msg.pose.pose.position.x = transform.transform.translation.x;
            odom_msg.pose.pose.position.y = transform.transform.translation.y;
            odom_msg.pose.pose.position.z = transform.transform.translation.z;
            odom_msg.pose.pose.orientation = transform.transform.rotation;

            // Covarianza della posa
            for (int i = 0; i < 36; i++) {
                odom_msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
            }

            // Calcolo della velocità
            if (last_transform_.header.stamp.sec > 0) { //vedo se c'è una trasformata precedente
                double dt = (now - last_time_).seconds();
                if (dt > 0) {
                    Eigen::Vector3d prev_pos(last_transform_.transform.translation.x,
                                             last_transform_.transform.translation.y,
                                             last_transform_.transform.translation.z);
                    Eigen::Vector3d curr_pos(transform.transform.translation.x,
                                             transform.transform.translation.y,
                                             transform.transform.translation.z);
                    Eigen::Vector3d velocity = (curr_pos - prev_pos) / dt;  // ds/dt

                    // odom_msg.twist.twist.linear.x = velocity.x();
                    // odom_msg.twist.twist.linear.y = velocity.y();
                    // odom_msg.twist.twist.linear.z = velocity.z();

                    // tf2::Quaternion q1(last_transform_.transform.rotation.x, last_transform_.transform.rotation.y,
                    //                    last_transform_.transform.rotation.z, last_transform_.transform.rotation.w);
                    // tf2::Quaternion q2(transform.transform.rotation.x, transform.transform.rotation.y,
                    //                    transform.transform.rotation.z, transform.transform.rotation.w);
                    // tf2::Quaternion q_delta = q1.inverse() * q2;
                    // double angle = 2.0 * std::acos(q_delta.w());
                    // tf2::Vector3 axis(q_delta.x(), q_delta.y(), q_delta.z());
                    // if (axis.length() > 0) {
                    //     axis.normalize();
                    //     tf2::Vector3 angular_velocity = (axis * angle) / dt;
                    //     odom_msg.twist.twist.angular.x = angular_velocity.x();
                    //     odom_msg.twist.twist.angular.y = angular_velocity.y();
                    //     odom_msg.twist.twist.angular.z = angular_velocity.z();
                    // }
                }
            }

            // Covarianza della velocità
            for (int i = 0; i < 36; i++) {
                odom_msg.twist.covariance[i] = (i % 7 == 0) ? 0.1 : 0.0;
            }

            odom_pub_->publish(odom_msg);

            last_transform_ = transform;
            last_time_ = now;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "Published odometry from %s to %s", child_frame_.c_str(), parent_frame_.c_str());

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF error: %s", ex.what());
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::msg::TransformStamped last_transform_;
    rclcpp::Time last_time_;

    std::string parent_frame_;
    std::string child_frame_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
