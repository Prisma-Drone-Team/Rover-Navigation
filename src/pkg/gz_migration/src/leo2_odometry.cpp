#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Odometry : public rclcpp::Node{
    public:
        Odometry() : Node("odometry") {

            //this->declare_parameter("use_sim_time", false);
            this->declare_parameter<std::string>("odom_frame", "robot2/odom");  //odom
            this->declare_parameter<std::string>("child_frame", "robot2/base_footprint");   //base_footprint
            odom_frame_ = this->get_parameter("odom_frame").as_string();
            child_frame_ = this->get_parameter("child_frame").as_string();

            // this->declare_parameter<double>("start_x", 5.0);
            // this->declare_parameter<double>("start_y", 0.0);
            // this->declare_parameter<double>("start_z", 0.0);
            // start_x = this->get_parameter("start_x").as_double();
            // start_y = this->get_parameter("start_y").as_double();
            // start_z = this->get_parameter("start_z").as_double();

            subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/model/leo2/pose", 10,   //model/leo/pose
                std::bind(&Odometry::topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot2/odom", 10);       //odom_sim
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            //timer_ = this->create_wall_timer(100ms, std::bind(&Odometry::topic_callback, this));

        }
    
    private:
        void topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
            auto now_ = this->get_clock()->now();
            //RCLCPP_INFO(this->get_logger(), "now: %f", now_.seconds());
            double start_x=-5.0; //-35
            double start_y=15.0; //35
            double start_z=11.0; //-2
            //geometry_msgs::msg::PoseStamped new_odom_;
            nav_msgs::msg::Odometry new_odom_;
            //tf2_msgs::msg::TFMessage odom_tf_;
            geometry_msgs::msg::TransformStamped odom_tf_;
            tf2_msgs::msg::TFMessage transform_;
            transform_.transforms = msg->transforms;

            //new_odom_.header = transform_.transforms[7].header;     //elemento 7 di transforms è la tf tra world frame e base_link
            new_odom_.header.stamp = now_;
            new_odom_.header.frame_id = odom_frame_;
            new_odom_.child_frame_id = transform_.transforms[7].child_frame_id;
            new_odom_.pose.pose.position.x = transform_.transforms[7].transform.translation.x - start_x;
            new_odom_.pose.pose.position.y = transform_.transforms[7].transform.translation.y - start_y;
            new_odom_.pose.pose.position.z = transform_.transforms[7].transform.translation.z - start_z;
            new_odom_.pose.pose.orientation = transform_.transforms[7].transform.rotation;
            new_odom_.pose.covariance =    {0.05, 0,    0,    0,    0,    0,           // x
                                            0,    0.05, 0,    0,    0,    0,        // y
                                            0,    0,    0.01, 0,    0,    0,        // z
                                            0,    0,    0,    0.1,  0,    0,        // roll
                                            0,    0,    0,    0,    0.1,  0,        // pitch
                                            0,    0,    0,    0,    0,    0.2};     // yaw            
            // new_odom_.pose.orientation.x = transform_.transforms[7].transform.rotation.x;
            // new_odom_.pose.orientation.y = transform_.transforms[7].transform.rotation.y;
            // new_odom_.pose.orientation.z = transform_.transforms[7].transform.rotation.z;
            // new_odom_.pose.orientation.w = transform_.transforms[7].transform.rotation.w;

                        // Calcolo della velocità
            if (last_transform_.header.stamp.sec > 0) { //vedo se c'è una trasformata precedente
                double dt = (this->now() - last_time_).seconds();
                if (dt > 0) {
                    Eigen::Vector3d prev_pos(last_transform_.transform.translation.x,
                                             last_transform_.transform.translation.y,
                                             last_transform_.transform.translation.z);
                    Eigen::Vector3d curr_pos(transform_.transforms[7].transform.translation.x,
                                             transform_.transforms[7].transform.translation.y,
                                             transform_.transforms[7].transform.translation.z);
                    Eigen::Vector3d velocity = (curr_pos - prev_pos) / dt;  // ds/dt

                    new_odom_.twist.twist.linear.x = velocity.x();
                    new_odom_.twist.twist.linear.y = velocity.y();
                    new_odom_.twist.twist.linear.z = velocity.z();

                    tf2::Quaternion q1(last_transform_.transform.rotation.x, last_transform_.transform.rotation.y,
                                       last_transform_.transform.rotation.z, last_transform_.transform.rotation.w);
                    tf2::Quaternion q2(transform_.transforms[7].transform.rotation.x, transform_.transforms[7].transform.rotation.y,
                                       transform_.transforms[7].transform.rotation.z, transform_.transforms[7].transform.rotation.w);
                    tf2::Quaternion q_delta = q1.inverse() * q2;
                    double angle = 2.0 * std::acos(q_delta.w());
                    tf2::Vector3 axis(q_delta.x(), q_delta.y(), q_delta.z());
                    if (axis.length() > 0) {
                        axis.normalize();
                        tf2::Vector3 angular_velocity = (axis * angle) / dt;
                        new_odom_.twist.twist.angular.x = angular_velocity.x();
                        new_odom_.twist.twist.angular.y = angular_velocity.y();
                        new_odom_.twist.twist.angular.z = angular_velocity.z();
                    }
                }
            }

            new_odom_.twist.covariance = {0.05, 0,    0,    0,    0,    0,           // x
                                             0,    0.05, 0,    0,    0,    0,        // y
                                             0,    0,    0.01, 0,    0,    0,        // z
                                             0,    0,    0,    0.1,  0,    0,        // roll
                                             0,    0,    0,    0,    0.1,  0,        // pitch
                                             0,    0,    0,    0,    0,    0.2};     // yaw  

            //tf tra odom e base_footprint
            odom_tf_.header.stamp = now_;
            odom_tf_.header.frame_id = odom_frame_;
            odom_tf_.child_frame_id = child_frame_;
            odom_tf_.transform.translation.x = new_odom_.pose.pose.position.x;
            odom_tf_.transform.translation.y = new_odom_.pose.pose.position.y;
            odom_tf_.transform.translation.z = new_odom_.pose.pose.position.z;
            odom_tf_.transform.rotation.x = new_odom_.pose.pose.orientation.x;
            odom_tf_.transform.rotation.y = new_odom_.pose.pose.orientation.y;
            odom_tf_.transform.rotation.z = new_odom_.pose.pose.orientation.z;
            odom_tf_.transform.rotation.w = new_odom_.pose.pose.orientation.w;
            // odom_tf_.transforms.transform.translation.x = transform_.transforms.transform.translation.x - start_x;
            // odom_tf_.transforms.transform.translation.y = transform_.transforms.transform.translation.y - start_y;
            // odom_tf_.transforms.transform.translation.z = transform_.transforms.transform.translation.z - start_z;
            // odom_tf_.transforms.transform.rotation.x = transform_.transforms.transform.rotation.x;
            // odom_tf_.transforms.transform.rotation.y = transform_.transforms.transform.rotation.y;
            // odom_tf_.transforms.transform.rotation.z = transform_.transforms.transform.rotation.z;
            // odom_tf_.transforms.transform.rotation.w = transform_.transforms.transform.rotation.w;

            publisher_->publish(new_odom_);
            tf_broadcaster_->sendTransform(odom_tf_);

        }

    std::string odom_frame_;
    std::string child_frame_;
    geometry_msgs::msg::TransformStamped last_transform_;
    rclcpp::Time last_time_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry>());
    rclcpp::shutdown();
    return 0;
}