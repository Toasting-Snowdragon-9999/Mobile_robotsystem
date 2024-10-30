#ifndef MOVE_TURTLEBOT_HPP
#define MOVE_TURTLEBOT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread> 
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include "turtlebot3_control/srv/get_yaw.hpp"

using namespace std::chrono_literals;

class MoveTurtlebot : public rclcpp::Node {
public:
    MoveTurtlebot() : Node("move_turtlebot"){
        _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);   
        
        // _odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        // "/odom", 10, std::bind(&MoveTurtlebot::odom_callback, this, std::placeholders::_1));

    }

    void move(int distance_cm, float velocity = 0.5) {
        float distance_m = distance_cm / 100.0;
        float time = distance_m / velocity;         // dist = speed * time
        auto message = geometry_msgs::msg::Twist();

        message.linear.x = velocity; //  m/s          
        message.angular.z = 0.0;                    

        RCLCPP_INFO(this->get_logger(), "Moving the TurtleBot!");
        auto start_time = this->now();
        rclcpp::Duration duration = rclcpp::Duration::from_seconds(time);
        while ((this->now() - start_time) < duration) {
            _publisher->publish(message);
            rclcpp::sleep_for(std::chrono::milliseconds(10));  
        }
        message.linear.x = 0.0;
        _publisher->publish(message);
    }

        // void turn(int angle_degree, float angular_velocity = 0.5) {
        //     float angle_radians = angle_degree * (M_PI / 180.0); 
        //     auto message = geometry_msgs::msg::Twist();
        //     float time = angle_radians / angular_velocity;         // angle = angular_velocity * time

        //     message.linear.x = 0.0;               
        //     message.angular.z = angular_velocity;                    

        //     RCLCPP_INFO(this->get_logger(), "Turning the TurtleBot!");
        //     auto start_time = this->now();
        //     rclcpp::Duration duration = rclcpp::Duration::from_seconds(time);
        //     rclcpp::Rate rate(100);  // 100 Hz control rate

        //     // Rotate for the calculated duration
        //     while ((this->now() - start_time) < duration) {
        //         _publisher->publish(message);
        //         rate.sleep();  // This will keep the loop running at ~100 Hz
        //     }

        //     // Ensure the bot stops rotating
        //     message.angular.z = 0.0;
        //     for (int i = 0; i < 5; ++i) {  // Publish the stop message multiple times
        //         _publisher->publish(message);
        //         rclcpp::sleep_for(std::chrono::milliseconds(10));
        //     }

        //     RCLCPP_INFO(this->get_logger(), "Rotation completed.");
        // }



    // void turn(int angle_degree, float angular_velocity = 0.5) {
    //     float target_angle = angle_degree * (M_PI / 180.0); 
    //     _initial_yaw = _current_yaw;  
    //     auto message = geometry_msgs::msg::Twist();
    //     message.angular.z = angular_velocity;

    //     RCLCPP_INFO(this->get_logger(), "Starting rotation...");
    //     auto start_time = this->now();
    //     rclcpp::Duration duration = rclcpp::Duration::from_seconds(20);
    //     rclcpp::Rate rate(100);  // Loop at 100 Hz
    //     const char* mes = "Rotation completed.";
    //     while (std::abs(_current_yaw - _initial_yaw) < std::abs(target_angle)) {
    //         if((this->now() - start_time) > duration){
    //             mes = "Rotation failed.";
    //             break;
    //         }
    //         _publisher->publish(message);
    //         printf("Current yaw: %f\n", _current_yaw);
    //         rate.sleep();
            
    //     }

    //     message.angular.z = 0.0;
    //     for (int i = 0; i < 5; ++i) {  // Publish the stop command multiple times
    //         _publisher->publish(message);
    //         rclcpp::sleep_for(std::chrono::milliseconds(10));
    //     }

    //     RCLCPP_INFO(this->get_logger(), mes);
    // }

private: 
    private:
        // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscriber;
        // double _current_yaw = 0.0;
        // double _initial_yaw = 0.0;

        // void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //     tf2::Quaternion quat;
        //     tf2::fromMsg(msg->pose.pose.orientation, quat);
        //     tf2::Matrix3x3 mat(quat);
        //     double roll, pitch;
        //     mat.getRPY(roll, pitch, _current_yaw);  // Update _current_yaw
        //     RCLCPP_INFO(this->get_logger(), "Updated Yaw: %f", _current_yaw);
        // }


        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
};

#endif
