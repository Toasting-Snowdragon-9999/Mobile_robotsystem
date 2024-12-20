#ifndef MOVE_TURTLEBOT_HPP
#define MOVE_TURTLEBOT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class MoveTurtlebot : public rclcpp::Node {
public:
    MoveTurtlebot() : Node("move_turtlebot"){
        _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);   

    }

    void move(int distance_cm) {
        float distance_m = distance_cm / 100.0;
        float time = std::abs(distance_m / _velocity);         // dist = speed * time
        auto message = geometry_msgs::msg::Twist();

        float temp_velocity = _velocity;

        if(distance_m < 0){
            temp_velocity = -temp_velocity;
        }

        //float acceleration = 0.5;
        // float acceleration_distance = (_velocity * _velocity) / (2 * acceleration);
        // float total_time;

        // if(2 * acceleration_distance >= distance_m){
        //     float acceleration_time = std::sqrt(2 * distance_m /acceleration);
        //     total_time = 2 * acceleration_time;
        // } else{
        //     float acceleration_time = _velocity / acceleration;
        //     float const_velocity_time = (distance_m - 2 * acceleration_distance) / _velocity;
        //     float deacceleration_time = _velocity / acceleration;

        //     total_time = acceleration_time + const_velocity_time + deacceleration_time;
        // }

//        std::cout << "\nDEBUGGING MOVE" << std::endl;
        //std::cout << "Acceleration: " << acceleration<< std::endl;
//        std::cout << "Distance in centimeters: " << distance_cm << std::endl;
//        std::cout << "Distance in meters: " << distance_m << std::endl;
//        std::cout << "Velocity: " << temp_velocity << std::endl;
        //std::cout << "Fancy time for move: " << total_time << std::endl;
//        std::cout << "Time for move: " << time << std::endl;

        message.linear.x = temp_velocity; //  m/s
        message.angular.z = 0.0;

//        RCLCPP_INFO(this->get_logger(), "Moving the TurtleBot!");
        auto start_time = this->now();
        rclcpp::Duration duration = rclcpp::Duration::from_seconds(time);
        while ((this->now() - start_time) < duration) {
            _publisher->publish(message);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        message.linear.x = 0.0;
        _publisher->publish(message);
    }

    void turn(int angle_degree) {
        float angle_radians = angle_degree * (M_PI / 180.0);
        auto message = geometry_msgs::msg::Twist();
        float time = std::abs(angle_radians / _angular_velocity);         // angle = _angular_velocity * time

        float temp_angular_velocity = _angular_velocity;
        if(angle_degree < 0){
            temp_angular_velocity = -temp_angular_velocity;
        }

        /* DEBUG */
//        std::cout << "\nDEBUGGING" << std::endl;
//        std::cout << "Angle in degrees: " << angle_degree << std::endl;
//        std::cout << "Angle in radians: " << angle_radians << std::endl;
//        std::cout << "Angular velocity: " << temp_angular_velocity << std::endl;
//        std::cout << "Time for turn: " << time << std::endl;


        message.linear.x = 0.0;
        message.angular.z = temp_angular_velocity;

//        RCLCPP_INFO(this->get_logger(), "Turning the TurtleBot!");
        auto start_time = this->now();
        rclcpp::Duration duration = rclcpp::Duration::from_seconds(time);
        rclcpp::Rate rate(100);  // 100 Hz control rate

        // Rotate for the calculated duration
        while ((this->now() - start_time) < duration) {
            _publisher->publish(message);
            rate.sleep();  // This will keep the loop running at ~100 Hz
        }

        // Ensure the bot stops rotating
        message.angular.z = 0.0;
        for (int i = 0; i < 5; ++i) {  // Publish the stop message multiple times
            _publisher->publish(message);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }

//        RCLCPP_INFO(this->get_logger(), "Rotation completed.");
    }

    void run_path(std::vector<std::vector<std::string>> &sequence){

        /* maybe this works? */
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        /* */

        for(auto command : sequence){

            std::string action = command[0];
            int length = std::stoll(command[1]);

            /* Debug purposes */
            std::string action_text;
            if(action == "-fw"){
                action_text = "forward";
            }
            if(action == "-bw"){
                action_text = "backward";
            }
            if(action == "-r"){
                action_text = "right";
            }
            if(action == "-l"){
                action_text = "left";
            }
            /* Debug purposes end */

            // If it's backwards or turn right
            if(action == "-bw" || action == "-r"){
                length = -length;
            }
            // If it's move -fw or move -bw
            if(action == "-fw" || action == "-bw"){
                move(length);
            }
            // If it's turn -l or turn -r
            if(action == "-l" || action == "-r"){ //apparantly 14 is right and 15 is left. OCD has left the building
                turn(length);
            }

            /* Debug purposes */
//            std::cout << "\n     MORE DEBUGGING" << std::endl;
//            std::cout << "     Action: " + action_text + ", " << action << std::endl;
//            std::cout << "     Length: " << length << std::endl;
            /* Debug purposes end */

            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

private:

   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;

    float _angular_velocity = 1;
    float _velocity = 0.15;

};

#endif
