#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <thread>
#include <chrono>

void drive_forward(ros::Publisher& pub, double duration, double speed = 0.2) {
    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = speed;  // Set forward speed

    pub.publish(move_cmd);  // Publish the velocity command
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));  // Sleep for the specified duration

    move_cmd.linear.x = 0.0;
    pub.publish(move_cmd);
}

void turn(ros::Publisher& pub, double duration, double speed = 0.5) {
    geometry_msgs::Twist turn_cmd;
    turn_cmd.angular.z = speed;  // Set rotational speed (positive for counterclockwise)

    pub.publish(turn_cmd);  // Publish the velocity command
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));  // Sleep for the specified duration

    turn_cmd.angular.z = 0.0;
    pub.publish(turn_cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drive_robot");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Duration(1.0).sleep();

    drive_forward(pub, 2.0, 0.2);

    turn(pub, 1.5, 0.5);

    drive_forward(pub, 2.0, 0.2);

    ros::spinOnce();

    return 0;
}



