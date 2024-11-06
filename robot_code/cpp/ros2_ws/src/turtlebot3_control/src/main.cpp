#include "move_turtlebot.cpp"  // Include header for movement control

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtlebot>();
    
    //node->move(100, 0.1);
    node->turn(45, 0.1);
    rclcpp::shutdown();
    return 0;
}
