#include "move_turtlebot.cpp"  // Include header for movement control

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtlebot>();
    
    node->move(100, 0.5); 
    // node->turn(90, 0.5);   
    rclcpp::shutdown();
    return 0;
}
