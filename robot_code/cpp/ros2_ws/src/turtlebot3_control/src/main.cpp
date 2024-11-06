#include "move_turtlebot.cpp"  // Include header for movement control

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtlebot>();

    // 12: forward
    // 13: backward
    // 14: right
    // 15: left

    std::vector<std::vector<int>> sequence = {{12, 2, 0}, {13, 2, 0}, {14, 9, 0}, {15, 9, 0}};
    std::vector<std::vector<int>> table_sequence = {{12, 4, 0}, {15, 9, 0}, {15, 4, 5}, {12, 2, 5}, {14, 9, 0}, {12, 3, 0}};

    node->run_path(table_sequence);

    // std::cout << "Enter distance (cm) " << std::endl;
    // float distance;
    // std::cin >> distance;

    // node->move(distance);

    // std::cout << "Enter angle (degrees) " << std::endl;
    // float angle;
    // std::cin >> angle;

    // node->turn(angle);
    //node->move(20, 0.2);
    rclcpp::shutdown();
    return 0;
}
