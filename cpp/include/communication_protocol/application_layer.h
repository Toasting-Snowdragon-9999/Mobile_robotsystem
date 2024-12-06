#ifndef APPLICATIONLAYER_H
#define APPLICATIONLAYER_H

#define nibble_size 4

#include <string>
#include <unordered_map>
#include <bitset>
#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>
#include <chrono>
#include "crc.h"


struct robot_command
{
    std::string direction;
    std::string value;

    robot_command(std::string input_command, std::string inputValue = "0");
};

class ApplicationLayer
{

private:
    std::unordered_map<std::string, std::string> _direction_map = {
        {"-fw", "1010"},
        {"-bw", "1011"},
        {"-r", "1100"},
        {"-l", "1101"},
    };

    std::unordered_map<std::string, std::string> _value_map = {
        {"0", "0000"},
        {"1", "0001"},
        {"2", "0010"},
        {"3", "0011"},
        {"4", "0100"},
        {"5", "0101"},
        {"6", "0110"},
        {"7", "0111"},
        {"8", "1000"},
        {"9", "1001"},
    };

    std::unordered_map<std::string, std::string> _all_commands_map;

    void create_all_commands_map();

public:
    ApplicationLayer();

    void add_direction(const std::string &key, const std::string &value);

    bool is_value(const std::string &bits);

    bool is_direction(const std::string &bits);

    std::string find_key(const std::string &value, const std::unordered_map<std::string, std::string> &map);

    void add_value(const std::string &key, const std::string &value);

    void add_command(const std::string &key, const std::string &value);

    std::string command_to_bits(const robot_command &input_command);

    std::string command_vector_to_bitstream(std::vector <robot_command> &command_vector);

    std::vector<robot_command> bits_to_commands(std::string input_bits);

    // Function to print all robot_command objects in a vector
    void print_robot_commands(const std::vector<robot_command> &command_vector);

    std::string encode_message(const std::string &message);

    void start_ack_timer();
};

#endif // APPLICATIONLAYER_H
