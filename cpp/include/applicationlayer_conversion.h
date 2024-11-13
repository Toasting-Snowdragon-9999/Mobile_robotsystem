#ifndef APPLICATIONLAYER_CONVERSION_H
#define APPLICATIONLAYER_CONVERSION_H

#include <string>
#include <unordered_map>
#include <bitset>
#include <iostream>
#include <vector>

using std::string;

struct robot_command
{
    string direction;
    string value;

    robot_command(string input_command, string inputValue="0") : direction(input_command), value(inputValue) {}
};

class ApplicationlayerConversion
{

private:
    std::unordered_map<string, string> _commandsMap = {
        {"-fw", "1100"},
        {"-bw", "1101"},
        {"-r", "1110"},
        {"-l", "1111"},
        {"s","1011"}};

public:
    ApplicationlayerConversion() {}

    string command_to_bits(const robot_command &input_command);

    std::vector <robot_command> bits_to_commands(string input_bits);

    // Function to print all robot_command objects in a vector
void print_robot_commands(const std::vector<robot_command>& command_vector);

};

#endif
