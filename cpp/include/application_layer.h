#ifndef APPLICATIONLAYER_H
#define APPLICATIONLAYER_H

#include <string>
#include <unordered_map>
#include <bitset>
#include <iostream>
#include <vector>
#include <algorithm>

using std::string;

struct robot_command
{
    string direction;
    string value;

    robot_command(string input_command, string inputValue = "0") : direction(input_command), value(inputValue) {}
};

class ApplicationLayer
{

private:
    std::unordered_map<string, string> _commandsMap = {
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
        {"-fw", "1010"},
        {"-bw", "1011"},
        {"-r", "1100"},
        {"-l", "1101"},
    };

public:
    ApplicationLayer() {}

    string command_to_bits(const robot_command &input_command);

    std::vector<robot_command> bits_to_commands(string input_bits);

    string crc7_encode(string binaryDataword);

    string crc7_decode(string binaryDataword);

    // Function to print all robot_command objects in a vector
    void print_robot_commands(const std::vector<robot_command> &command_vector);
};

#endif // APPLICATIONLAYER_H
