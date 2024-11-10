#ifndef APPLICATIONLAYER_CONVERSION_H
#define APPLICATIONLAYER_CONVERSION_H

#include <string>
#include <unordered_map>
#include <bitset>
#include <iostream>

using std::string;

struct robot_command
{
    string command;
    string value;

    robot_command(string input_command, string inputValue) : command(input_command), value(inputValue) {}
};

class ApplicationlayerConversion
{

private:
    std::unordered_map<string, string> _commandsMap = {
        {"-fw", "1100"},
        {"-bw", "1101"},
        {"-r", "1110"},
        {"-l", "1111"}};

public:
    ApplicationlayerConversion() {}

    string command_to_bits(const robot_command &input_command);

    robot_command bits_to_command(string inputBits)
    {
        for (const auto &commands : _commandsMap)
        {
            if (commands.second == inputBits)
            {
            }
        }
    }
};

#endif
