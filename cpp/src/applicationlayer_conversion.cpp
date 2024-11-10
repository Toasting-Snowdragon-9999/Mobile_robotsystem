#include "applicationlayer_conversion.h"

using std::string;

string ApplicationlayerConversion::command_to_bits(const robot_command &input_command)
{
    // put command = bits and then values integers er lig med bits;
    string final_bits_converted = "";
    if (input_command.command != "-fw" && input_command.command != "-bw" && input_command.command != "-r" && input_command.command != "-l")
    {
        return "Error: Input_command's directional command is invalid";
    }
    else if (input_command.value[0] == '-')
    {
        return "Error: Input_command's value is negative, which is invalid";
    }
    else
    {
        final_bits_converted.append(_commandsMap.find(input_command.command)->second);
        string input_val = input_command.value;

        for (int i = 0; i < input_val.length(); i++)
        {
            unsigned long int_val = input_val[i] - '0';
            std::bitset<4> temp{int_val};
            string appendstring = temp.to_string();
            final_bits_converted.append(appendstring);
        }

        return final_bits_converted;
    }
}

robot_command ApplicationlayerConversion::bits_to_command(string inputBits)
{
    {
        for (const auto &commands : _commandsMap)
        {
            if (commands.second == inputBits)
            {
                remove;
            };
        }
    }
}