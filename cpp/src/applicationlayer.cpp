#include "applicationlayer_conversion.h"

using std::string;



// Necessities: the command can be -fw, -bw, -l, -r or s for space
string ApplicationlayerConversion::command_to_bits(const robot_command &input_command)
{
    // put command = bits and then values integers er lig med bits;
    string final_bits_converted = "";

    if (input_command.direction != "-fw" && input_command.direction != "-bw" && input_command.direction != "-r" && input_command.direction != "-l" && input_command.direction != "s")
    {
        return "Error: Input_command's directional command character is invalid";
    }
    else if (input_command.value[0] == '-')
    {
        return "Error: Input_command's value is negative, which is invalid";
    }
    else
    {
        final_bits_converted.append(_commandsMap.find(input_command.direction)->second);
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
// Necessities: only the data is put as input, and the commands have to still be separated by space bits
std::vector<robot_command> ApplicationlayerConversion::bits_to_commands(string input_bits)
{
    std::vector<robot_command> command_vector;

    string input_command = "";
    string value = "";
    size_t pos = 0;

    string space_bits = _commandsMap["s"];

    // Erase spaces
    // while ((pos = input_bits.find(space_bits, pos)) != string::npos)
    // {
    //     input_bits.erase(pos, space_bits.length());
    // }

    bool is_command = false;
    bool is_command_once = false;
    for (int i = 0; i < input_bits.length(); i += 4)
    {
        std::string temp_bits = input_bits.substr(i, 4);

        if (i + 4 > input_bits.length())
        {
            break;
        }

        if (!is_command_once)
        {
            for (const auto &commands : _commandsMap)
            {
                if (commands.second == temp_bits)
                {
                    input_command = commands.first;
                    is_command = true;
                    break;
                }
            }
        }

        if (is_command && !is_command_once)
        {
            is_command_once = true;
            continue;
        }

        if (is_command && is_command_once)
        { // If space then add to vector and reset
            if (temp_bits == space_bits)
            {
                command_vector.emplace_back(input_command, value);
                input_command = "";
                value = "";
                is_command = false;
                is_command_once = false;
                continue;
            }
            else
            {
                int int_value = std::stoi(temp_bits, nullptr, 2);
                value += std::to_string(int_value);
            }
        }
    }

    //Final check
    if (!input_command.empty())
    {
        command_vector.emplace_back(input_command, value);
    }
    return command_vector;
}

void ApplicationlayerConversion::print_robot_commands(const std::vector<robot_command> &command_vector)
{
    for (const auto &selection : command_vector)
    {
        std::cout << selection.direction << " " << selection.value << " , ";
    }
    std::cout << std::endl;
}