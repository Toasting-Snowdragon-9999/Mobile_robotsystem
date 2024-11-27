#include "application_layer.h"

using std::string;

robot_command::robot_command(string input_command, string inputValue = "0") : direction(input_command), value(inputValue) {}

ApplicationLayer::ApplicationLayer() { create_all_commands_map(); }

void ApplicationLayer::create_all_commands_map()
{
    _all_commands_map.clear();
    _all_commands_map.insert(_direction_map.begin(), _direction_map.end());
    _all_commands_map.insert(_value_map.begin(), _value_map.end());
};

void ApplicationLayer::add_direction(const string &key, const std::string &value)
{
    _direction_map[key] = value;
    _all_commands_map[key] = value;
}

void ApplicationLayer::add_value(const string &key, const std::string &value)
{
    _value_map[key] = value;
    _all_commands_map[key] = value;
}

void ApplicationLayer::add_command(const string &key, const std::string &value)
{
    _value_map[key] = value;
    _all_commands_map[key] = value;
}

string ApplicationLayer::command_to_bits(const robot_command &input_command)
{
    // put command = bits and then values integers er lig med bits;
    string final_bits_converted = "";

    int length_of_value = input_command.value.length();

    bool is_command_found = false;
    bool is_value_found = false;

    for (auto command : _all_commands_map)
    {

        if (input_command.direction == command.first)
        {
            is_command_found = true;
            final_bits_converted += command.second;
        }
    }

    string values = "";
    for (int i = 0; i < length_of_value; i++)
    {
        for (auto command : _all_commands_map)
        {
            std::string char_to_str(1, input_command.value[i]);
            if (char_to_str == command.first)
            {
                is_value_found = true;
                values += command.second;
            }
        }
    }

    final_bits_converted += values;

    if (!is_command_found || !is_value_found)
    {
        return "Error: Input_command is invalid and not found in mapping of commands";
    }

    return final_bits_converted;
}

// Necessities: only the data is put as input
// std::vector<robot_command> ApplicationLayer::bits_to_commands(string input_bits)
// {
//     std::vector<robot_command> command_vector;

//     string input_command = "";
//     string value = "";
//     size_t pos = 0;

//     string space_bits = _all_commands_map["s"];

//     // Erase spaces
//     // while ((pos = input_bits.find(space_bits, pos)) != string::npos)
//     // {
//     //     input_bits.erase(pos, space_bits.length());
//     // }

//     bool is_command = false;
//     bool is_command_once = false;
//     for (int i = 0; i < input_bits.length(); i += 4)
//     {
//         std::string temp_bits = input_bits.substr(i, 4);

//         if (i + 4 > input_bits.length())
//         {
//             break;
//         }

//         if (!is_command_once)
//         {
//             for (const auto &commands : _all_commands_map)
//             {
//                 if (commands.second == temp_bits)
//                 {
//                     input_command = commands.first;
//                     is_command = true;
//                     break;
//                 }
//             }
//         }

//         if (is_command && !is_command_once)
//         {
//             is_command_once = true;
//             continue;
//         }

//         if (is_command && is_command_once)
//         { // If space then add to vector and reset
//             if (temp_bits == space_bits)
//             {
//                 command_vector.emplace_back(input_command, value); // Creates objects of the struct to append to vector
//                 input_command = "";
//                 value = "";
//                 is_command = false;
//                 is_command_once = false;
//                 continue;
//             }
//             else
//             {
//                 int int_value = std::stoi(temp_bits, nullptr, 2);
//                 value += std::to_string(int_value);
//             }
//         }
//     }

//     // Final check
//     if (!input_command.empty())
//     {
//         command_vector.emplace_back(input_command, value);
//     }
//     return command_vector;
// }

std::vector<robot_command> ApplicationLayer::bits_to_commands(string input_bits)
{

    std::vector<robot_command> command_vector;

    string input_command = "";
    string value = "";
    size_t pos = 0;

    for (int i = 0; i < input_bits.length(); i += 4)
    {
        std::string temp_bits = input_bits.substr(i, 4);

        std::unordered_map<string, string> value_map;

        value_map.insert )

        if (temp_bits)
    }
}

void ApplicationLayer::print_robot_commands(const std::vector<robot_command> &command_vector)
{
    for (const auto &selection : command_vector)
    {
        std::cout << selection.direction << " " << selection.value << " , ";
    }
    std::cout << std::endl;
}