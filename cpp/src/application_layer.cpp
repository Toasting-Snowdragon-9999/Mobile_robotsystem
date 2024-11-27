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

bool ApplicationLayer::is_value(const string &bits)
{
    bool is_value = false;
    for (const auto &i : _value_map)
    {
        if (i.second == bits)
        {
            is_value = true;
        }
    }
}

bool ApplicationLayer::is_direction(const string &bits)
{
    bool is_direction = false;
    for (const auto &i : _direction_map)
    {
        if (i.second == bits)
        {
            is_direction = true;
        }
    }
}

std::vector<robot_command> ApplicationLayer::bits_to_commands(string input_bits)
{
    int i;
    string temp_bits = input_bits.substr(0, 4);
    string next_bits = input_bits.substr(0 + 4, 4);

    std::unordered_map<std::string, std::string>::iterator iterator;

    if (i + 4 > input_bits.size() - 1)
    {
        // yadayada
    }

    // Check if bits are a value
    for (const auto &i : _value_map)
    {
        if (i.second == temp_bits)
        {
            iterator = _value_map.find(i.first);
        }
    }
    // Check if bits are a direction
    for (const auto &i : _direction_map)
    {
        if (i.second == temp_bits)
        {
            iterator = _direction_map.find(i.first);
        }
    }

    // Check if next bits are a value or a direction
    if (ApplicationLayer::is_value(temp_bits) && ApplicationLayer::is_direction(next_bits))
    {
        // emplace or sum shet
    }

    std::vector<robot_command> command_vector;

    string input_command = "";
    string value = "";
    size_t pos = 0;

    bool is_command = false;
    bool is_command_once = false;
    for (int i = 0; i < input_bits.length(); i += 4)
    {
        std::string temp_bits = input_bits.substr(i, 4);
        string next_bits = input_bits.substr(i + 4, 4);

        if (i + 4 > input_bits.length() - 1)
        {
            break;
        }

        if (!is_command_once)
        {
            for (const auto &commands : _all_commands_map)
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
            if (ApplicationLayer::is_value(temp_bits) && ApplicationLayer::is_direction(next_bits))
            {
                command_vector.emplace_back(input_command, value); // Creates objects of the struct to append to vector
                input_command = "";
                value = "";
                is_command = false;
                is_command_once = false;
                continue;
            }
            else
            {
                for (const auto &command : _value_map)
                {
                    if (command.second == temp_bits)
                    {
                        value += command.first;
                    }
                }
            }
        }
    }

    // Final check
    if (!input_command.empty())
    {
        command_vector.emplace_back(input_command, value);
    }
    return command_vector;
}

void ApplicationLayer::print_robot_commands(const std::vector<robot_command> &command_vector)
{
    for (const auto &selection : command_vector)
    {
        std::cout << selection.direction << " " << selection.value << " , ";
    }
    std::cout << std::endl;
}