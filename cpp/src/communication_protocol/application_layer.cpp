#include "communication_protocol/application_layer.h"

robot_command::robot_command(std::string input_command, std::string inputValue) : direction(input_command), value(inputValue) {}

ApplicationLayer::ApplicationLayer() { create_all_commands_map(); }

void ApplicationLayer::create_all_commands_map()
{
    _all_commands_map.clear();
    _all_commands_map.insert(_direction_map.begin(), _direction_map.end());
    _all_commands_map.insert(_value_map.begin(), _value_map.end());
};

void ApplicationLayer::add_direction(const std::string &key, const std::string &value)
{
    _direction_map[key] = value;
    _all_commands_map[key] = value;
}

void ApplicationLayer::add_value(const std::string &key, const std::string &value)
{
    _value_map[key] = value;
    _all_commands_map[key] = value;
}

void ApplicationLayer::add_command(const std::string &key, const std::string &value)
{
    _value_map[key] = value;
    _all_commands_map[key] = value;
}

std::string ApplicationLayer::command_to_bits(const robot_command &input_command)
{
    // put command = bits and then values integers er lig med bits;
    std::string final_bits_converted = "";

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

    std::string values = "";
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

std::string ApplicationLayer::command_vector_to_bitstream(std::vector<robot_command> &command_vector)
{

    std::string bitstream = "";

    for (const auto &command : command_vector)
    {
        bitstream += command_to_bits(command);
    }

    return bitstream;
}

bool ApplicationLayer::is_value(const std::string &bits)
{
    bool is_value = false;
    for (const auto &i : _value_map)
    {
        if (i.second == bits)
        {
            is_value = true;
        }
    }

    return is_value;
}

bool ApplicationLayer::is_direction(const std::string &bits)
{
    bool is_direction = false;
    for (const auto &i : _direction_map)
    {
        if (i.second == bits)
        {
            is_direction = true;
        }
    }

    return is_direction;
}

std::string ApplicationLayer::find_key(const std::string &value, const std::unordered_map<std::string, std::string> &map)
{
    for (const auto &i : map)
    {
        if (i.second == value)
        {
            return i.first;
        }
    }
    return "";
}

std::vector<robot_command> ApplicationLayer::bits_to_commands(std::string input_bits)
{

    std::vector<robot_command> command_vector;
    std::string temp_bits;

    std::string Value = "";
    size_t length = input_bits.length();
    std::string value_bits = "";

    for (int i = 0; i <= length; i += nibble_size)
    {
        temp_bits = input_bits.substr(i, nibble_size);
        std::string commandB = find_key(temp_bits, _direction_map);

        if (!commandB.empty())
        {
            Value = "";
            while (i + nibble_size <= length)
            {
                std::string value_bits = input_bits.substr(i + nibble_size, nibble_size);
                std::string valueB = find_key(value_bits, _value_map);

                if (!valueB.empty())
                {
                    Value += valueB;
                    i += nibble_size;
                }
                else
                {
                    break; // If it's not a value then this should be the end of the robot command according to current command rules
                }
            }

            command_vector.emplace_back(commandB, Value); // Creates objects of the struct to append to vector
        }
        else
        {
            break;
        }
    }
    return command_vector;
}

void ApplicationLayer::print_robot_commands(const std::vector<robot_command> &command_vector)
{
    for (const auto &selection : command_vector)
    {
        std::cout << "Command: " << selection.direction << "   Value: " << selection.value << " " << std::endl;
    }
    std::cout << std::endl;
}

std::string ApplicationLayer::encode_message(const std::string &message)
{
    return CRC::CRC32::encode(message);
}
