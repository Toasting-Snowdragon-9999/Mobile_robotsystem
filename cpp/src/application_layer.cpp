#include "application_layer.h"

using std::string;

// Necessities: the command can be -fw, -bw, -l, -r or s for space
string ApplicationLayer::command_to_bits(const robot_command &input_command)
{
    // put command = bits and then values integers er lig med bits;
    string final_bits_converted = "";

    int length_of_value = input_command.value.length();

    bool is_command_found = false;
    bool is_value_found = false;

    for (auto command : _commandsMap)
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
        for (auto command : _commandsMap)
        {
            std::string char_to_str(1, input_command.value[i]);
            if (char_to_str == command.first)
            {
                is_command_found = true;
                values += command.second;
            }
        }
    }

    final_bits_converted += values;

    if (!is_command_found)
    {
        return "Error: Input_command is invalid and not found in mapping of commands";
    }

    return final_bits_converted;
}
// Necessities: only the data is put as input, and the commands have to still be separated by space bits
std::vector<robot_command> ApplicationLayer::bits_to_commands(string input_bits)
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

    // Final check
    if (!input_command.empty())
    {
        command_vector.emplace_back(input_command, value);
    }
    return command_vector;
}


string ApplicationLayer::crc7_encode(string binaryDataword)
{
    string codeword = "110100111";
    string encodedBinaryData = binaryDataword;

    int crcDegree = codeword.length() - 1;

    int selectionPlusOneIdx = codeword.length();
    int codewordLength = codeword.length();

    string binaryDatawordWithZeroes = binaryDataword + string(crcDegree, '0'); // Append CRC-Degree zeroes to data

    string selection = binaryDatawordWithZeroes.substr(0, codeword.length());
    int datawordLength = binaryDatawordWithZeroes.length();

    while (selectionPlusOneIdx < datawordLength) // Binary-division
    {
        if (selection[0] == '1')
        {
            selection = exclusive_or_strings(selection, codeword);
        }

        selection = selection.substr(1) + binaryDatawordWithZeroes[selectionPlusOneIdx];
        selectionPlusOneIdx++;
    }

    if ((selection[0] == '1')) // XOR the last selection with the codeword if needed
    {
        selection = exclusive_or_strings(selection, codeword);
    }

    string remainder = selection.substr(1); // Return substring since codeword is CRC-Degree+1 in size

    return encodedBinaryData = binaryDataword + remainder;
}

string ApplicationLayer::crc7_decode(string binaryEncodedDataword)
{
    string codeword = "110100111";
    string decodedBinaryData = binaryEncodedDataword;

    int crcDegree = codeword.length() - 1;

    int selectionPlusOneIdx = codeword.length();
    int codewordLength = codeword.length();

    string selection = binaryEncodedDataword.substr(0, codeword.length());
    int encodedDatawordLength = binaryEncodedDataword.length();

    while (selectionPlusOneIdx < encodedDatawordLength) // Binary-division
    {
        if (selection[0] == '1')
        {
            selection = exclusive_or_strings(selection, codeword);
        }

        selection = selection.substr(1) + binaryEncodedDataword[selectionPlusOneIdx];
        selectionPlusOneIdx++;
    }

    if ((selection[0] == '1')) // XOR the last selection with the codeword if needed
    {
        selection = exclusive_or_strings(selection, codeword);
    }

    string remainder = selection.substr(1); // Return substring since codeword is CRC-Degree+1 in size

    return decodedBinaryData = binaryEncodedDataword + remainder;
}

void ApplicationLayer::print_robot_commands(const std::vector<robot_command> &command_vector)
{
    for (const auto &selection : command_vector)
    {
        std::cout << selection.direction << " " << selection.value << " , ";
    }
    std::cout << std::endl;
}