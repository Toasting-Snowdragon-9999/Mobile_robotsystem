#include "com_protocol.h"
#include "crc.h"

ComProtocol::ComProtocol(std::string robot_path) : _robot_path(robot_path) {}

std::string ComProtocol::length_of_string(std::string s)
{
    int length_of_path = s.size();
    std::string binary_length = "";

    if (length_of_path == 0)
    {
        binary_length += '0'; // If a tone is zero, no conversion is needed
    }

    else
    {
        while (length_of_path > 0)
        {
            binary_length += (length_of_path % 2) ? '1' : '0';

            length_of_path /= 2;
        }
    }

    std::reverse(binary_length.begin(), binary_length.end());

    return binary_length;
}

// THIS NEEDS REMODELING
std::string ComProtocol::protocol_structure()
{
    std::string length_of_path = length_of_string(_robot_path);

    std::stringstream ss_header_and_data;
    ss_header_and_data << _SFD << length_of_path << _EFD << _robot_path;
    std::string header_and_data = ss_header_and_data.str();

    std::string zero_padded_header_and_data = zero_pad(header_and_data);

    std::string crc_encoded = CRC::CRC16::encode(zero_padded_header_and_data);

    std::stringstream creating_package;
    creating_package << _pre_and_postamble
                     << crc_encoded
                     << _pre_and_postamble;

    std::string full_package = creating_package.str();
    return full_package;
}

std::string ComProtocol::zero_pad(std::string binary_msg)
{
    int length_of_msg = binary_msg.size();

    int zeros = 4 - (length_of_msg % 4);
    if(zeros == 4) { zeros = 0; }

    for(int i = 0; i < zeros; i++)
    {
        binary_msg += '0';
    }

    return binary_msg;
}

string ComProtocol::decimal_seq_to_binary_msg(const std::vector<std::vector<int>> &decimalSequence)
{
    string binaryConvertedTone = "";
    std::string tmpTone;

    for (auto pair : decimalSequence)
    {
        for (auto tone : pair)
        {
            uint16_t toneCopy = tone;
            if (tone == 0)
            {
                tmpTone += "0000"; // If a tone is zero, no conversion is needed
            }
            else
            {

                while (toneCopy > 0)
                {
                    tmpTone += (toneCopy % 2) ? '1' : '0';

                    toneCopy /= 2;
                }

                while (tmpTone.length() < 4)
                {
                    tmpTone += '0';
                }
            }

            std::reverse(tmpTone.begin(), tmpTone.end());
            binaryConvertedTone += tmpTone;
            tmpTone.clear();
        }
    }

    return binaryConvertedTone;
}

// bool ComProtocol::is_message_correct(const string &decoded_remainder)
// {
//     return find_remainder(decoded_remainder) == "00000000";
// }

// // THIS NEEDS REMODELING
// std::string ComProtocol::find_remainder(std::string dataword)
// {
//     dataword = dataword.substr(dataword.size() - 8);
//     return dataword;
// }

// // THIS NEEDS REMODELING
// std::string ComProtocol::get_binary_message_from_package(std::vector<std::vector<int>> package)
// {
//     package[0].erase(package[0].begin(), package[0].begin() + 1); // Removes preamble, length
//     package[0].erase(package[0].end() - 3, package[0].end());     // Removes postamble

//     std::string binaryEncodedMsg = decimal_seq_to_binary_msg(package);
//     std::string binaryDecodedMsg = crc16_decode(binaryEncodedMsg);

//     if (find_remainder(binaryDecodedMsg) == "0000000000000000") // If message is correct return it
//     {
//         binaryDecodedMsg.erase(binaryDecodedMsg.end() - (4 * 2), binaryDecodedMsg.end()); // Removes CRC from end of message
//         return binaryDecodedMsg;
//     }
//     else // If message is not correct return 0 and print error
//     {
//         std::cout << "The recieved message IS NOT correct" << std::endl;
//         return 0;
//     }
// }

std::string ComProtocol::remove_pre_and_postamble(std::string received_package)
{
    int length_of_pre_and_postamble = _pre_and_postamble.size();

    received_package.erase(received_package.begin(), received_package.begin() + length_of_pre_and_postamble);
    received_package.erase(received_package.end() - length_of_pre_and_postamble, received_package.end());

    return received_package;
}