#include "com_protocol.h"
#include "crc.h"

ComProtocol::ComProtocol(std::string robot_path) : _robot_path(robot_path) {}

std::string ComProtocol::length_of_string(std::string s)
{
    int length_of_string = s.size();
    std::string binary_length = "";

    if (length_of_string == 0)
    {
        binary_length += '0'; // If a tone is zero, no conversion is needed
    }

    else
    {
        while (length_of_string > 0)
        {
            binary_length += (length_of_string % 2) ? '1' : '0';

            length_of_string /= 2;
        }
    }

    std::reverse(binary_length.begin(), binary_length.end());

    return binary_length;
}

std::string ComProtocol::protocol_structure()
{
    std::string length_of_path = length_of_string(_robot_path);

    std::stringstream ss_header_and_data;
    ss_header_and_data << _SFD << length_of_path << _EFD << _robot_path;
    std::string header_and_data = ss_header_and_data.str();

    std::string zero_padded_header_and_data = zero_pad(header_and_data);

    std::string crc_encoded_header_and_data = CRC::CRC16::encode(zero_padded_header_and_data);

    std::stringstream creating_package;
    creating_package << _pre_and_postamble
                     << crc_encoded_header_and_data
                     << _pre_and_postamble;

    std::string full_package = creating_package.str();
    return full_package;
}

std::string ComProtocol::zero_pad(std::string binary_msg)
{
    int length_of_msg = binary_msg.size();

    int zeros = 4 - (length_of_msg % 4);
    if (zeros == 4)
    {
        zeros = 0;
    }

    for (int i = 0; i < zeros; i++)
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

std::string ComProtocol::remove_pre_and_postamble(std::string received_package)
{
    int length_of_pre_and_postamble = _pre_and_postamble.size();

    received_package.erase(received_package.begin(), received_package.begin() + length_of_pre_and_postamble);
    received_package.erase(received_package.end() - length_of_pre_and_postamble, received_package.end());

    return received_package;
}

std::vector<int> ComProtocol::find_length_pos_in_header(std::string received_package)
{
    int SFD_length = _SFD.size();
    int EFD_length = _EFD.size();

    // Finding index of SFD in the received package
    std::size_t index_SFD = received_package.find(_SFD);
    if (index_SFD != std::string::npos)
    {
        std::cout << "SFD \"" << _SFD << "\" found at index: " << index_SFD << std::endl;
    }
    else
    {
        std::cout << "SFD \"" << _SFD << "\" not found in the package.\n";
    }
    int end_idx_of_SFD = index_SFD + SFD_length - 1;

    // Finding index of EFD in the received package
    std::size_t index_EFD = received_package.find(_EFD);
    if (index_EFD != std::string::npos)
    {
        std::cout << "SFD \"" << _SFD << "\" found at index: " << index_EFD << std::endl;
    }
    else
    {
        std::cout << "SFD \"" << _SFD << "\" not found in the package.\n";
    }
    int start_idx_of_EFD = index_EFD;

    std::cout << "Length is at index: " << end_idx_of_SFD + 1 << "-" << start_idx_of_EFD - 1 << std::endl;
    std::vector<int> position_of_length = {end_idx_of_SFD + 1, start_idx_of_EFD - 1};

    return position_of_length;
}

std::string ComProtocol::get_data_from_package(std::string received_package)
{
    // Removing the pre- and postamble from recveived package
    received_package = remove_pre_and_postamble(received_package);

    // Checking CRC (validity) of received package
    std::string crc_decoded_remainder = CRC::CRC16::decode(received_package);
    int int_crc_decoded_remainder = std::stoi(crc_decoded_remainder, nullptr, 2);
    if (int_crc_decoded_remainder != 0)
    {
        std::cout << "Received package IS NOT correct. CRC remainder not equal to 0." << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Received package is correct. CRC remainder equals 0." << std::endl;

        // Getting length of data
        std::vector<int> length_pos = find_length_pos_in_header(received_package);
        std::string binary_length_of_data = received_package.substr(length_pos[0], length_pos[1] - length_pos[0] + 1);
        int int_length_of_data = std::stoi(binary_length_of_data, nullptr, 2);
        std::cout << "Length of data: " << int_length_of_data << std::endl;

        // Retrieving the data from the received package
        int size_of_SFD = _SFD.size();
        int size_of_EFD = _EFD.size();
        int size_of_length_description = binary_length_of_data.size();

        int start_idx = size_of_SFD + size_of_length_description + size_of_EFD;
        std::string data = received_package.substr(start_idx, int_length_of_data);
        std::cout << "The received data: " << data << std::endl;

        return data;
    }
}