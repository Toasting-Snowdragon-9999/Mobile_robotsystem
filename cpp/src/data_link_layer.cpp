#include "data_link_layer.h"
#include "crc.h"

ComProtocol::ComProtocol(std::string robot_path) : _robot_path(robot_path) {}

string ComProtocol::get_ready_for_pl_path()
{
    return _ready_for_pl_path;
}

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

    std::string crc_encoded_header_and_data = CRC::CRC32::encode(zero_padded_header_and_data);
    std::string nibble_stuffed_header_and_data = nibble_stuffing(crc_encoded_header_and_data);

    std::stringstream creating_package;
    creating_package << _pre_and_postamble
                     << nibble_stuffed_header_and_data
                     << _pre_and_postamble;

    _ready_for_pl_path = creating_package.str();

    return ComProtocol::get_ready_for_pl_path();
}

std::string ComProtocol::nibble_stuffing(std::string package)
{
    int i = 0;
    while (i <= package.size() - nibble_size)
    {
        std::string nibble = package.substr(i, nibble_size);
        if (nibble == _pre_and_postamble || nibble == _ESC_nibble)
        {
            package.insert(i, _ESC_nibble);
            i += _ESC_nibble.size();
        }
        i += 4; // Move to the next nibble
    }

    return package;
}

std::string ComProtocol::remove_esc_nibbles(std::string received_package)
{
    std::string ESC_and_pre_and_postamble = _ESC_nibble + _pre_and_postamble;
    std::string ESC_and_ESC = _ESC_nibble + _ESC_nibble;

    int i = 0;
    while (i <= received_package.size() - byte_size)
    {
        std::string byte = received_package.substr(i, byte_size);
        if (byte == ESC_and_pre_and_postamble || byte == ESC_and_ESC)
        {
            received_package.erase(i, _ESC_nibble.size());
        }
        i += 4;
    }

    return received_package;
}

std::string ComProtocol::zero_pad(std::string binary_msg)
{
    int length_of_msg = binary_msg.size();

    int zeros = nibble_size - (length_of_msg % nibble_size);
    if (zeros == nibble_size)
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

                while (tmpTone.length() < nibble_size)
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

    // Removing ESC nibbles
    received_package = remove_esc_nibbles(received_package);

    // Checking CRC (validity) of received package
    std::string crc_decoded_remainder = CRC::CRC32::decode(received_package);
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

void ComProtocol::start_ack_timer()
{
    auto start = std::chrono::steady_clock::now();

    while (1)
    {
        auto now = std::chrono::steady_clock::now();

        auto elapsed = now - start;

        if (elapsed >= timeout)
        {
            break;
        }
    }
}

bool ComProtocol::is_header_and_msg_correct(const string &header_and_msg){
    return ~std::stoi(CRC::CRC32::decode(header_and_msg),nullptr,2);
}