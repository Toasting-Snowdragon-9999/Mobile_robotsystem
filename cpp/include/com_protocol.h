#ifndef COM_PROTOCOL_H
#define COM_PROTOCOL_H

#include <vector>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <cstdint>
#include <sstream>

using std::cout;
using std::endl;
using std::string;

class ComProtocol
{

private:
    std::string _pre_and_postamble = "111011101110"; // Sequence of DTMF tones for preamble and postamble
    std::string _SFD = "10011001";                   // Start-of-Frame Delimiter for header
    std::string _EFD = "01100110";                   // End-of-Frame Delimiter for header
    std::string _robot_path;                         // Data formed by path for robot, created by user

public:
    /// @brief Constructor to create instance of ComProtocol
    /// @param robotPath Type: vector of vectors { {...}, {...}, ....., {...} } - The message itself (path for robot created by user)
    ComProtocol(std::string robotPath);

    /// @brief Method for finding the length of a string
    /// @param s Type: String
    /// @return Type: String
    std::string length_of_string(std::string s);

    /// @brief Method for creating entire package - Adds preamble, header, data, CRC and postamble toghether in a bitstream
    /// @return Type: String
    std::string protocol_structure();

    /// @brief Binary conversion for CRC-check
    /// @param decimalSequence Type: vector of vectors { {...}, {...}, ....., {...} } - The original tone sequence in decimal, divided in vectors of vectors
    /// @return Type: String - The sequence in a long binary string
    string decimal_seq_to_binary_msg(const std::vector<std::vector<int>> &decimalSequence);

    /// @brief Method for removing the pre- and postamble from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The received package as a string of bits without the pre- and postamble
    std::string remove_pre_and_postamble(std::string received_package);

    /// @brief Zero-pads a binary message to make it %4=0
    /// @param binary_msg Type: String - The binary message
    /// @return Type: String - The binary message with zero-padding
    std::string zero_pad(std::string binary_msg);

    /// @brief Finds the indexes at which the length description begins and ends
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: vector<int> - {start index, end index}
    std::vector<int> find_length_pos_in_header(std::string received_package);

    /// @brief Finds the data/message itself, from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The data itself
    std::string get_data_from_package(std::string received_package);
};

#endif // COM_PROTOCOL_H