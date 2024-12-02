#ifndef COM_PROTOCOL_H
#define COM_PROTOCOL_H

#define nibble_size 4
#define byte_size 8
#define timeout std::chrono::seconds(10)

#include <vector>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <cstdint>
#include <sstream>
#include <chrono>
#include "crc.h"

using std::cout;
using std::endl;
using std::string;

class ComProtocol
{

private:
    string _pre_and_postamble = "1110"; // DTMF tone for preamble and postamble
    string _SFD = "10011001";           // Start-of-Frame Delimiter for header
    string _EFD = "01100110";           // End-of-Frame Delimiter for header
    string _ESC_nibble = "1111";        // ESC nibble
    string _robot_path;                 // Data formed by path for robot, created by user
    string _ready_for_pl_path = "";     // Path that's ready to send to physical layer


public:
    string get_ready_for_pl_path();

    /// @brief Constructor to create instance of ComProtocol
    /// @param robotPath Type: vector of vectors { {...}, {...}, ....., {...} } - The message itself (path for robot created by user)
    ComProtocol(string robotPath);

    /// @brief Method for finding the length of a string
    /// @param s Type: String
    /// @return Type: String
    string length_of_string(string s);

    /// @brief Method for creating entire package - Adds preamble, header, data, CRC and postamble together in a bitstream
    /// @return The full package while it's saved as a private variable
    string protocol_structure();

    /// @brief Nibble stuffing the package with ESC nibbles
    /// @param package Type: String - The package to be stuffed
    /// @return Type: String - The nibble stuffed package
    string nibble_stuffing(string package);

    /// @brief Removes ESC nibbles from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The received package without ESC nibbles
    string remove_esc_nibbles(string received_package);

    /// @brief Binary conversion for CRC-check
    /// @param decimalSequence Type: vector of vectors { {...}, {...}, ....., {...} } - The original tone sequence in decimal, divided in vectors of vectors
    /// @return Type: String - The sequence in a long binary string
    string decimal_seq_to_binary_msg(const std::vector<std::vector<int>> &decimalSequence);

    /// @brief Method for removing the pre- and postamble from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The received package as a string of bits without the pre- and postamble
    string remove_pre_and_postamble(string received_package);

    /// @brief Zero-pads a binary message to make it %4=0
    /// @param binary_msg Type: String - The binary message
    /// @return Type: String - The binary message with zero-padding
    string zero_pad(string binary_msg);

    /// @brief Finds the indexes at which the length description begins and ends
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: vector<int> - {start index, end index}
    std::vector<int> find_length_pos_in_header(string received_package);

    /// @brief Finds the data/message itself, from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The data itself
    string get_data_from_package(string received_package);

    // Method for main
    void start_ack_timer();

    bool is_header_and_msg_correct(const string &header_and_msg);

};

#endif // COM_PROTOCOL_H