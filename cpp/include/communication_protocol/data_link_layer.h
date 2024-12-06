#ifndef DATA_LINK_LAYER_H
#define DATA_LINK_LAYER_H

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


class DataLinkLayer
{

private:
    std::string _pre_and_postamble = "11100000"; // DTMF tone for preamble and postamble
    std::string _SFD = "11110";                  // Start-of-Frame Delimiter for header
    std::string _EFD = _SFD;                     // End-of-Frame Delimiter for header
    std::string _ESC_nibble = "1111";            // ESC nibble
    std::string _robot_path = "";                // Data formed by path for robot, created by user
    std::string _ready_for_pl_path = "";         // Path that's ready to send to physical layer
    bool _is_ack_received = false;

public:
    std::string get_ready_for_pl_path();

    /// @brief Constructor to create instance of DataLinkLayer
    /// @param robotPath - Path for robot created by user
    DataLinkLayer(std::string robotPath);

    /// @brief Method for finding the length of a string in binary
    /// @param s Type: String
    /// @return Type: String - Length in binary
    std::string length_of_string(std::string s);

    /// @brief Method for creating entire package - Adds preamble, header, data, CRC, ESC-nibbles, and postamble together in a bitstream
    /// @return The full package while it's saved as a private variable
    std::string protocol_structure();

    /// @brief Nibble stuffing the package with ESC nibbles
    /// @param package Type: String - The package to be stuffed
    /// @return Type: String - The nibble stuffed package
    std::string nibble_stuffing(std::string package);

    /// @brief Removes ESC nibbles from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The received package without ESC nibbles
    std::string remove_esc_nibbles(std::string received_package);

    /// @brief Binary conversion for CRC-check
    /// @param decimalSequence Type: vector of vectors { {...}, {...}, ....., {...} } - The original tone sequence in decimal, divided in vectors of vectors
    /// @return Type: String - The sequence in a long binary string
    std::string decimal_seq_to_binary_msg(const std::vector<std::vector<int>> &decimalSequence);

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
    std::string get_data_from_package();

    // Method for main
    void start_ack_timer();

    /// @brief Returns true if header and message is correct by checking CRC-remainder and false if not
    /// @param header_and_msg
    /// @return True or False
    bool is_header_and_msg_correct(const std::string &header_and_msg);

    void stop_and_wait_arq();

    bool is_ack_received();

    void set_ack_received(const bool &boolean);

    /// @brief Finds maximum consecutive ones in a string
    /// @param s
    /// @return Max number of consecutive ones
    int find_max_ones(const std::string &s);

    std::string bit_stuff(const std::string &header);

    std::string bit_unstuff(const std::string &header);
};

#endif // COM_PROTOCOL_H