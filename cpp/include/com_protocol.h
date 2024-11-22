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

    /// @brief Performs XOR-operation on 2 strings characterwise
    /// @param a Type: String - 1st string to perform XOR-operation on
    /// @param b Type. String - 2nd string to perform XOR-operation on
    /// @return Type: String - Result of XOR-operation
    string exclusive_or_strings(string a, string b);

    /// @brief Encode binary dataword with CRC16-Codeword
    /// @param binaryDataword Type: String - String of binary numbers to be encoded
    /// @return Type: String - Binary dataword with CRC16-Codeword appended
    string crc16_encode(string binaryDataword);

    /// @brief Decodes an encoded binary dataword using CRC16 (binary division)
    /// @param binaryEncodedDataword Type: String - A binary encoded dataword
    /// @return Type: String - A binary decoded dataword
    string crc16_decode(string binaryEncodedDataword);

    /// @brief Finds remainder in a binary dataword (last 4 binary digits)
    /// @param dataword Type: String - A binary dataword
    /// @return Type: String - Remainder of dataword (4 binary digits)
    std::string find_remainder(std::string dataword);

    /// @brief Checks if decoded CRC-message is correct (if remainder is 0000)
    /// @param binaryDecodedDataword Type: String - A binary decoded dataword
    /// @return Type: Bool - 1 if message is correct and 0 if not
    bool is_message_correct(const string &binaryDecodedDataword);

    /// @brief Derives the binary message from the whole package (removes pre- and postamble, length, and CRC).
    /// @param package Type: String - The full binary package with pre- and postamble, length and CRC
    /// @return Type: String - The binary message
    /// @note Only returns message if it is correct (CRC of decoded message is 0000)
    std::string get_binary_message_from_package(std::vector<std::vector<int>> package);

    /// @brief Method for removing the pre- and postamble from the received package
    /// @param received_package Type: String - The received package as a string of bits
    /// @return Type: String - The received package as a string of bits without the pre- and postamble
    std::string remove_pre_and_postamble(std::string received_package);

    /// @brief Zero-pads a binary message to make it %4=0
    /// @param binary_msg Type: String - The binary message
    /// @return Type: String - The binary message with zero-padding
    std::string zero_pad(std::string binary_msg);
};

#endif // COM_PROTOCOL_H