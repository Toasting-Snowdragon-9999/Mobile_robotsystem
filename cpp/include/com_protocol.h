#ifndef COM_PROTOCOL_H
#define COM_PROTOCOL_H

#include <vector>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>
#include <cstdint>

using std::cout;
using std::endl;
using std::string;

class ComProtocol
{

private:
    std::vector<uint16_t> _preAndPostamble = {10, 10, 10}; // Sequence of DTMF tones for preamble and postamble
    std::vector<uint16_t> _robotPathLength;                // Length of data in the message itself
    std::vector<std::vector<uint16_t>> _robotPath;         // Data formed by path for robot, created by user

public:
    /// @brief Constructor to create instance of ComProtocol
    /// @param robotPath Type: vector of vectors { {...}, {...}, ....., {...} } - The message itself (path for robot created by user)
    ComProtocol(std::vector<std::vector<uint16_t>> robotPath);

    /// @brief Method for creating entire package - Adds preamble, length, data, CRC and postamble toghether in a vector of vectors
    /// @return Type: Vectors in vector { {preamble}, {length}, {robot path}, {CRC}, {postamble} }
    std::vector<std::vector<uint16_t>> protocol_structure();

    /// @brief Method for testing purpose - Prints all elements for a package (vector of vectors)
    /// @param package Type: vector of vectors { {...}, {...}, ....., {...} }
    /// @param description Type: String "description" - Describes what the package contains
    void print_vec_elements(std::vector<std::vector<uint16_t>> package, std::string description);

    /// @brief Binary conversion for CRC-check
    /// @param decimalSequence Type: vector of vectors { {...}, {...}, ....., {...} } - The original tone sequence in decimal, divided in vectors of vectors
    /// @return Type: String - The sequence in a long binary string
    string decimal_seq_to_binary_msg(const std::vector<std::vector<uint16_t>> &decimalSequence);

    /// @brief Performs XOR-operation on 2 strings characterwise
    /// @param a Type: String - 1st string to perform XOR-operation on
    /// @param b Type. String - 2nd string to perform XOR-operation on
    /// @return Type: String - Result of XOR-operation
    string exclusive_or_strings(string a, string b);

    /// @brief Encode binary dataword with CRC4-Codeword
    /// @param binaryDataword Type: String - String of binary numbers to be encoded
    /// @return Type: String - Binary dataword with CRC4-Codeword appended
    string crc4_encode(string binaryDataword);

    /// @brief Decodes an encoded binary dataword using CRC4 (binary division)
    /// @param binaryEncodedDataword Type: String - A binary encoded dataword
    /// @return Type: String - A binary decoded dataword
    string crc4_decode(string binaryEncodedDataword);

    /// @brief Prints nested vector
    /// @param preambleSeq Type: vector of vectors { {...}, {...}, ....., {...} } - Decimal values for sequence
    /// @param name Type: String - Name for the sequence
    void print_nested_vector(const std::vector<std::vector<uint16_t>> &preambleSeq, const std::string &name = "Preamble Sequence");

    /// @brief Finds remainder in a binary dataword (last 4 binary digits)
    /// @param dataword Type: String - A binary dataword
    /// @return Type: String - Remainder of dataword (4 binary digits)
    std::string find_remainder(std::string dataword);

    /// @brief Checks if decoded CRC-message is correct (if remainder is 0000)
    /// @param binaryDecodedDataword Type: String - A binary decoded dataword
    /// @return Type: Bool - 1 if message is correct and 0 if not
    bool is_message_correct(const string &binaryDecodedDataword);
};

#endif // COM_PROTOCOL_H