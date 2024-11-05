#ifndef COM_PROTOCOL_H
#define COM_PROTOCOL_H

#include <vector>
#include <iostream>
#include <bitset>
#include <string>
#include <algorithm>

using std::cout;
using std::endl;
using std::string;

class ComProtocol
{

private:
    std::vector<int> _preAndPostamble = {10, 10, 10}; // Sequence of DTMF tones for preamble and postamble
    std::vector<int> _robotPathLength;                // Length of data in the message itself
    std::vector<std::vector<int>> _robotPath;         // Data formed by path for robot, created by user

public:
    /// @brief Constructor to create instance of ComProtocol
    /// @param robotPath Type: vector of vectors { {...}, {...}, ....., {...} } - The message itself (path for robot created by user)
    ComProtocol(std::vector<std::vector<int>> robotPath);

    /// @brief Method for creating entire package - Adds preamble, length, data, CRC and postamble toghether in a vector of vectors
    /// @return Type: Vectors in vector { {preamble}, {length}, {robot path}, {CRC}, {postamble} }
    std::vector<std::vector<int>> protocol_structure();

    /// @brief Method for testing purpose - Prints all elements for a package (vector of vectors)
    /// @param package Type: vector of vectors { {...}, {...}, ....., {...} }
    /// @param description Type: String "description" - Describes what the package contains
    void print_vec_elements(std::vector<std::vector<int>> package, std::string description);

    /// @brief Binary conversion for CRC-check
    /// @param decimalSequence Type: vector of vectors { {...}, {...}, ....., {...} } - The original tone sequence in decimal, divided in vectors of vectors
    /// @return Type: String - The sequence in a long binary string
    string decimal_seq_to_binary_msg(const std::vector<std::vector<int>> &decimalSequence);

    /// @brief Performs XOR-operation on 2 strings characterwise
    /// @param a Type: String - 1st string to perform XOR-operation on
    /// @param b Type. String - 2nd string to perform XOR-operation on
    /// @return Type: String - Result of XOR-operation
    string exclusive_or_strings(string a, string b);

    /// @brief Encode binary dataword with CRC4-Codeword
    /// @param binaryDataword Type: String - String of binary numbers to be encoded
    /// @return Type: String - Binary dataword with CRC4-Codeword appended
    string crc4(string binaryDataword);

    /// @brief Prints nested vector 
    /// @param preambleSeq Type: vector of vectors { {...}, {...}, ....., {...} } - Decimal values for sequence
    /// @param name Type: String - Name for the sequence
    void print_nested_vector(const std::vector<std::vector<int>> &preambleSeq, const std::string &name = "Preamble Sequence");

    std::string find_remainder(std::string msg);
};

#endif // COM_PROTOCOL_H