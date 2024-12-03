#ifndef CRC_H
#define CRC_H

#include <iostream>
#include <string>

using std::string;

namespace CRC
{
    /// @brief Performs XOR-operation on 2 strings characterwise
    /// @param a 1st string to perform XOR-operation on
    /// @param b 2nd string to perform XOR-operation on
    /// @return Result of XOR-operation
    string exclusive_or_strings(string a, string b);

    namespace CRC7
    {
        /// @brief Encodes the codeword using CRC7-ITU-T generator polynomial
        /// @param dataword The binary message that is to be encoded
        /// @return The codeword (encoded dataword)
        string encode(string dataword);

        /// @brief Decodes the codeword using CRC7-ITU-T generator polynomial
        /// @param codeword The encoded dataword
        /// @return The last 7 bits after decoding
        string decode(string codeword);
    }
    namespace CRC16
    {
        /// @brief Encodes the codeword using CRC-16-IBM generator polynomial
        /// @param dataword The binary message that is to be encoded
        /// @return The codeword (encoded dataword)
        string encode(string dataword);

        /// @brief Decodes the codeword using CRC-16-IBM generator polynomial
        /// @param codeword The encoded dataword
        /// @return The last 16 bits after decoding
        string decode(string codeword);
    }
    namespace CRC32
    {
        /// @brief Encodes the codeword using IEEE 802.3 CRC-32 polynomial
        /// @param dataword The binary message that is to be encoded
        /// @return The codeword (encoded dataword)
        string encode(string dataword);

        /// @brief Decodes the codeword using IEEE 802.3 CRC-32 polynomial
        /// @param codeword The encoded dataword
        /// @return The last 32 bits after decoding
        string decode(string codeword);
    }
}

#endif // CRC_H