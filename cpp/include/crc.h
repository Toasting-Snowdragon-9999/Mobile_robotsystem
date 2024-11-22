#ifndef CRC_H
#define CRC_H

#include <iostream>
#include <string>

using std::string;

namespace CRC
{
    string exclusive_or_strings(string a, string b);

    namespace CRC7
    {
        string crc7_encode(string binaryDataword);
        string crc7_decode(string binaryDataword);
    }
    namespace CRC16
    {
        string crc16_encode(string binaryDataword);
        string crc16_decode(string binaryDataword);
    }
}

#endif