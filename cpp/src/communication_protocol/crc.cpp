#include "communication_protocol/crc.h"

namespace CRC
{
    std::string exclusive_or_strings(std::string a, std::string b)
    {
        std::string xorresult = "";
        if (a.size() != b.size())
        {
            throw std::invalid_argument("Strings of XOR-operation are not same size");
        }
        for (int i = 0; i < a.size(); i++)
        {
            xorresult += (a[i] == b[i]) ? '0' : '1';
        }
        return xorresult;
    }
    namespace CRC7
    {

        std::string encode(std::string dataword)
        {
            std::string generator_poly = "110100111"; // Dataword CRC7-ITU-T
            std::string codeword = dataword;

            int crcDegree = generator_poly.length() - 1;

            int selectionPlusOneIdx = generator_poly.length();
            int generator_polyLength = generator_poly.length();

            std::string binaryDatawordWithZeroes = dataword + std::string(crcDegree, '0'); // Append CRC-Degree zeroes to data

            std::string selection = binaryDatawordWithZeroes.substr(0, generator_poly.length());
            int datawordLength = binaryDatawordWithZeroes.length();

            while (selectionPlusOneIdx < datawordLength) // Binary-division
            {
                if (selection[0] == '1')
                {
                    selection = CRC::exclusive_or_strings(selection, generator_poly);
                }

                selection = selection.substr(1) + binaryDatawordWithZeroes[selectionPlusOneIdx];
                selectionPlusOneIdx++;
            }

            if ((selection[0] == '1')) // XOR the last selection with the generator_poly if needed
            {
                selection = CRC::exclusive_or_strings(selection, generator_poly);
            }

            std::string remainder = selection.substr(1); // Return substring since generator_poly is CRC-Degree+1 in size

            return codeword = dataword + remainder;
        }
        std::string decode(std::string codeword)
        {
            std::string generator_poly = "110100111";
            std::string decodedBinaryData = codeword;

            int crcDegree = generator_poly.length() - 1;

            int selectionPlusOneIdx = generator_poly.length();
            int generator_polyLength = generator_poly.length();

            std::string selection = codeword.substr(0, generator_poly.length());
            int encodedDatawordLength = codeword.length();

            while (selectionPlusOneIdx < encodedDatawordLength) // Binary-division
            {
                if (selection[0] == '1')
                {
                    selection = CRC::exclusive_or_strings(selection, generator_poly);
                }

                selection = selection.substr(1) + codeword[selectionPlusOneIdx];
                selectionPlusOneIdx++;
            }

            if ((selection[0] == '1')) // XOR the last selection with the generator_poly if needed
            {
                selection = CRC::exclusive_or_strings(selection, generator_poly);
            }

            std::string remainder = selection.substr(1); // Return substring since generator_poly is CRC-Degree+1 in size

            return remainder;
        }
    }
    namespace CRC16
    {
        std::string encode(std::string dataword)
        {
            std::string generator_poly = "11000000000000101"; // Dataword CRC7-ITU-T
            std::string codeword = dataword;

            int crcDegree = generator_poly.length() - 1;

            int selectionPlusOneIdx = generator_poly.length();
            int generator_polyLength = generator_poly.length();

            std::string binaryDatawordWithZeroes = dataword + std::string(crcDegree, '0'); // Append CRC-Degree zeroes to data

            std::string selection = binaryDatawordWithZeroes.substr(0, generator_poly.length());
            int datawordLength = binaryDatawordWithZeroes.length();

            while (selectionPlusOneIdx < datawordLength) // Binary-division
            {
                if (selection[0] == '1')
                {
                    selection = CRC::exclusive_or_strings(selection, generator_poly);
                }

                selection = selection.substr(1) + binaryDatawordWithZeroes[selectionPlusOneIdx];
                selectionPlusOneIdx++;
            }

            if ((selection[0] == '1')) // XOR the last selection with the generator_poly if needed
            {
                selection = CRC::exclusive_or_strings(selection, generator_poly);
            }

            std::string remainder = selection.substr(1); // Return substring since generator_poly is CRC-Degree+1 in size

            return codeword = dataword + remainder;
        }
        std::string decode(std::string codeword)
        {
            std::string generator_poly = "11000000000000101";
            std::string decodedBinaryData = codeword;

            int crcDegree = generator_poly.length() - 1;

            int selectionPlusOneIdx = generator_poly.length();
            int generator_polyLength = generator_poly.length();

            std::string selection = codeword.substr(0, generator_poly.length());
            int encodedDatawordLength = codeword.length();

            while (selectionPlusOneIdx < encodedDatawordLength) // Binary-division
            {
                if (selection[0] == '1')
                {
                    selection = CRC::exclusive_or_strings(selection, generator_poly);
                }

                selection = selection.substr(1) + codeword[selectionPlusOneIdx];
                selectionPlusOneIdx++;
            }

            if ((selection[0] == '1')) // XOR the last selection with the generator_poly if needed
            {
                selection = CRC::exclusive_or_strings(selection, generator_poly);
            }

            std::string remainder = selection.substr(1); // Return substring since generator_poly is CRC-Degree+1 in size

            return remainder;
        }

    }
    namespace CRC32
    {
        std::string encode(std::string dataword)
        {
            std::string generator_poly = "100000100110000010001110110110111"; // IEEE 802.3 CRC-32 polynomial
            std::string codeword = dataword;

            int crcDegree = generator_poly.length() - 1;

            int selectionPlusOneIdx = generator_poly.length();
            int generator_polyLength = generator_poly.length();

            std::string binaryDatawordWithZeroes = dataword + std::string(crcDegree, '0'); // Append CRC-Degree zeroes to data

            std::string selection = binaryDatawordWithZeroes.substr(0, generator_poly.length());
            int datawordLength = binaryDatawordWithZeroes.length();

            while (selectionPlusOneIdx < datawordLength) // Binary-division
            {
                if (selection[0] == '1')
                {
                    selection = CRC::exclusive_or_strings(selection, generator_poly);
                }

                selection = selection.substr(1) + binaryDatawordWithZeroes[selectionPlusOneIdx];
                selectionPlusOneIdx++;
            }

            if ((selection[0] == '1')) // XOR the last selection with the generator_poly if needed
            {
                selection = CRC::exclusive_or_strings(selection, generator_poly);
            }

            std::string remainder = selection.substr(1); // Return substring since generator_poly is CRC-Degree+1 in size

            return codeword = dataword + remainder;
        }
        std::string decode(std::string codeword)
        {
            std::string generator_poly = "100000100110000010001110110110111"; // IEEE 802.3 CRC-32 polynomial
            std::string decodedBinaryData = codeword;

            int crcDegree = generator_poly.length() - 1;

            int selectionPlusOneIdx = generator_poly.length();
            int generator_polyLength = generator_poly.length();

            std::string selection = codeword.substr(0, generator_poly.length());
            int encodedDatawordLength = codeword.length();

            while (selectionPlusOneIdx < encodedDatawordLength) // Binary-division
            {
                if (selection[0] == '1')
                {
                    selection = CRC::exclusive_or_strings(selection, generator_poly);
                }

                selection = selection.substr(1) + codeword[selectionPlusOneIdx];
                selectionPlusOneIdx++;
            }

            if ((selection[0] == '1')) // XOR the last selection with the generator_poly if needed
            {
                selection = CRC::exclusive_or_strings(selection, generator_poly);
            }

            std::string remainder = selection.substr(1); // Return substring since generator_poly is CRC-Degree+1 in size

            return remainder;
        }

    }
}
