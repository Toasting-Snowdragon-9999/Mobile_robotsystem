#include "com_protocol.h"

ComProtocol::ComProtocol(std::vector<std::vector<uint16_t>> robotPath) : _robotPath(robotPath)
{
    uint16_t outerVecSize = _robotPath.size(); // Size of outer vector (how many vectors are in the vector)
    uint16_t size = 0;

    for (uint16_t i = 0; i < outerVecSize; i++)
    {
        uint16_t innerVecSize = _robotPath[i].size(); // Size of inner vector (how many elements are in the currently looked at vector)

        for (uint16_t i = 0; i < innerVecSize; i++)
        {
            size++;
        }
    }
    size++; // Adds 1 to size to account for CRC¤, which is 1 tone

    uint16_t DTMFLengthTone1 = 0;
    uint16_t DTMFLengthTone2 = 0;
    uint16_t DTMFLengthTone3 = 0;
    std::string sizeString = std::to_string(size);

    if (size < 10)
    {
        DTMFLengthTone3 = size;
    }
    else if (size < 100 && size > 9)
    {
        DTMFLengthTone3 = sizeString[1] - '0';
        DTMFLengthTone2 = sizeString[0] - '0';
    }
    else if (size < 1000 && size > 99)
    {
        DTMFLengthTone3 = sizeString[2] - '0';
        DTMFLengthTone2 = sizeString[1] - '0';
        DTMFLengthTone1 = sizeString[0] - '0';
    }

    _robotPathLength = {DTMFLengthTone1, DTMFLengthTone2, DTMFLengthTone3};
}

std::vector<std::vector<uint16_t>> ComProtocol::protocol_structure()
{
    std::vector<std::vector<uint16_t>> protocolStructureVec;

    std::string binaryRobotPath = decimal_seq_to_binary_msg(_robotPath);
    std::string remainderString = find_remainder(crc4_encode(binaryRobotPath));
    uint16_t remainderInt = static_cast<uint16_t>(std::stoi(remainderString, nullptr, 2)); // Converts remainder from 4-bit string to integer value
    std::vector<uint16_t> remainderVecInt = {remainderInt};                                // Inserts remainder integer value into a vector

    // Add all elements of data to the protocol structure (creating the entire package)
    protocolStructureVec.push_back(_preAndPostamble);
    protocolStructureVec.push_back(_robotPathLength);
    for (uint16_t i = 0; i < _robotPath.size(); i++)
    {
        protocolStructureVec.push_back(_robotPath[i]);
    }
    protocolStructureVec.push_back(remainderVecInt);
    protocolStructureVec.push_back(_preAndPostamble);

    return protocolStructureVec;
}

void ComProtocol::print_vec_elements(std::vector<std::vector<uint16_t>> package, std::string description)
{
    for (size_t i = 0; i < package.size(); ++i)
    {
        std::cout << description << ":      ";

        // Loop through the elements of the current vector
        for (size_t j = 0; j < package[i].size(); ++j)
        {
            std::cout << package[i][j] << " "; // Print each element
        }

        std::cout << std::endl; // New line after each vector
    }
}

string ComProtocol::decimal_seq_to_binary_msg(const std::vector<std::vector<uint16_t>> &decimalSequence)
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
                tmpTone += '0'; // If a tone is zero, no conversion is needed

                continue;
            }

            while (toneCopy > 0)
            {
                tmpTone += (toneCopy % 2) ? '1' : '0';

                toneCopy /= 2;
            }

            std::reverse(tmpTone.begin(), tmpTone.end());
            binaryConvertedTone += tmpTone;
            tmpTone.clear();
        }
    }

    return binaryConvertedTone;
}

string ComProtocol::exclusive_or_strings(string a, string b)
{
    string xorresult = "";
    if (a.size() != b.size())
    {
        throw std::invalid_argument("Strings of XOR-operation are not same size");
    }
    for (uint16_t i = 0; i < a.size(); i++)
    {
        xorresult += (a[i] == b[i]) ? '0' : '1';
    }
    return xorresult;
}

string ComProtocol::crc4_encode(string binaryDataword)
{
    string codeword = "10011";
    string encodedBinaryData = binaryDataword;

    uint16_t crcDegree = codeword.length() - 1;

    uint16_t selectionPlusOneIdx = codeword.length();
    uint16_t codewordLength = codeword.length();

    string binaryDatawordWithZeroes = binaryDataword + string(crcDegree, '0'); // Append CRC-Degree zeroes to data

    string selection = binaryDatawordWithZeroes.substr(0, codeword.length());
    uint16_t datawordLength = binaryDatawordWithZeroes.length();

    while (selectionPlusOneIdx < datawordLength) // Binary-division
    {
        if (selection[0] == '1')
        {
            selection = exclusive_or_strings(selection, codeword);
        }

        selection = selection.substr(1) + binaryDatawordWithZeroes[selectionPlusOneIdx];
        selectionPlusOneIdx++;
    }

    if ((selection[0] == '1')) // XOR the last selection with the codeword if needed
    {
        selection = exclusive_or_strings(selection, codeword);
    }

    string remainder = selection.substr(1); // Return substring since codeword is CRC-Degree+1 in size

    return encodedBinaryData = binaryDataword + remainder;
}

string ComProtocol::crc4_decode(string binaryEncodedDataword)
{
    string codeword = "10011";
    string decodedBinaryData = binaryEncodedDataword;

    uint16_t crcDegree = codeword.length() - 1;

    uint16_t selectionPlusOneIdx = codeword.length();
    uint16_t codewordLength = codeword.length();

    string selection = binaryEncodedDataword.substr(0, codeword.length());
    uint16_t encodedDatawordLength = binaryEncodedDataword.length();

    while (selectionPlusOneIdx < encodedDatawordLength) // Binary-division
    {
        if (selection[0] == '1')
        {
            selection = exclusive_or_strings(selection, codeword);
        }

        selection = selection.substr(1) + binaryEncodedDataword[selectionPlusOneIdx];
        selectionPlusOneIdx++;
    }

    if ((selection[0] == '1')) // XOR the last selection with the codeword if needed
    {
        selection = exclusive_or_strings(selection, codeword);
    }

    string remainder = selection.substr(1); // Return substring since codeword is CRC-Degree+1 in size

    return decodedBinaryData = binaryEncodedDataword + remainder;
}

bool ComProtocol::is_message_correct(const string &binaryDecodedDataword)
{
    return find_remainder(binaryDecodedDataword) == "0000";
}

void ComProtocol::print_nested_vector(const std::vector<std::vector<uint16_t>> &preambleSeq, const std::string &name)
{
    std::cout << name << ":" << std::endl;
    std::cout << "[" << std::endl;
    for (const std::vector<uint16_t> &innerVec : preambleSeq)
    {
        std::cout << "  [ ";
        for (int num : innerVec)
        {
            std::cout << num << " ";
        }
        std::cout << "]" << std::endl;
    }
    std::cout << "]" << std::endl;
}

std::string ComProtocol::find_remainder(std::string dataword)
{
    dataword = dataword.substr(dataword.size() - 4);
    return dataword;
}

std::string ComProtocol::get_binary_message_from_package(std::string binary_package)
{
}