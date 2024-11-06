#include "com_protocol.h"

ComProtocol::ComProtocol(std::vector<std::vector<int>> robotPath) : _robotPath(robotPath)
{
    int outerVecSize = _robotPath.size(); // Size of outer vector (how many vectors are in the vector)
    int size = 0;

    for (int i = 0; i < outerVecSize; i++)
    {
        int innerVecSize = _robotPath[i].size(); // Size of inner vector (how many elements are in the currently looked at vector)

        for (int i = 0; i < innerVecSize; i++)
        {
            size++;
        }
    }
    std::bitset<12> binaryBitLength(size);
    std::string binaryStrLength = binaryBitLength.to_string();

    std::string DTMFLength1 = binaryStrLength.substr(0, 4);
    std::string DTMFLength2 = binaryStrLength.substr(4, 4);
    std::string DTMFLength3 = binaryStrLength.substr(8, 4);

    int DTMFLengthTone1 = std::stoi(DTMFLength1, nullptr, 2);
    int DTMFLengthTone2 = std::stoi(DTMFLength2, nullptr, 2);
    int DTMFLengthTone3 = std::stoi(DTMFLength3, nullptr, 2);

    _robotPathLength = {DTMFLengthTone1, DTMFLengthTone2, DTMFLengthTone3};
}

std::vector<std::vector<int>> ComProtocol::protocol_structure()
{
    std::vector<std::vector<int>> protocolStructureVec;

    // Add all elements of data to the protocol structure (creating the entire package)
    protocolStructureVec.push_back(_preAndPostamble);
    protocolStructureVec.push_back(_robotPathLength);
    for (int i = 0; i < _robotPath.size(); i++)
    {
        protocolStructureVec.push_back(_robotPath[i]);
    }
    protocolStructureVec.push_back(_preAndPostamble);

    return protocolStructureVec;
}

void ComProtocol::print_vec_elements(std::vector<std::vector<int>> package, std::string description)
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

string ComProtocol::decimal_seq_to_binary_msg(const std::vector<std::vector<int>> &decimalSequence)
{
    string binaryConvertedTone = "";
    std::string tmpTone;

    for (auto pair : decimalSequence)
    {
        for (auto tone : pair)
        {
            int toneCopy = tone;
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
    for (int i = 0; i < a.size(); i++)
    {
        xorresult += (a[i] == b[i]) ? '0' : '1';
    }
    return xorresult;
}

string ComProtocol::crc4_encode(string binaryDataword)
{
    string codeword = "10011";
    string encodedBinaryData = binaryDataword;

    int crcDegree = codeword.length() - 1;

    int selectionPlusOneIdx = codeword.length();
    int codewordLength = codeword.length();

    string binaryDatawordWithZeroes = binaryDataword + string(crcDegree, '0'); // Append CRC-Degree zeroes to data

    string selection = binaryDatawordWithZeroes.substr(0, codeword.length());
    int datawordLength = binaryDatawordWithZeroes.length();

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

    int crcDegree = codeword.length() - 1;

    int selectionPlusOneIdx = codeword.length();
    int codewordLength = codeword.length();

    string selection = binaryEncodedDataword.substr(0, codeword.length());
    int encodedDatawordLength = binaryEncodedDataword.length();

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

void ComProtocol::print_nested_vector(const std::vector<std::vector<int>> &preambleSeq, const std::string &name)
{
    std::cout << name << ":" << std::endl;
    std::cout << "[" << std::endl;
    for (const std::vector<int> &innerVec : preambleSeq)
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

std::string ComProtocol::find_remainder(std::string msg)
{
    msg = msg.substr(msg.size()-4);
    return msg;
}