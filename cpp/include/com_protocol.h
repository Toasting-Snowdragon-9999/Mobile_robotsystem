#ifndef COM_PROTOCOL_H
#define COM_PROTOCOL_H

#include <vector>
#include <iostream>
#include <bitset>
#include <string>

class ComProtocol
{

private:
    std::vector<int> _preAndPostamble = {10, 10, 10}; // Sequence of DTMF tones for preamble and postamble
    std::vector<int> _dataLength;                     // Length of data in the message itself
    std::vector<std::vector<int>> _data;              // Data (message) to be transmitted

public:
    // Constructor for ComProtocol
    ComProtocol(std::vector<std::vector<int>> data);
    // {
    //     int outerVecSize = _data.size();    // Size of outer vector (how many vectors are in the vector)
    //     int size = 0;

    //     for (int i = 0; i < outerVecSize; i++)
    //     {
    //         int innerVecSize = _data[i].size();     // Size of inner vector (how many elements are in the currently looked at vector)

    //         for (int i = 0; i < innerVecSize; i++)
    //         {
    //             size++;
    //         }
    //     }
    //     std::bitset<12> binaryBitLength(size);
    //     std::string binaryStrLength = binaryBitLength.to_string();

    //     std::string DTMFLength1 = binaryStrLength.substr(0, 4);
    //     std::string DTMFLength2 = binaryStrLength.substr(4, 4);
    //     std::string DTMFLength3 = binaryStrLength.substr(8, 4);

    //     int DTMFLengthTone1 = std::stoi(DTMFLength1, nullptr, 2);
    //     int DTMFLengthTone2 = std::stoi(DTMFLength2, nullptr, 2);
    //     int DTMFLengthTone3 = std::stoi(DTMFLength3, nullptr, 2);

    //     _dataLength = {DTMFLengthTone1, DTMFLengthTone2, DTMFLengthTone3};
    // }

    std::vector<std::vector<int>> protocol_structure();
    // {
    //     std::vector<std::vector<int>> protocolStructureVec;

    //     // Add all elements of data to the protocol structure (creating the entire package)
    //     protocolStructureVec.push_back(_preAndPostamble);
    //     protocolStructureVec.push_back(_dataLength);
    //     for (int i = 0; i < _data.size(); i++)
    //     {
    //         protocolStructureVec.push_back(_data[i]);
    //     }
    //     protocolStructureVec.push_back(_preAndPostamble);

    //     return protocolStructureVec;
    // }

    // For testing purpose - Prints elements from a package
    void printVecElements(std::vector<std::vector<int>> package, std::string description);
    // {
    //     for (size_t i = 0; i < package.size(); ++i)
    //     {
    //         std::cout << description << ":      ";

    //         // Loop through the elements of the current vector
    //         for (size_t j = 0; j < package[i].size(); ++j)
    //         {
    //             std::cout << package[i][j] << " "; // Print each element
    //         }

    //         std::cout << std::endl; // New line after each vector
    //     }
    // }
};

#endif // COM_PROTOCOL_H