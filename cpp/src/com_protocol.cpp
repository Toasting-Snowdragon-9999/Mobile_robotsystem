#include "../include/com_protocol.h"

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