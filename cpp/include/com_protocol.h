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
    std::vector<int> _robotPathLength;                // Length of data in the message itself
    std::vector<std::vector<int>> _robotPath;         // Data formed by path for robot, created by user

public:
    /// @brief Constructor to create instance of ComProtocol
    /// @param robotPath Type: vector of vectors { {...}, {...}, ....., {...} } - The message itself (path for robot created by user)
    ComProtocol(std::vector<std::vector<int>> robotPath);

    /// @brief Method for creating entire package - Adds preamble, length, data, CRC and postamble toghether in a vector of vectors
    /// @return { {preamble}, {length}, {robot path}, {CRC}, {postamble} }
    std::vector<std::vector<int>> protocol_structure();

    /// @brief Method for testing purpose - Prints all elements for a package (vector of vectors)
    /// @param package Type: vector of vectors { {...}, {...}, ....., {...} }
    /// @param description Type: String "description" - Describes what the package contains
    void print_vec_elements(std::vector<std::vector<int>> package, std::string description);
};

#endif // COM_PROTOCOL_H