#ifndef AL_TO_DLL_H
#define AL_TO_DLL_H

#include <iostream>
#include <string>
#include <bitset>

#define nibble_size 4

using std::string;

class AlToDll
{
private:
    int _how_many_zeros = 0;

public:
    string zero_pad(string &non_zero_padded_commands);

    string remove_zero_pad(string &possibly_zero_padded_commands);
};

#endif // AL_TO_DLL_H