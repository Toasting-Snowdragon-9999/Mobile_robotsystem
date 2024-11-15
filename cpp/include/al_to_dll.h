#ifndef AL_TO_DLL_H
#define AL_TO_DLL_H

#include <iostream>
#include <string>

#define nibble_size 4

using std::string;

class Al_To_Dll

{
public:
    string zero_pad(string &non_zero_padded_commands);
};

#endif // AL_TO_DLL_H