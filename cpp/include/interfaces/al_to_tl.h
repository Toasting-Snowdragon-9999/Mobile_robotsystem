#ifndef AL_TO_TL_H
#define AL_TO_TL_H

#include <iostream>
#include <string>
#include <bitset>

#define nibble_size 4

using std::string;

class AlToTl
{
private:
    string _buffer = "";

public:
    void update_buffer(const string &binary_msg);

    string get_buffer();
};

#endif // AL_TO_DLL_H