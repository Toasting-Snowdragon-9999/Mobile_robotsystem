#ifndef AL_TO_TL_H
#define AL_TO_TL_H

#include <iostream>
#include <string>
#include <bitset>

#define nibble_size 4


class AlToTl
{
private:
    std::string _buffer = "";

public:
    void add_string_to_buffer(const std::string &binary_msg);

    std::string get_buffer();
};

#endif // AL_TO_DLL_H