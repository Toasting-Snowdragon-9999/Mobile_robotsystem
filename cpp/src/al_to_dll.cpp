#include "al_to_dll.h"

string AlToDll::zero_pad(string &non_zero_padded_commands)
{
    int string_length = non_zero_padded_commands.length();

    int zeros_int = 0;

    if (!(string_length % nibble_size))
    {

        zeros_int = nibble_size * (1 + (string_length / nibble_size)) - string_length;
    }

    std::bitset <nibble_size> how_many_zeros(zeros_int);

    string how_many_zeros_str = how_many_zeros.to_string();

    string zeros_str(zeros_int, '0');

    non_zero_padded_commands += zeros_str;

    how_many_zeros_str += non_zero_padded_commands;

    return how_many_zeros_str;
}

    string AlToDll::remove_zero_pad(string &possibly_zero_padded_commands)
{
    string non_zero_padded_commands = "";

    int len = possibly_zero_padded_commands.length();
    string how_many_zeros_str = possibly_zero_padded_commands.substr(0,nibble_size);

    int how_many_zeros_int = std::stoi(how_many_zeros_str,nullptr,2);

    non_zero_padded_commands = possibly_zero_padded_commands.substr(0+nibble_size,len-how_many_zeros_int);

    return non_zero_padded_commands;
}