#include "al_to_dll.h"

string AlToDll::zero_pad(string &non_zero_padded_commands)
{
    int string_length = non_zero_padded_commands.length();

    int zeros_int;

    if(!(string_length % nibble_size)){

        zeros_int = nibble_size*(1+(string_length/nibble_size))-string_length;
    }

    string zeros_str(zeros_int,'0');

    non_zero_padded_commands += zeros_str;

};
