#include "interfaces/al_to_tl.h"

void AlToTl::update_buffer(const string &binary_msg)
{
    _buffer = binary_msg;
}

string AlToTl::get_buffer()
{
    return _buffer;
}