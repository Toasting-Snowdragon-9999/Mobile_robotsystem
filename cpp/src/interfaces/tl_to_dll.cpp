#include "tl_to_dll.h"

std::vector<string> TlToDll::add_segment_to_buffer(const string &encoded_segment)
{
    string to_add_segment = encoded_segment;
    _segment_buffer.push_back(to_add_segment);
}