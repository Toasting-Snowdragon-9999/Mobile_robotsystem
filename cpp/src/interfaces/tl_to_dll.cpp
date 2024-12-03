#include "interfaces/tl_to_dll.h"

std::deque<string> TlToDll::add_segments_buffer(const std::vector<string> &segment_buffer)
{
    _segment_buffer = std::deque<string>(segment_buffer.begin(),segment_buffer.end());

    return _segment_buffer;
}

void TlToDll::add_segment_to_buffer(const string &encoded_segment)
{
    string to_add_segment = encoded_segment;
    _segment_buffer.push_back(to_add_segment);
}

string TlToDll::take_segment_from_buffer()
{
    string select_segment = "";
    if (!_segment_buffer.empty())
    {
        string select_segment = _segment_buffer[0];
        _segment_buffer.pop_front();
        return select_segment;
    }

    return select_segment;
}
