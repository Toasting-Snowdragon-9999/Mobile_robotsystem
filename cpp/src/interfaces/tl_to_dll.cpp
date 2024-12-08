#include "interfaces/tl_to_dll.h"

std::deque<std::string> TlToDll::add_segments_to_buffer(const std::vector<std::string> &segment_buffer)
{
    _segment_buffer = std::deque<std::string>(segment_buffer.begin(),segment_buffer.end());
    std::cout << _segment_buffer.size() << std::endl;
    return _segment_buffer;
}

void TlToDll::add_segment_to_buffer(const std::string &encoded_segment)
{
    std::string to_add_segment = encoded_segment;
    _segment_buffer.push_back(to_add_segment);
}

std::string TlToDll::take_segment_from_buffer()
{
    std::string select_segment = "";
    if (!_segment_buffer.empty())
    {
        std::string select_segment = _segment_buffer[0];
        return select_segment;
    }

    return select_segment;
}

void TlToDll::remove_segment_from_buffer()
{
    if (!_segment_buffer.empty())
    {
        _segment_buffer.pop_front();
    }
}

bool TlToDll::is_buffer_empty()
{
    return _segment_buffer.empty();
}
