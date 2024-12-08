#include "interfaces/tl_to_dll.h"

bool TlToDll::get_all_segments_received()
{
    return _all_segments_received;
}

void TlToDll::set_all_segments_received(const bool &input)
{
    _all_segments_received = input;
}

std::deque<std::string> TlToDll::add_segments_to_buffer(const std::vector<std::string> &segment_buffer)
{
    _segment_buffer = std::deque<std::string>(segment_buffer.begin(), segment_buffer.end());

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
        _segment_buffer.pop_front();
        return select_segment;
    }

    return select_segment;
}

std::string TlToDll::get_first_segment_from_buffer()
{
    return _segment_buffer[0];
}

void TlToDll::remove_first_segment_from_buffer()
{
    return _segment_buffer.pop_front();
}


std::deque<std::string> TlToDll::get_segment_buffer(){
    return _segment_buffer;
}
