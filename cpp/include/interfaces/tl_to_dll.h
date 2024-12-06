#ifndef TL_TO_DLL_H
#define TL_TO_DLL_H

#include <vector>
#include <deque>
#include <string>


class TlToDll
{
private:
    std::deque<std::string> _segment_buffer;

public:
    // Sender
    std::deque<std::string> add_segments_to_buffer(const std::vector<std::string> &segment_buffer);

    std::string take_segment_from_buffer();

    // Receiver
    void add_segment_to_buffer(const std::string &encoded_segment);
};

#endif