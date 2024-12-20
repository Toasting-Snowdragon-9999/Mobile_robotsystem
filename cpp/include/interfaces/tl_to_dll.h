#ifndef TL_TO_DLL_H
#define TL_TO_DLL_H

#include <vector>
#include <deque>
#include <string>
#include <iostream>

class TlToDll
{
private:
    std::deque<std::string> _segment_buffer;

    bool _all_segments_received = false;

public:
    bool get_all_segments_received();

    void set_all_segments_received(const bool &input);

    std::deque<std::string> get_segment_buffer();

    // Sender
    std::deque<std::string> add_segments_to_buffer(const std::vector<std::string> &segment_buffer);

    std::string take_segment_from_buffer();

    // Receiver
    void add_segment_to_buffer(const std::string &encoded_segment);

    std::string get_first_segment_from_buffer();

    void remove_first_segment_from_buffer();

    bool is_buffer_empty();
  
};

#endif