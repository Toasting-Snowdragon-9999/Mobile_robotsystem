#include "transport_layer.h"

Transport_Layer::Transport_Layer() {}

string Transport_Layer::add_begin_and_end(const string &binary_msg)
{

    return _begin_byte + binary_msg + _end_byte;
}

string Transport_Layer::remove_begin_and_end(const string &full_binary_msg)
{
    string stripped_msg = full_binary_msg;

    size_t length = stripped_msg.length();
    int begin_length = _begin_byte.length();
    size_t end_length = _end_byte.length();

    // Remove begin byte
    if (stripped_msg.find(_begin_byte) != 0)
    {
        std::cout << "Beginning is not the begin-byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(0, _begin_byte.length());
    }

    length = stripped_msg.length();

    // Remove end byte
    if (stripped_msg.rfind(_end_byte) != length - end_length)
    {
        std::cout << "End-byte is not final byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(length - end_length, end_length);
    }

    return stripped_msg;
}

std::vector<string> Transport_Layer::segment_msg(const string &full_binary_msg)
{
    std::vector<string> segments_vector;
    string unsegmented_msg = full_binary_msg;

    while (unsegmented_msg.length() >= mms)
    {
        segments_vector.push_back(unsegmented_msg.substr(0, mms));
        unsegmented_msg.erase(0, mms);
    }

    // Final segmentation if msg isn't divisible by the mms
    if (!unsegmented_msg.empty())
    {
        segments_vector.push_back(unsegmented_msg.substr(0, unsegmented_msg.size()));
    }
    return segments_vector;
}

string Transport_Layer::combine_segments(const std::vector<string> &segment_vector)
{
    string full_binary_msg = "";
    for (const auto &segment : segment_vector){
        full_binary_msg += segment;
    }
    return full_binary_msg;
}

void Transport_Layer::print_segment_vector(const std::vector<string> &vector)
{
    int i = 0;
    std::cout << "Number of segments: " << vector.size() << std::endl;
    for (const string &element : vector)
    {
        std::cout << "Segment " << i << ":  " << element << std::endl;
        i++;
    }
}