#include "transport_layer.h"

Transport_Layer::Transport_Layer() {}

std::vector<string> Transport_Layer::get_segments_vector()
{
    return _segments_vector;
}

string Transport_Layer::find_length(const string &binary_msg)
{
    int length_of_string = binary_msg.size();
    string binary_length = "";

    if (length_of_string == 0)
    {
        binary_length += '0'; // If a tone is zero, no conversion is needed
    }

    else
    {
        while (length_of_string > 0)
        {
            binary_length += (length_of_string % 2) ? '1' : '0';

            length_of_string /= 2;
        }
    }

    std::reverse(binary_length.begin(), binary_length.end());

    return binary_length;
}

string Transport_Layer::add_header(const string &binary_msg)
{
    return SFD + Transport_Layer::find_length(binary_msg) + EFD + binary_msg;
}

string Transport_Layer::remove_header(const string &full_binary_msg)
{
    string stripped_msg = full_binary_msg;

    size_t length = stripped_msg.length();
    int begin_length = SFD.length();
    size_t end_length = EFD.length();

    // Remove begin byte
    if (stripped_msg.find(SFD) != 0)
    {
        std::cerr << "Beginning is not the begin-byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(0, SFD.length());
    }

    length = stripped_msg.length();

    // Remove end byte
    if (stripped_msg.rfind(EFD) != length - end_length)
    {
        std::cerr << "End-byte is not final byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(length - end_length, end_length);
    }

    return stripped_msg;
}

string Transport_Layer::bit_stuff(const string &full_binary_msg)
{
    std::string stuffed = "";
    int consecutiveOnes = 0;

    for (char bit : full_binary_msg)
    {
        stuffed += bit;

        if (bit == '1')
        {
            consecutiveOnes++;
            if (consecutiveOnes == 3)
            {
                stuffed += '0';
                consecutiveOnes = 0;
            }
        }
        else
        {
            consecutiveOnes = 0; // Reset counter if bit = 0
        }
    }

    return stuffed;
}

string Transport_Layer::bit_unstuff(const string &full_binary_msg)
{
    std::string unstuffed = "";
    int consecutiveOnes = 0;
    size_t i = 0;

    while (i < full_binary_msg.length())
    {
        char bit = full_binary_msg[i];
        unstuffed += bit;

        if (bit == '1')
        {
            consecutiveOnes++;
            if (consecutiveOnes == 3)
            {
                i++; // Skip bit stuffed 0
                if (i < full_binary_msg.length())
                {
                    if (full_binary_msg[i] != '0')
                    {
                        std::cerr << "Error: Expected '0' after three consecutive '1's at position " << i << std::endl;
                    }
                }
                consecutiveOnes = 0;
            }
        }
        else
        {
            consecutiveOnes = 0; // Reset counter if bit = 0
        }

        i++;
    }

    return unstuffed;
}

void Transport_Layer::segment_msg(const string &full_binary_msg)
{
    string unsegmented_msg = full_binary_msg;

    while (unsegmented_msg.length() >= mms)
    {
        _segments_vector.push_back(unsegmented_msg.substr(0, mms));
        unsegmented_msg.erase(0, mms);
    }

    // Final segmentation if msg isn't divisible by the mms
    if (!unsegmented_msg.empty())
    {
        _segments_vector.push_back(unsegmented_msg.substr(0, unsegmented_msg.size()));
    }
}

string Transport_Layer::combine_segments_to_string()
{
    string full_binary_msg = "";
    for (const auto &segment : _segments_vector)
    {
        full_binary_msg += segment;
    }
    return full_binary_msg;
}

void Transport_Layer::add_segment(const string &segment)
{
    _segments_vector.push_back(segment);
}

string Transport_Layer::get_next_segment()
{
    string next_segment = _segments_vector[0];
    _segments_vector.erase(_segments_vector.begin());
    return next_segment;
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
