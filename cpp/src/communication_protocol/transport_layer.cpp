#include "communication_protocol/transport_layer.h"

Transport_Layer::Transport_Layer() {}

/// @brief Getter method for the private segments vector
/// @return The private segments vector
std::vector<std::string> Transport_Layer::get_segments_vector()
{
    return _segments_vector;
}

/// @brief Finds maximum consecutive ones in a string
/// @param s
/// @return Max number of consecutive ones
int Transport_Layer::find_max_ones(const std::string &s)
{
    int one_count = 0, max_ones = 0;
    for (auto character : s)
    {
        if (character == '1')
        {
            one_count++;
            max_ones = std::max(one_count, max_ones);
        }
        else
        {
            one_count = 0;
        }
    }

    return max_ones;
}

std::string Transport_Layer::find_length(const std::string &binary_msg)
{
    int length_of_string = binary_msg.size();
    std::string binary_length = "";

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

std::string Transport_Layer::add_header(const std::string &binary_msg)
{
    std::string stuffed_msg = Transport_Layer::bit_stuff(binary_msg);
    std::string stuffed_length = Transport_Layer::bit_stuff(Transport_Layer::find_length(binary_msg));
    return _SFD + stuffed_length + _EFD + stuffed_msg;
}

int Transport_Layer::get_length_from_header(const std::string &binary_msg)
{
    size_t begin_length = _SFD.length();
    size_t end_length = _EFD.length();

    auto SFD_pos_start = binary_msg.find(_SFD);
    if (SFD_pos_start == std::string::npos)
    {
        std::cerr << "SFD not found in the binary message";
    }

    auto EFD_pos_start = binary_msg.find(_EFD, begin_length);
    if (EFD_pos_start == std::string::npos)
    {
        std::cerr << "EFD not found in the binary message";
    }

    auto length_part_start = SFD_pos_start + begin_length;
    auto length_part_len = EFD_pos_start - length_part_start;

    if (length_part_len <= 0)
    {
        std::cerr << "The length of the message is invalid";
    }

    std::string length_part = Transport_Layer::bit_unstuff(binary_msg.substr(length_part_start, length_part_len));

    return std::stoi(length_part, nullptr, 2);
}

std::string Transport_Layer::remove_header_and_unstuff(const std::string &full_binary_msg)
{
    std::string stripped_msg = full_binary_msg;

    size_t length = stripped_msg.length();
    size_t begin_length = _SFD.length();
    size_t end_length = _EFD.length();

    // Remove begin byte
    if (stripped_msg.find(_SFD) != 0)
    {
        std::cerr << "Beginning is not the begin-byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(0, _SFD.length());
    }

    length = stripped_msg.length();

    // Remove end byte
    auto end_pos_end = stripped_msg.find(_EFD) + end_length - 1;
    auto header_length = end_pos_end + 1;
    stripped_msg.erase(0, header_length);

    return Transport_Layer::bit_unstuff(stripped_msg);
}

std::string Transport_Layer::bit_stuff(const std::string &full_binary_msg)
{
    std::string stuffed = "";
    int consecutiveOnes = 0;

    for (char bit : full_binary_msg)
    {
        stuffed += bit;

        if (bit == '1')
        {
            consecutiveOnes++;
            if (consecutiveOnes == (Transport_Layer::find_max_ones(_SFD) - 1))
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

std::string Transport_Layer::bit_unstuff(const std::string &full_binary_msg)
{
    std::string unstuffed = "";
    int consecutiveOnes = 0;
    size_t i = 0;

    while (i < full_binary_msg.length())
    {
        char bit = full_binary_msg[i];

        if (bit == '1')
        {
            consecutiveOnes++;
            unstuffed += bit;

            /// Skip however many maximum consecutive ones are in the EFD or SFD
            if (consecutiveOnes == Transport_Layer::find_max_ones(_SFD) - 1)
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
            unstuffed += bit;
        }

        i++;
    }

    return unstuffed;
}

std::vector <std::string> Transport_Layer::segment_msg(const std::string &full_binary_msg)
{
    std::string unsegmented_msg = full_binary_msg;

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
    std::cout << "Number of segments: " << _segments_vector.size() << std::endl;

    return _segments_vector;
}

std::string Transport_Layer::combine_segments_to_string()
{
    std::string full_binary_msg = "";
    for (const auto &segment : _segments_vector)
    {
        full_binary_msg += segment;
    }
    return full_binary_msg;
}

void Transport_Layer::add_segment(const std::string &segment)
{
    _segments_vector.push_back(segment);
}

std::string Transport_Layer::get_next_segment()
{
    std::string next_segment = _segments_vector[0];
    _segments_vector.erase(_segments_vector.begin());
    return next_segment;
}

void Transport_Layer::print_segment_vector(const std::vector<std::string> &vector)
{
    int i = 0;
    std::cout << "Number of segments: " << vector.size() << std::endl;
    for (const std::string &element : vector)
    {
        std::cout << "Segment " << i << ":  " << element << std::endl;
        i++;
    }
}



