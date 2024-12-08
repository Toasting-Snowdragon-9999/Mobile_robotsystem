#ifndef TRANSPORT_LAYER_H
#define TRANSPORT_LAYER_H

#define nibble_size 4
#define mms 32 // (Maximum Segment Size)

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>


class Transport_Layer
{

private:
    std::string _SFD = "11110";
    std::string _EFD = _SFD;
    std::vector<std::string> _segments_vector;

public:
    Transport_Layer();



    std::vector<std::string> get_segments_vector();

    int find_max_ones(const std::string &s);

    bool is_combined_msg_complete(const std::string &combined_msg);

    /// @brief Finds length of the given binary msg
    /// @param binary_msg
    /// @return Length of binary msg in binary
    std::string find_length(const std::string &binary_msg);

    /// @brief Add header to the binary msg after bitstuffing the msg
    /// @param binary_msg
    /// @return The modified binary msg
    std::string add_header(const std::string &binary_msg);

    int get_length_from_header(const std::string &binary_msg);

    /// @brief Remove the SFD-, length and EFD-part of the binary msg
    /// @param full_binary_msg
    /// @return The binary msg without the header
    std::string remove_header_and_unstuff(const std::string &full_binary_msg);

    std::string bit_stuff(const std::string &full_binary_msg);

    std::string bit_unstuff(const std::string &full_binary_msg);

    /// @brief Segments depending on the mms and creates a private vector of segments
    /// @param full_binary_msg
    std::vector <std::string> segment_msg(const std::string &full_binary_msg);

    /// @brief Combines all segments in the private segment vector
    /// @return The recombined binary msg
    std::string combine_segments_to_string();

    /// @brief Add a segment to the private segment vector
    /// @param segment
    void add_segment(const std::string &segment);

    /// @brief Retrieves the first segment of the private segment vector and removes it from the private vector
    /// @return The next segment to be sent to lower layers
    std::string get_next_segment();

    /// @brief Prints out the number of segments and all segments in the private segment vector
    /// @param vector
    void print_segment_vector(const std::vector<std::string> &vector);

    void store_till_length(){

    }


};

#endif