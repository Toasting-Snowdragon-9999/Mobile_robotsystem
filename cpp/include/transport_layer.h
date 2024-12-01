#ifndef TRANSPORT_LAYER_H
#define TRANSPORT_LAYER_H

#define nibble_size 4
#define mms nibble_size*8 // (Maximum Segment Size)

#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>


using std::string;

class Transport_Layer{

private:

string SFD = "11110";
string EFD = "11110";
std::vector <string> _segments_vector;


public:

Transport_Layer();

std::vector <string> get_segments_vector();

/// @brief Finds length of the given binary msg
/// @param binary_msg
/// @return Length of binary msg in binary
string find_length(const string &binary_msg);

/// @brief Add header to the binary msg
/// @param binary_msg
/// @return The modified binary msg
string add_header(const string &binary_msg);

/// @brief Remove the begin- and end-byte of the binary msg
/// @param full_binary_msg
/// @return The binary msg without begin- and end-bytes
string remove_header(const string &full_binary_msg);

string bit_stuff(const string &full_binary_msg);

string bit_unstuff(const string &full_binary_msg);


/// @brief Segments depending on the mms and creates a private vector of segments
/// @param full_binary_msg
void segment_msg(const string &full_binary_msg);

/// @brief Combines all segments in the private segment vector
/// @return The recombined binary msg
string combine_segments_to_string();

/// @brief Add a segment to the private segment vector
/// @param segment
void add_segment(const string &segment);

/// @brief Retrieves the first segment of the private segment vector and removes it from the private vector
/// @return The next segment to be sent to lower layers
string get_next_segment();

/// @brief Prints out the number of segments and all segments in the private segment vector
/// @param vector
void print_segment_vector(const std::vector<string> &vector);

};


#endif