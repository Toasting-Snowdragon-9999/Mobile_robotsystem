#ifndef TRANSPORT_LAYER_H
#define TRANSPORT_LAYER_H

#define nibble_size 4
#define mms nibble_size*8 // (Maximum Segment Size)

#include <string>
#include <iostream>
#include <sstream>
#include <vector>


using std::string;

class Transport_Layer{

private:

string _begin_byte = "11110000";
string _end_byte = "11110000";
std::vector <string> msg_segments;

public:

Transport_Layer();

string add_begin_and_end(const string &binary_msg);

string remove_begin_and_end(const string &full_binary_msg);

std::vector <string> segment_msg(const string &full_binary_msg);

string combine_segments(const std::vector <string> &segment_vector);

void print_segment_vector(const std::vector<string> &vector);

};


#endif