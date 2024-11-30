#ifndef TRANSPORT_LAYER_H
#define TRANSPORT_LAYER_H

#include <string>
#include <iostream>
#include <sstream>


using std::string;

class Transport_Layer{

private:

string _begin_byte = "11110000";
string _end_byte = "11110000";

public:

Transport_Layer();

string add_begin_and_end(const string &binary_msg);

string remove_begin_and_end(const string &full_binary_msg);


};


#endif