#ifndef SIGNAL_PROCESSING_H
#define SIGAL_PROCESSING_H

#include <iostream>
#include <vector>
#include <map>
#include <string>


class SignalProcessing{

private:
    
    std::vector<int> _message_vec_int;

    std::map<int, std::string> _int_binary_mapping = {
            {0, "0000"}, {1, "0001"}, {2, "0010"}, {3, "0011"},
            {4, "0100"}, {5, "0101"}, {6, "0110"}, {7, "0111"},
            {8, "1000"}, {9, "1001"}, {10, "1010"}, {11, "1011"},
            {12, "1100"}, {13, "1101"}, {14, "1110"}, {15, "1111"}
    };

public:
    SignalProcessing();
    SignalProcessing(std::vector<int> message_vec_int);
    std::vector<int> pre_postamble_remover(std::vector<int> message_vec);
    std::string message_str_binary();
    std::vector<int> convert_to_dtmf(std::string message_str);


};

#endif