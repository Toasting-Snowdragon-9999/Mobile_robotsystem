#include <signal_processing.h>

std::vector<int> SIGNAL_PROCESSING::pre_postamble_remover(std::vector<int> message_vec_int){
    if (!(message_vec_int.empty())) {

        message_vec_int.erase(message_vec_int.begin()); 
    }
    if (!(message_vec_int.empty())) {
        
        message_vec_int.erase(message_vec_int.end()-1);
        
    }
    
    _message_vec_int = message_vec_int;
    
    return message_vec_int;
}

std::string SIGNAL_PROCESSING::message_str_binary(){

    std::string message_vect_str;

    for (auto i : _message_vec_int){
    
        if (_int_binary_mapping.find(i) != _int_binary_mapping.end()) {
            
            message_vect_str += _int_binary_mapping[i];
            
        } else {

            std::cerr << "Error finding the corresponding value" << std::endl;
        }
    }
    return message_vect_str;
}