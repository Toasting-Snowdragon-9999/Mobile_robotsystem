#include <interfaces/signal_processing.h>

SignalProcessing::SignalProcessing(){
}

SignalProcessing::SignalProcessing(std::vector<int> message_vec_int): _message_vec_int(message_vec_int){
}

std::vector<int> SignalProcessing::pre_postamble_remover(std::vector<int> message_vec_int){
    if (!(message_vec_int.empty())) {

        message_vec_int.erase(message_vec_int.begin()); 
    }
    if (!(message_vec_int.empty())) {
        
        message_vec_int.erase(message_vec_int.end()-1);
        
    }
    
    _message_vec_int = message_vec_int;
    
    return message_vec_int;
}

std::string SignalProcessing::message_str_binary(){

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

std::vector<int> SignalProcessing::convert_to_dtmf(std::string message_str){
    std::vector<int> dtmf_tones;
    for (size_t i = 0; i < message_str.length(); i += 4){
        std::string temp = message_str.substr(i, 4);
        // std::cout << temp << std::endl; 
        for (auto it = _int_binary_mapping.begin(); it != _int_binary_mapping.end(); it++){
            if (it->second == temp){
                dtmf_tones.push_back(it->first);
            }
        }
    }
    return dtmf_tones;
}