#include "goertzel.h"

Goertzel::Goertzel(){}

Goertzel::Goertzel(const std::vector<float> data)
        : _data(data), _size_of_signal(data.size()) {}

void Goertzel::translate_signal_goertzel(){

    _init_coefficients();

    compute_goertzel();

    sort(_magnitudes, _DTMF_freq);

    detect_DTMF(_freq_from_signals[0], _freq_from_signals[1]);

}

void Goertzel::_init_coefficients()  {
    _coefficients.resize(_DTMF_freq.size());

    for (int i = 0; i < _DTMF_freq.size(); ++i) {
        double omega = ( (2.0 * M_PI) / _size_of_signal ) * (0.5 + (2.0 * M_PI * _DTMF_freq[i]) / _sample_freq );
        _coefficients[i] = 2.0 * std::cos(omega);
    }
}

void Goertzel::compute_goertzel() {
    _magnitudes.resize(_DTMF_freq.size());

    for (int freq_index = 0; freq_index < _DTMF_freq.size(); ++freq_index) {
        
        //double omega = (2.0 * M_PI * _DTMF_freq[freq_index]) / _sample_freq;
        double omega = (2.0 * M_PI) / _size_of_signal * (0.5 + (_size_of_signal * _DTMF_freq[freq_index] / _sample_freq) );
        double cosine = std::cos(omega);
        double coeff = 2.0 * cosine;
        double signal = 0.0;
        double last_signal = 0.0;
        double before_last_signal = 0.0;

        for (int i = 0; i < _size_of_signal; ++i) {

            signal = coeff * last_signal - before_last_signal + _data[i];
            before_last_signal = last_signal;
            last_signal = signal;
        }
        double last_signal_square = last_signal * last_signal;
        double before_last_signal_square = before_last_signal * before_last_signal;

        _magnitudes[freq_index] = std::sqrt(last_signal_square + before_last_signal_square - last_signal * before_last_signal * coeff);

    }
}


void Goertzel::read_from_file(const std::string &file_name){
    std::ifstream in_file(file_name);
    std::string line;

    while (std::getline(in_file, line)) {
        double value = std::stod(line);
        _data.push_back(value);

    }

    _size_of_signal = _data.size();

    in_file.close();
}

void Goertzel::load_data(const std::vector<float> &data){
    _data = data;
    _size_of_signal = data.size();
}

void Goertzel::sort(std::vector <double> &x, std::vector <int> &y){

    bool swap;

    for(int i = 0; i < x.size()-1; i++){

        swap = false;

        for (int j = 0; j < x.size()-1; j++){

            if (x[j] < x[j+1]){
                std::swap(x[j], x[j+1]);
                std::swap(y[j],y[j+1]);

                swap = true;
            }
        }
        if (!swap){
            break;
        }
    }

    _freq_from_signals.push_back(y[0]);
    _freq_from_signals.push_back(y[1]);

    //std::cout << "Frequency found: " << _freq_from_signals[0] << " and " << _freq_from_signals[1] << std::endl;

}


bool Goertzel::detect_DTMF(int freq_1, int freq_2) {
    if (freq_1 > freq_2) {
       std::swap(freq_1, freq_2);
    }

    auto DTMF_freq = _DTMF_mapping.find({freq_1, freq_2});

    if (DTMF_freq != _DTMF_mapping.end()) {
        _message_vec.push_back(DTMF_freq->second);
        std::cout << " DTMF_Freq found: " << DTMF_freq->second << std::endl;
        return true;
    }
    else {

        std::cout<< " DTMF_Freq does not found " << std::endl;
        return false; 
    }

     /* -------------------- FOR DEBUG: -----------------------*/

    //std::cout << "Size of message vector: "<< _message_vec.size() << std::endl;

    //std::cout << "Content of message vector: "<< _message_vec[0] << std::endl;

}

std::vector<int> Goertzel::get_message_vec(){

    return _message_vec;
}

bool Goertzel::detect_start_bit() {
    int freq_1 = 1477;
    int freq_2 = 852;
    if ((_freq_from_signals[0] == freq_1 && _freq_from_signals[1] == freq_2) || (_freq_from_signals[0] == freq_2 && _freq_from_signals[1] == freq_1)) {
        std::cout << "Start bit detected" << std::endl;
        return true;
    }
    return false;
}