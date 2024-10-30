#include "goertzel.h"

Goertzel::Goertzel(){}

Goertzel::Goertzel(const std::vector<double> data)
    : _data(data), _size_of_signal(data.size()) {}

void Goertzel::init(){
    _DTMF_freq = {697, 770, 852, 941, 1209, 1336, 1477, 1633};

    _sample_freq = 44100;

    _init_coefficients();

    compute_Goertzel();

    //printResults();

    sort(_magnitudes, _DTMF_freq);


}

void Goertzel::compute_Goertzel() {
    _magnitudes.resize(_DTMF_freq.size());

    for (int freqIndex = 0; freqIndex < _DTMF_freq.size(); ++freqIndex) {
        double omega = (2.0 * M_PI * _DTMF_freq[freqIndex]) / _sample_freq;
        double sine = std::sin(omega);
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

        double real = (last_signal - before_last_signal * cosine);
        double imag = (before_last_signal * sine);
        _magnitudes[freqIndex] = std::sqrt(real * real + imag * imag);
    }
}

void Goertzel::printResults(){
    for (int i = 0; i < _DTMF_freq.size(); ++i) {
        std::cout << "Frequency: " << _DTMF_freq[i] << " Hz, Magnitude: " << _magnitudes[i] << std::endl;
    }
}

void Goertzel::read_from_file(const std::string &fileName){
    std::ifstream inFile(fileName);
    std::string line;

    while (std::getline(inFile, line)) {
        double value = std::stod(line);
        _data.push_back(value);
       //std::cout << "Line: " << line << std::endl;

    }

    _size_of_signal = _data.size();
    std::cout << "Size of signal: " << _size_of_signal << std::endl;
    inFile.close();
}


void Goertzel::sort(std::vector <double> &x, std::vector <int> &y){
    // Using the bubble sort algorithm
    // Evaluates x and swaps both x and y
    // The smallest value -> largest value
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
     std::cout << "Frequency found: " << y[0] << " and " << y[1] << std::endl;
}


