#include "dft.h"

DFT::DFT(){}

DFT::DFT(const std::vector<double> data)
        : _data(data), _size_of_signal(data.size()){}

void DFT::init(){
    _DTMF_freq = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    _sample_freq = 44100;

    _DFT_coef.resize(_size_of_signal, std::complex<double>(0.0, 0.0));
    _abs_coef.resize(_size_of_signal, 0.0);
    compute_DFT();
    sort(_abs_coef, _DTMF_freq);

}

void DFT::compute_DFT() {
    std::complex<double> i(0.0, -1.0);
    int nr_of_DTMF_freq = _DTMF_freq.size();

    for (int n = 0; n < nr_of_DTMF_freq; ++n) {
        double omega = 2 * M_PI * _DTMF_freq[n];
        double omega_0 = (2 * M_PI) / (_size_of_signal * (1.0 / _sample_freq));
        int nr_sample = std::round(omega / omega_0);

        for (int j = 0; j < _size_of_signal; ++j) {
            double angle = 2 * M_PI * nr_sample * j / _size_of_signal;
            _DFT_coef[nr_sample] += _data[j] * std::exp(std::complex<double>(0.0, -angle));
//                std::cout << "Vec value: " << _DFT_coef[j] << std::endl;


        }

        _abs_coef[n] = std::abs(_DFT_coef[nr_sample]);
        //std::cout << "Abs value for frequency " << _DTMF_freq[n] << ": " << _abs_coef[n] << std::endl;
    }
    //std::cout << "Vec size: " << _DFT_coef.size() << std::endl;
}

const std::vector<std::complex<double>> DFT::get_DFT_coef(){
    return _DFT_coef;
}

const std::vector<double> DFT::get_Abs_coef(){
    return _abs_coef;
}

void DFT::read_from_file(const std::string &fileName){
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

void DFT::sort(std::vector <double> &x, std::vector <int> &y){
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



