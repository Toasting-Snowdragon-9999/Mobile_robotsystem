#ifndef FFT_TEST_H
#define FFT_TEST_H


#include <iostream>
#include <vector>
#include <complex>
#include <fstream>
#include <cmath>
#include <algorithm>
#include<numeric>
#include<unordered_map>
#include <chrono>


class FFT {
public:
    void read_from_file(const std::string &file_name);
    void perform_fft();

private:
    std::vector<std::complex<double>> _data;
    int _size_of_signal = 0;
    double _half_signal_size = 0;
    double _sample_freq = 44100.0;
    void _fft(std::vector<std::complex<double>>& data);
    void _detect_DTMF_freq();
    void _sort_DTMF_freq();


    std::vector<double> _DTMF_frequencies = {697.0, 770.0, 852.0, 941.0, 1209.0, 1336.0, 1477.0, 1633.0};
    std::vector<std::pair<double, double>> _found_DTMF_frequencies;
    std::vector<double> _frequencies;
    std::vector<double> _half_sampling_amplitude;
};

#endif // FFT_TEST_H
