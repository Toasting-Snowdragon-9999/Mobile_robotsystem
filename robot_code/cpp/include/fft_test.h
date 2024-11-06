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
    void read_from_file(const std::string &fileName);
    void perform_fft();
    void print_top_frequencies(int top_n = 7) const;

private:
    std::vector<std::complex<double>> _data;
    int _size_of_signal = 0;
    double _sample_freq = 44100.0;
    void fft(std::vector<std::complex<double>>& x);
};

#endif // FFT_TEST_H
