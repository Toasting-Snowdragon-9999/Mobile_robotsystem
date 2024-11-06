#ifndef FFT_H
#define FFT_H
#include <iostream>
#include <cmath>
#include <vector>
#include <complex>
#include <fstream>
#include <algorithm>

class FFT
{
private:
    std::vector<std::complex<double>> _data;
    int _size_of_signal = 0;
     std::vector<double> _abs;
     std::vector<double> _freq_vec;
     double _freq_val;
     double _sample_freq = 44100;
     double _actual_size_of_signal;
     double _size_of_signal_with_zero;

public:
    FFT();
    FFT(std::vector<double> data);
    void compute_FFT(std::vector<std::complex<double>>& data);
    void read_from_file(const std::string &fileName);
    std::vector<std::complex<double>> get_data();
    std::vector<int> abs_coef();
    void sort(std::vector <double> &x, std::vector <double> &y);
    void perform_FFT();
    void sort_FFT();
};

#endif // FFT_H
