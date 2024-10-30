#ifndef GOERTZEL_H
#define GOERTZEL_H
#include <iostream>
#include <cmath>
#include <vector>
#include <complex>
#include <fstream>
#include <algorithm>
#include <ctime>

class Goertzel
{
private:
    int _sample_freq;
    int _size_of_signal;
    std::vector<double> _data;
    std::vector<int> _DTMF_freq;
    std::vector<double> _coefficients;
    std::vector<double> _magnitudes;

    void _init_coefficients() {
        _coefficients.resize(_DTMF_freq.size());

        for (int i = 0; i < _DTMF_freq.size(); ++i) {
            double omega = (2.0 * M_PI * _DTMF_freq[i]) / _sample_freq;
            _coefficients[i] = 2.0 * std::cos(omega);
        }
    }

public:
    Goertzel();
    Goertzel(const std::vector<double> data);
    void compute_Goertzel();
    void printResults();
    void read_from_file(const std::string &fileName);
    void init();
    void sort(std::vector <double> &x, std::vector <int> &y);
};

#endif // GOERTZEL_H
