#ifndef DFT_H
#define DFT_H

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <fstream>


class DFT {
private:

    double _sample_freq = 44100.0;
    int _size_of_signal;

    std::vector<double> _data;
    std::vector<int> _DTMF_freq  = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<std::complex<double>> _dft_coef;
    std::vector<double> _abs_coef;

public:

    DFT();
    DFT(const std::vector<double> data);

    void compute_dft();
    void read_from_file(const std::string &fileName);
    void frequencies_of_signal();
    void sort(std::vector <double> &x, std::vector <int> &y);

};
#endif // DFT_H
