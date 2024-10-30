#ifndef DFT_H
#define DFT_H
#include <iostream>
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <fstream>
#include <ctime>

class DFT {
private:
    std::vector<double> _data;
    std::vector<int> _DTMF_freq;
    int _sample_freq;
    int _size_of_signal;
    // = data.size();

    std::vector<std::complex<double>> _DFT_coef;
    std::vector<double> _abs_coef;

public:
    DFT();
    DFT(const std::vector<double> data);
    void compute_DFT();
    const std::vector<std::complex<double>> get_DFT_coef();
    const std::vector<double> get_Abs_coef();
    void read_from_file(const std::string &fileName);
    void init();
    void sort(std::vector <double> &x, std::vector <int> &y);

};
#endif // DFT_H
