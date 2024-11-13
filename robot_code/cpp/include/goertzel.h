#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <iostream>
#include <cmath>
#include <vector>
#include <complex>
#include <fstream>
#include <algorithm>
#include <map>
#include <chrono>


class Goertzel
{
private:
    
    double _sample_freq = 44100.0;
    int _size_of_signal;
    
    std::vector<double> _data;
    std::vector<int> _DTMF_freq = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<double> _coefficients;
    std::vector<double> _magnitudes;
    std::vector<double> _freq_from_signals;
//    std::vector<int> _message_vec;

    // std::map<std::pair<int, int>, int> _DTMF_mapping = {
    //         {{697, 1209}, 0}, {{697, 1336}, 1}, {{697, 1477}, 2}, {{697, 1633}, 3},
    //         {{770, 1209}, 4}, {{770, 1336}, 5}, {{770, 1477}, 6}, {{770, 1633}, 7},
    //         {{852, 1209}, 8}, {{852, 1336}, 9}, {{852, 1477}, 10}, {{852, 1633}, 11},
    //         {{941, 1209}, 12}, {{941, 1336}, 13}, {{941, 1477}, 14}, {{941, 1633}, 15}
    //     };

    
    void _init_coefficients() {
        _coefficients.resize(_DTMF_freq.size());

        for (int i = 0; i < _DTMF_freq.size(); ++i) {
            double omega = ( (2.0 * M_PI) / _size_of_signal ) * (0.5 + (2.0 * M_PI * _DTMF_freq[i]) / _sample_freq );
            _coefficients[i] = 2.0 * std::cos(omega);
        }
    }


public:

    Goertzel();
    Goertzel(const std::vector<double> data);

    void compute_goertzel();
    void read_from_file(const std::string &file_name);
    void translate_signal_goertzel();
    void sort(std::vector <double> &x, std::vector <int> &y);
    //void detect_DTMF(int freq_1, int freq_2);

};

#endif // GOERTZEL_H
