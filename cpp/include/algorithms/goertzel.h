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
#include <iomanip>


struct GoertzelResult {
    int dtmf_tone;
    bool garbage_flag;
    bool tone_flag;
    bool esc_flag;
};

class Goertzel
{
private:
    
    double _sample_freq = 16000.0;
    int _size_of_signal;
    bool _hyperx = false;
    double multiplier;
    std::vector<float> _data;
    std::vector<int> _DTMF_freq = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<double> _coefficients;
    std::vector<double> _magnitudes;
    std::vector<double> _freq_from_signals;
    std::vector<int> _message_vec;
    std::vector<std::string> _json_string;

    GoertzelResult _result;


    std::map<std::pair<int, int>, int> _DTMF_mapping = {
            {{697, 1209}, 0}, {{697, 1336}, 1}, {{697, 1477}, 2}, {{697, 1633}, 3},
            {{770, 1209}, 4}, {{770, 1336}, 5}, {{770, 1477}, 6}, {{770, 1633}, 7},
            {{852, 1209}, 8}, {{852, 1336}, 9}, {{852, 1477}, 10}, {{852, 1633}, 11},
            {{941, 1209}, 12}, {{941, 1336}, 13}, {{941, 1477}, 14}, {{941, 1633}, 15}, {{697, 770}, -1}
        };

    std::map<int, std::pair<int, int>> _reverse_DTMF_mapping = {
            {0, {697, 1209}}, {1, {697, 1336}}, {2, {697, 1477}}, {3, {697, 1633}},
            {4, {770, 1209}}, {5, {770, 1336}}, {6, {770, 1477}}, {7, {770, 1633}},
            {8, {852, 1209}}, {9, {852, 1336}}, {10, {852, 1477}}, {11, {852, 1633}},
            {12, {941, 1209}}, {13, {941, 1336}}, {14, {941, 1477}}, {15, {941, 1633}}, {-1, {697, 770}}
        };
    
    void _init_coefficients();


public:

    Goertzel();
    Goertzel(GoertzelResult& r, bool hyperx=false);
    Goertzel(bool hyperx=false);
    Goertzel(const std::vector<float> data);

    void compute_goertzel();
    void read_from_file(const std::string &file_name);
    void translate_signal_goertzel(GoertzelResult& r);
    void sort(std::vector <double> &x, std::vector <int> &y);
    void detect_DTMF(int freq_1, int freq_2, GoertzelResult& r);
    void load_data(const std::vector<float> &data);
    std::vector<int> get_message_vec();
    bool detect_bit(std::string start_stop, int dtmf_tone);
    void save_to_json(const int& key);
    std::string generate_json_string(const int& key);
};

#endif // GOERTZEL_H
