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
#include "algorithms/dft.h"

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
    /**
     * @brief Construct a new Goertzel object
     */
    Goertzel();
    /**
     * @brief Construct a new Goertzel object
     * @param r The GoertzelResult object
     * @param hyperx The flag to indicate if the signal is captured from a HyperX Quadcast microphone, this changes the threshholds
     * @param sample_rate The sample rate of the signal
     */
    Goertzel(GoertzelResult& r, bool hyperx=false, int sample_rate=16000);
    /**
     * @brief Construct a new Goertzel object
     * @param hyperx The flag to indicate if the signal is captured from a HyperX Quadcast microphone, this changes the threshholds
     * @param sample_rate The sample rate of the signal
     */
    Goertzel(bool hyperx=false, int sample_rate=16000);
    /**
     * @brief Construct a new Goertzel object
     * @param data The signal data
     */
    Goertzel(const std::vector<float> data);
    /**
     * @brief Compute the Goertzel algorithm and saves the result in the _result object and the _freq_from_signals
     */
    void compute_goertzel();
    /**
     * @brief Read the signal data from a file
     * @param file_name The name of the file to read from
     */
    void read_from_file(const std::string &file_name);
    /**
     * @brief This function is used to compute the Goertzel algorithm
     * @param r A GoertzelResult object to store the result
     */
    void translate_signal_goertzel(GoertzelResult& r);
    /**
     * @brief This function using bubble sort to sort the frequencies and amplitudes
     * @param x The frequencies of the signal
     * @param y The index of the frequencies
     */
    void sort(std::vector <double> &x, std::vector <int> &y);
    /**
     * @brief This function is used to detect the DTMF tones, it will map two freq to a DTMF tone
     * @param freq_1 The first frequency
     * @param freq_2 The second frequency
     * @param r The GoertzelResult object to store the result
     */
    void detect_DTMF(int freq_1, int freq_2, GoertzelResult& r);
    /**
     * @brief This function is used to load the data into the _data vector
     * @param data The signal data
     */
    void load_data(const std::vector<float> &data);
    /**
     * @brief This function is used to return the result of the Goertzel algorithm
     * @return GoertzelResult The result of the Goertzel algorithm
     */
    std::vector<int> get_message_vec();
    /**
     * @brief This function is used to detect if the tone is start, stop or escape
     * @param start_stop The tone to detect
     * @param dtmf_tone The DTMF tone to check
     * @return a boolean value weather the tone was detected
     */
    bool detect_bit(std::string start_stop, int dtmf_tone);
    /**
     * @brief This function is used to save the result of the Goertzel algorithm to a json file
     * @param key The key to save the json file
     */
    void save_to_json(const int& key);
    /**
     * @brief This function is used to generate a json string from the result of the Goertzel algorithm
     * @param key The key to save the json file
     * @return std::string The json string to be saved
     */
    std::string generate_json_string(const int& key);
};

#endif // GOERTZEL_H
