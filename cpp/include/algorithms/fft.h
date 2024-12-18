#ifndef FFT_TEST_H
#define FFT_TEST_H

#include <iostream>
#include <vector>
#include <complex>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <chrono>


class FFT {
private:

    int _size_of_signal = 0;
    double _half_signal_size = 0;
    double _sample_freq = 44100.0;

    /**
     * @brief This function is used to compute the Fast Fourier Transform
     * @param data The signal data
     */
    void _fft(std::vector<std::complex<double>>& data);
    /**
     * @brief This function is used to extract DTMF freq from the FFT, this is saved in the _freq_vec and _abs_vecs
     */
    void _extract_DTMF_freq();
    /**
     * @brief This function is used to sort the frequencies and amplitudes, is used the bubblesort algorithm
     * @param abs_vec The amplitudes of the signal
     * @param freq_vec The frequencies of the signal
     */
    void _sort(std::vector <double> &abs_vec, std::vector <double> &freq_vec);

    std::vector<std::complex<double>> _data;
    std::vector<double> _DTMF_frequencies = {697.0, 770.0, 852.0, 941.0, 1209.0, 1336.0, 1477.0, 1633.0};
    std::vector<std::pair<double, double>> _found_DTMF_frequencies;
    std::vector<double> _frequencies;
    std::vector<double> _half_sampling_amplitude;
    std::vector<double> _freq_vec, _abs_vec;


public:
    /**
     * @brief Reading signal data from a file
     * @param file_name The path of the file to read from
     */
    void read_from_file(const std::string &file_name);

    /**
     * @brief Perform the Fast Fourier Transform
     */
    void perform_fft();
    
};

#endif // FFT_TEST_H
