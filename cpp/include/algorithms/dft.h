#ifndef DFT_H
#define DFT_H

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <fstream>


class DFT {
private:

    double _sample_freq = 16000.0;
    int _size_of_signal;

    std::vector<float> _data;
    std::vector<int> _DTMF_freq  = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<std::complex<double>> _dft_coef;
    std::vector<double> _abs_coef;

public:
    /**
     * @brief Construct a new DFT object
     * 
     */
    DFT();

    /**
     * @brief Construct a new DFT object
     * @param data The signal data
     * 
     */
    DFT(const std::vector<float> data);
    /**
     * @brief The destructor
     * 
     */
    ~DFT();
    /**
     * @brief Compute the Discrete Fourier Transform
     * @return void but it alters the _dft_coef and _abs_coef to give the output.
     * 
     */
    void compute_dft();
    /**
     * @brief Read the signal data from a file
     * @param fileName The name of the file to read from
     * 
     */
    void read_from_file(const std::string &fileName);
    /**
     * @brief This function is used in the compute_dft function to compute the DFT
     * 
     */
    void frequencies_of_signal();
    /**
     * @brief This method is a bubble sort algorithm used for sorting the frequencies of the signal
     * @param x The frequencies of the signal
     * @param y The index of the frequencies
     */
    void sort(std::vector <double> &x, std::vector <int> &y);
    /**
     * @brief This method is used to return the dtf result
     * @return std::vector<double> The result of the dft
     */
    std::vector<double> return_data();
};
#endif
