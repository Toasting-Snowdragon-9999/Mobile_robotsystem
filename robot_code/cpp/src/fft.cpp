#include "fft.h"

void FFT::read_from_file(const std::string &file_name) {

    std::ifstream inFile(file_name);
    std::string line;

    while (std::getline(inFile, line)) {

        double value = std::stod(line);
        _data.push_back(std::complex<double>(value, 0.0));

    }

    _size_of_signal = _data.size();
    inFile.close();
}

void FFT::_fft(std::vector<std::complex<double>>& data) {

    int size_of_signal = data.size();

    if (size_of_signal <= 1) return;

    // Divide the signal into Even & Odd
    std::vector<std::complex<double>> even(size_of_signal / 2);
    std::vector<std::complex<double>> odd(size_of_signal / 2);

    for (int i = 0; i < size_of_signal / 2; ++i) {

        even[i] = data[i * 2];
        odd[i] = data[i * 2 + 1];
    }

    // Recursiv
    _fft(even);
    _fft(odd);

    // Combine data
    for (int k = 0; k < size_of_signal / 2; ++k) {

        std::complex<double> coef = std::polar(1.0, -2 * M_PI * k / size_of_signal) * odd[k];

        data[k] = even[k] + coef;
        data[k + size_of_signal / 2] = even[k] - coef;
    }
}

void FFT::perform_fft() {

    int log2_of_signal = std::log2(_size_of_signal);

    int signal_size_with_padding = std::exp2(log2_of_signal+1); // Size with zero-padding

    std::vector<std::complex<double>> padded_data(signal_size_with_padding, {0, 0});

    // Copy original data to padded_data
    for (int i = 0; i < _size_of_signal && i < signal_size_with_padding; ++i) {
        padded_data[i] = _data[i];
    }

    // Perform FFT på Padded data
    _fft(padded_data);

    // Process FFT results
    _half_signal_size = signal_size_with_padding / 2;

    for (int i = 0; i < _half_signal_size; ++i) {

        double abs = std::abs(padded_data[i]);
        _half_sampling_amplitude.push_back(abs);

        double freq = _sample_freq * static_cast<double>(i) / signal_size_with_padding;
        _frequencies.push_back(freq);
    }

    _extract_DTMF_freq();

    _sort(_abs_vec, _freq_vec);

}

void FFT::_extract_DTMF_freq(){
    // Create a map to store frequency-amplitude pairs
    std::unordered_map<double, double> frequency_amplitude_map;

    // Populate the frequency-amplitude map
    for (size_t i = 0; i < _frequencies.size() ; ++i) {
        frequency_amplitude_map[_frequencies[i]] = _half_sampling_amplitude[i];
    }

    double tolerance = 0.8; // Tolerance for matching DTMF frequencies

    // Iterate over the target DTMF frequencies and find corresponding amplitudes
    for (double target_freq : _DTMF_frequencies) {
        bool found = false; // Flag to indicate if we found a match

        for (const auto& pair : frequency_amplitude_map) {
            
            if (std::abs(pair.first - target_freq) < tolerance) {
                
                _found_DTMF_frequencies.emplace_back(pair.first, pair.second); // Store found frequency and amplitude
                
                found = true;
                break; // Exit the loop once a match is found
            }
        }
        if (!found) {
   //         std::cout << "Frequency: " << target_freq << " not found." << std::endl;
        }
    }

    for(int i = 0; i < _found_DTMF_frequencies.size(); i++){

        double freq = _found_DTMF_frequencies[i].first;
        double abs = _found_DTMF_frequencies[i].second;

        _freq_vec.push_back(freq);
        _abs_vec.push_back(abs);
    }
}

void FFT::_sort(std::vector <double> &abs_vec, std::vector <double> &freq_vec){

    bool swap;
    for(int i = 0; i < abs_vec.size()-1; i++){
        
        swap = false;
        
        for (int j = 0; j < abs_vec.size()-1; j++){
            if (abs_vec[j] < abs_vec[j+1]){
        
                std::swap(abs_vec[j], abs_vec[j+1]);
                std::swap(freq_vec[j],freq_vec[j+1]);
        
                swap = true;
            }
        }
        if (!swap){
            break;
        }
    }

    std::vector<double> _freq_from_signals;
    
    _freq_from_signals.push_back(freq_vec[0]);
    _freq_from_signals.push_back(freq_vec[1]);

  //  std::cout << "Frequency found: " << _freq_from_signals[0] << " and " << _freq_from_signals[1] << std::endl;

}
