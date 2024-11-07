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

    int _size_of_signal = data.size();

    if (_size_of_signal <= 1) return;

    // Divide the signal into Even & Odd
    std::vector<std::complex<double>> even(_size_of_signal / 2);
    std::vector<std::complex<double>> odd(_size_of_signal / 2);
    
    for (int i = 0; i < _size_of_signal / 2; ++i) {
    
        even[i] = data[i * 2];
        odd[i] = data[i * 2 + 1];
    }

    // Recursiv
    _fft(even);
    _fft(odd);

    // Combine data
    for (int k = 0; k < _size_of_signal / 2; ++k) {
        
        std::complex<double> coef = std::polar(1.0, -2 * M_PI * k / _size_of_signal) * odd[k];
        
        data[k] = even[k] + coef;
        data[k + _size_of_signal / 2] = even[k] - coef;
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

    _detect_DTMF_freq();

    _sort_DTMF_freq();

}

void FFT::_detect_DTMF_freq(){
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
            std::cout << "Frequency: " << target_freq << " not found." << std::endl;
        }
    }

}

void FFT::_sort_DTMF_freq(){
    std::sort(_found_DTMF_frequencies.begin(), _found_DTMF_frequencies.end(),
                [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                    return a.second > b.second; // Sort by amplitude (second element of the pair) in descending order
                });

    // Check if we found at least two frequencies
    if (_found_DTMF_frequencies.size() >= 2) {
        std::cout << "The two DTMF frequencies found: " << _found_DTMF_frequencies[0].first << "  " << _found_DTMF_frequencies[1].first << std::endl;
    } else {
        std::cout << "Not enough DTMF frequencies found." << std::endl;
        }
}
