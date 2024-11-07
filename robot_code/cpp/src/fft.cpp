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

void FFT::fft(std::vector<std::complex<double>>& data) {
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
    fft(even);
    fft(odd);

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
    fft(padded_data);

    // Process FFT results
    int half_signal_size = signal_size_with_padding / 2;
    std::vector<double> half_sampling_amplitude(half_signal_size);
    std::vector<double> frequencies(half_signal_size);

    for (int i = 0; i < half_signal_size; ++i) {
        half_sampling_amplitude[i] = std::abs(padded_data[i]);
        frequencies[i] = _sample_freq * static_cast<double>(i) / signal_size_with_padding;
    }

    // Find top frequencies
    std::vector<int> indices(half_signal_size);
    std::iota(indices.begin(), indices.end(), 0);
    
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return half_sampling_amplitude[a] > half_sampling_amplitude[b];
    });

    // // The 8 DTMF-frequencies
    // std::vector<double> target_frequencies = {697.0, 770.0, 852.0, 941.0, 1209.0, 1336.0, 1477.0, 1633.0};

    // // Create a map to store frequency-amplitude pairs
    // std::unordered_map<double, double> frequency_amplitude_map;

    // for (size_t i = 0; i < frequencies.size(); ++i) {
    //     frequency_amplitude_map[frequencies[i]] = half_sampling_amplitude[i];


    // }

    // double tolerance = 0.8;

    // // Vector to hold found frequencies and their amplitudes
    // std::vector<std::pair<double, double>> found_frequencies;

    // for (double target : target_frequencies) {
    //     bool found = false; // Flag to indicate if we found a match
    //     for (const auto& pair : frequency_amplitude_map) {
    //         if (std::abs(pair.first - target) < tolerance) {
    //             found_frequencies.emplace_back(pair.first, pair.second); // Store found frequency and amplitude
    //             found = true;
    //             break; // Exit the loop once found
    //         }
    //     }
    //     if (!found) {
    //         std::cout << "Frequency: " << target << " not found." << std::endl;
    //     }
    // }

    // // Sort the found frequencies by amplitude in descending order
    // std::sort(found_frequencies.begin(), found_frequencies.end(),
    //             [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
    //                 return a.second > b.second; // Sort by amplitude (second element of the pair) in descending order
    //             });

            // Print the sorted found frequencies and their amplitudes
//            std::cout << "Sorted Found Frequencies by Amplitude:" << std::endl;

//            std::cout << "The two DTMF frequencies found: " << found_frequencies[0].first << "  " << found_frequencies[1].first << std::endl;
// The 8 DTMF-frequencies
std::vector<double> target_frequencies = {697.0, 770.0, 852.0, 941.0, 1209.0, 1336.0, 1477.0, 1633.0};

// Create a map to store frequency-amplitude pairs
std::unordered_map<double, double> frequency_amplitude_map;

// Populate the frequency-amplitude map
for (size_t i = 0; i < frequencies.size(); ++i) {
    frequency_amplitude_map[frequencies[i]] = half_sampling_amplitude[i];
}

double tolerance = 0.8; // Tolerance for matching DTMF frequencies

// Vector to hold found frequencies and their amplitudes
std::vector<std::pair<double, double>> found_frequencies;

// Iterate over the target DTMF frequencies and find corresponding amplitudes
for (double target : target_frequencies) {
    bool found = false; // Flag to indicate if we found a match
    for (const auto& pair : frequency_amplitude_map) {
        if (std::abs(pair.first - target) < tolerance) {
            found_frequencies.emplace_back(pair.first, pair.second); // Store found frequency and amplitude
            found = true;
            break; // Exit the loop once a match is found
        }
    }
    if (!found) {
        std::cout << "Frequency: " << target << " not found." << std::endl;
    }
}

// Sort the found frequencies by amplitude in descending order
std::sort(found_frequencies.begin(), found_frequencies.end(),
            [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                return a.second > b.second; // Sort by amplitude (second element of the pair) in descending order
            });

// Check if we found at least two frequencies
if (found_frequencies.size() >= 2) {
    std::cout << "The two DTMF frequencies found: " << found_frequencies[0].first << "  " << found_frequencies[1].first << std::endl;
} else {
    std::cout << "Not enough DTMF frequencies found." << std::endl;
}

}
