#ifndef WAVE_GENERATOR_H
#define WAVE_GENERATOR_H

#include <portaudio.h>
#include <cmath>
#include <vector>
#include <iostream>

class WaveGenerator {
private:
    const unsigned _sample_rate = 48000;         // Samples per second
    const float _amplitude = 0.5;              // Amplitude of the sine waves
    const unsigned _duration = 120;              // Duration of each tone in milliseconds
    const unsigned _time_between_sounds = 60;    // Time between each sound that is played (milliseconds)

    std::vector<float> _low_frequencies = {697, 770, 852, 941};
    std::vector<float> _high_frequencies = {1209, 1336, 1477, 1633};
    std::vector<std::vector<float>> _frequency_combinations_DTMF;

    std::vector<int> _sequence;
    std::vector<std::vector<float>> _all_frequencies_to_be_played;
    std::vector<float> _audio_data;  // Combined audio data for playback

    void initialize_frequency_combinations();
    void add_start_sequence();
    void generate_sine_wave_pairs();
    void apply_fade_in(std::vector<float>& samples, int fadeLength);
    void apply_fade_out(std::vector<float>& samples, int fadeLength);

public:
    WaveGenerator();
    WaveGenerator(std::vector<int>& sequence);
    ~WaveGenerator();

    void play_sounds();
    void save_to_wav_file(const std::string& filename);
};

#endif
