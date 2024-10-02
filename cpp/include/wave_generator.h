#ifndef WAVE_GENERATOR_H
#define WAVE_GENERATOR_H

#include <SFML/Audio.hpp>
#include <cmath>
#include <iostream>
#include <vector>

class WaveGenerator {

private:

    // some constants
    const unsigned _sample_rate = 44100;   // Samples per second
    const float _amplitude = 12500;       // Amplitude of the sine waves
    const unsigned _duration = 200;         // Duration of the sound in seconds

    // all frquencies & combinations
    std::vector<float> _low_frequencies = {697, 770, 852, 941};
    std::vector<float> _high_frequencies = {1209, 1336, 1477, 1633};
    std::vector<std::vector<float>> _frequency_combinations_DTMF;

    std::vector<std::vector<int>> _sequence;
    std::vector<std::vector<std::vector<float>>> _all_frequencies_to_be_played;

    // vectors to store all sound stuff
    std::vector<std::vector<sf::Int16>> _all_samples;
    std::vector<sf::SoundBuffer> _sound_buffers;
    std::vector<sf::Sound> _sounds;

    void add_zeros_to_start_of_signal();
    void apply_fade_in(std::vector<sf::Int16>& samples, int fadeLength);
    void apply_fade_out(std::vector<sf::Int16>& samples, int fadeLength);
    void generate_sine_wave_pairs();
    void load_all_into_buffers();
    void create_sounds_from_buffers();
    void print_frequency_vector();

public:

    WaveGenerator();
    WaveGenerator(std::vector<std::vector<int>> sequence);

    void play_sounds();

};

#endif