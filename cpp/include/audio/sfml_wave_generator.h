#ifndef WAVE_GENERATOR_H
#define WAVE_GENERATOR_H

#include <SFML/Audio.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include <sndfile.h> 

class WaveGenerator {

private:

    // some constants

    const unsigned _sample_rate = 16000;         // Samples per second
    const float _amplitude = 12500;              // Amplitude of the sine waves
    const unsigned _duration = 120;              // Duration of the sound in seconds
    const unsigned _time_between_sounds = 120;    // Time between each sound that is played

    // all frquencies & combinations
    std::vector<float> _low_frequencies = {697, 770, 852, 941};
    std::vector<float> _high_frequencies = {1209, 1336, 1477, 1633};
    std::vector<std::vector<float>> _frequency_combinations_DTMF;

    std::vector<int> _sequence;
    std::vector<std::vector<float>> _all_frequencies_to_be_played;

    // vectors to store all sound stuff
    std::vector<std::vector<sf::Int16>> _all_samples;
    std::vector<sf::SoundBuffer> _sound_buffers;
    std::vector<sf::Sound> _sounds;
    /**
     * @brief This function is used to add the start sequence to the sequence, this is done for a better result when the microphone is capturing sound
     */
    void add_start_sequence();
    /**
     * @brief This function is used to add zeros to the start of the signal, this is done for a better sound when playing the sound
     */
    void add_zeros_to_start_of_signal();
    /**
     * @brief This function applies a fade in for a smoother sound
     */
    void apply_fade_in(std::vector<sf::Int16>& samples, int fadeLength);
    /**
     * @brief This function applies a fade out for a smoother sound
     */
    void apply_fade_out(std::vector<sf::Int16>& samples, int fadeLength);
    /**
     * @brief This function is used to generate the sine wave pairs, the dtmf sounds
     */
    void generate_sine_wave_pairs();
    /**
     * @brief This function is used to load our DTMF soynds into the sound buffers
     */
    void load_all_into_buffers();
    /**
     * @brief This function is used to create the sounds from the sound buffers
     */
    void create_sounds_from_buffers();
    /**
     * @brief This function is used to print the frequency vector
     */
    void print_frequency_vector();

public:
    /**
     * @brief Construct a new Wave Generator object
     */
    WaveGenerator();
    /**
     * @brief Construct a new Wave Generator object
     * @param sequence The sequence to be played
     */
    WaveGenerator(std::vector<int> &sequence);
    /**
     * @brief Destroy the Wave Generator object
     */
    ~WaveGenerator();
    /**
     * @brief This function is used to save the generated sound to a wav file
     */
    void save_to_wav_file(const std::string& filename);
    /**
     * @brief This function is used to play the sounds
     */
    void play_sounds();
};

#endif