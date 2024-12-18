#ifndef WAVE_GENERATOR_H
#define WAVE_GENERATOR_H

#include <portaudio.h>
#include <cmath>
#include <vector>
#include <iostream>

class WaveGenerator {
private:
    unsigned _sample_rate;         // Samples per second
    const float _amplitude = 0.5;              // Amplitude of the sine waves
    const unsigned _duration = 120;              // Duration of each tone in milliseconds
    const unsigned _time_between_sounds = 60;    // Time between each sound that is played (milliseconds)

    std::vector<float> _low_frequencies = {697, 770, 852, 941};
    std::vector<float> _high_frequencies = {1209, 1336, 1477, 1633};
    std::vector<std::vector<float>> _frequency_combinations_DTMF;

    std::vector<int> _sequence;
    std::vector<std::vector<float>> _all_frequencies_to_be_played;
    std::vector<float> _audio_data;  // Combined audio data for playback

    /** 
     * @brief Initialize the frequency combinations
     */
    void initialize_frequency_combinations();
    /**
     * @brief Add a few tones to the start of the played sequence, this gives a better result when the microphone is capturing sound
     */
    void add_start_sequence();
    /**
     * @brief Generate the sine wave pairs for the DTMF sounds
     */
    void generate_sine_wave_pairs();
    /**
     * @brief Apply fade in for a smoother sound
     * @param samples The samples to apply the fade in to
     * @param fadeLength The length of the fade in
     */
    void apply_fade_in(std::vector<float>& samples, int fadeLength);
    /**
     * @brief Apply fade out for a smoother sound
     * @param samples The samples to apply the fade out to
     * @param fadeLength The length of the fade out
     */
    void apply_fade_out(std::vector<float>& samples, int fadeLength);

public:
    /**
     * @brief Construct a new Wave Generator object
     */
    WaveGenerator();
    /**
     * @brief Construct a new Wave Generator object
     * @param sequence The sequence of DTMF tones to play
     * @param sample_rate The sample rate of the signal
     */
    WaveGenerator(std::vector<int>& sequence, int sample_rate);
    /**
     * @brief Destroy the Wave Generator object
     */
    ~WaveGenerator();
    /**
     * @brief Play the DTMF tones
     */
    void play_sounds();
    /**
     * @brief Save the combined audio data to a WAV file
     * @param filename The name of the file to save the audio data to
     */
    void save_to_wav_file(const std::string& filename);
};

#endif
