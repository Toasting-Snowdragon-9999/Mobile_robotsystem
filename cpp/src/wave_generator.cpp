#include "wave_generator.h"

WaveGenerator::WaveGenerator()
{
}

WaveGenerator::WaveGenerator(std::vector<std::vector<uint16_t>> sequence) : _sequence(sequence)
{
    _frequency_combinations_DTMF.push_back({0,0});
    for (int i = 0; i < _low_frequencies.size(); i++)
    {
        std::vector<float> temp_vec;
        for (int j = 0; j < _high_frequencies.size(); j++)
        {
            temp_vec.push_back(_low_frequencies[i]);
            temp_vec.push_back(_high_frequencies[j]);

            _frequency_combinations_DTMF.push_back(temp_vec);
            temp_vec.clear();
        }
    }
    add_zeros_to_start_of_signal();

    for (int i = 0; i < sequence.size(); i++)
    {
        std::vector<std::vector<float>> temp_vec;
        int j = 0;
        for (int j = 0; j < sequence[i].size(); j++) // Keep pushing frequencies no matter how many inside
        {
            temp_vec.push_back(_frequency_combinations_DTMF[sequence[i][j]+1]);
        }

        _all_frequencies_to_be_played.push_back(temp_vec);
        temp_vec.clear();
    }
}
void WaveGenerator::add_zeros_to_start_of_signal()
{
    std::vector<std::vector<float>> temp_vec;
    temp_vec.push_back({0, 0});
    temp_vec.push_back({0, 0});
    temp_vec.push_back({0, 0});

    _all_frequencies_to_be_played.push_back(temp_vec);
}
void WaveGenerator::apply_fade_in(std::vector<sf::Int16> &samples, int fadeLength)
{
    for (int i = 0; i < fadeLength; ++i)
    {
        float fadeFactor = static_cast<float>(i) / fadeLength;        // Linear fade factor from 0 to 1
        samples[i] = static_cast<sf::Int16>(samples[i] * fadeFactor); // Apply fade-in to the sample
    }
}

// Apply a fade-out effect by gradually decreasing amplitude from 1 to 0 over the fade length
void WaveGenerator::apply_fade_out(std::vector<sf::Int16> &samples, int fadeLength)
{
    int sampleCount = samples.size();
    for (int i = 0; i < fadeLength; ++i)
    {
        float fadeFactor = static_cast<float>(fadeLength - i) / fadeLength;                                                 // Linear fade factor from 1 to 0
        samples[sampleCount - fadeLength + i] = static_cast<sf::Int16>(samples[sampleCount - fadeLength + i] * fadeFactor); // Apply fade-out to the sample
    }
}
void WaveGenerator::generate_sine_wave_pairs()
{
    // Iterates through each pair of frequencies
    for (int j = 0; j < _all_frequencies_to_be_played.size(); j++)
    {
        for (const auto &frequencies : _all_frequencies_to_be_played[j])
        {
            if (frequencies.size() == 2)
            {
                std::cout << "Generating samples for frequencies: " << frequencies[0] << " Hz and " << frequencies[1] << " Hz" << std::endl;

                // Generates samples for one sound at a time
                int conversion_to_seconds = 1000;
                std::vector<sf::Int16> samples(_sample_rate * _duration / conversion_to_seconds);
                for (unsigned i = 0; i < samples.size(); ++i)
                {
                    float time = static_cast<float>(i) / _sample_rate;
                    samples[i] = static_cast<sf::Int16>(
                        _amplitude * (std::sin(2 * M_PI * frequencies[0] * time) + std::sin(2 * M_PI * frequencies[1] * time)));
                }

                int fade_length = 100; // ms
                apply_fade_in(samples, fade_length);
                apply_fade_out(samples, fade_length);

                _all_samples.push_back(samples);
            }
            else
            {
                std::cerr << "Warning: Each inner vector must contain exactly two frequencies." << std::endl;
            }
        }
    }
    // return _all_samples;  // Return the generated samples for all pairs
}
void WaveGenerator::load_all_into_buffers()
{
    // Load the samples for each pair of frequencies into a sound buffer
    for (const auto &samples : _all_samples)
    {
        sf::SoundBuffer buffer;
        if (!buffer.loadFromSamples(&samples[0], samples.size(), 1, _sample_rate))
        {
            std::cerr << "Failed to load sound buffer!" << std::endl;
            // return -1;
        }
        _sound_buffers.push_back(std::move(buffer));
    }
}
void WaveGenerator::create_sounds_from_buffers()
{
    // After generating all sound buffers, play the sounds sequentially
    for (auto &buffer : _sound_buffers)
    {
        sf::Sound sound;
        sound.setBuffer(buffer);
        _sounds.push_back(std::move(sound));
    }
}
void WaveGenerator::play_sounds()
{
    generate_sine_wave_pairs();
    load_all_into_buffers();
    create_sounds_from_buffers();
    print_frequency_vector();

    // Play all sounds sequentially
    for (auto &sound : _sounds)
    {
        sound.play();
        sf::sleep(sf::milliseconds(_duration + _time_between_sounds));
    }
}
void WaveGenerator::print_frequency_vector()
{
    for (auto a : _all_frequencies_to_be_played)
    {
        for (auto b : a)
        {
            std::cout << " { " << b[0] << " Hz, " << b[1] << " Hz } ";
        }
        std::cout << "\n";
    }
}

void WaveGenerator::save_to_wav_file(const std::string &filename)
{
    std::vector<sf::Int16> combined_samples;

    // Number of samples for the silence (delay between sounds)
    int silence_duration_in_samples = (_sample_rate * _time_between_sounds) / 1000; // Convert ms to number of samples
    std::vector<sf::Int16> silence(silence_duration_in_samples, 0);                 // Silence is represented by 0-valued samples

    // Combine all the samples with silence in between
    for (size_t i = 0; i < _all_samples.size(); ++i)
    {
        combined_samples.insert(combined_samples.end(), _all_samples[i].begin(), _all_samples[i].end());

        if (i != _all_samples.size() - 1)
        {
            combined_samples.insert(combined_samples.end(), silence.begin(), silence.end());
        }
    }
    SF_INFO sfinfo;
    sfinfo.channels = 1; // Mono (can be changed to 2 for stereo if needed)
    sfinfo.samplerate = _sample_rate;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16; // This sets the file format to wav and 16-bit PCM (Pulse Code Modulation), it was recommended.
    SNDFILE *outFile = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
    if (!outFile)
    {
        std::cerr << "Error: could not open file for writing: " << sf_strerror(outFile) << std::endl; // Should throw a custom WaveGenError
        return;
    }
    std::cout << combined_samples.size() << std::endl;
    sf_count_t count = sf_write_short(outFile, combined_samples.data(), combined_samples.size()); // Here sf_write_short is used to write to 16-bit register for sound storage, thats means each sample is 16 bits aka a short int.
    if (count != combined_samples.size())
    {
        std::cerr << "Error: only wrote " << count << " samples out of " << combined_samples.size() << std::endl;
    }

    sf_close(outFile);
    std::cout << "Successfully saved " << filename << std::endl;
}