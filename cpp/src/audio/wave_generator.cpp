#include "audio/wave_generator.h"

WaveGenerator::WaveGenerator() {
    initialize_frequency_combinations();
    add_start_sequence();
}

WaveGenerator::WaveGenerator(std::vector<int>& sequence, int sample_rate) : _sequence(sequence), _sample_rate(sample_rate) {
    initialize_frequency_combinations();
    add_start_sequence();

    for (int index : sequence) {
        _all_frequencies_to_be_played.push_back(_frequency_combinations_DTMF[index]);
    }
}

WaveGenerator::~WaveGenerator() {
    // Clean up PortAudio
    Pa_Terminate();
}

void WaveGenerator::initialize_frequency_combinations() {
    for (float low_freq : _low_frequencies) {
        for (float high_freq : _high_frequencies) {
            _frequency_combinations_DTMF.push_back({low_freq, high_freq});
        }
    }
}

void WaveGenerator::add_start_sequence() {
    _all_frequencies_to_be_played.push_back(_frequency_combinations_DTMF[0]);
    _all_frequencies_to_be_played.push_back(_frequency_combinations_DTMF[1]);
    _all_frequencies_to_be_played.push_back(_frequency_combinations_DTMF[2]);
}

void WaveGenerator::apply_fade_in(std::vector<float>& samples, int fadeLength) {
    for (int i = 0; i < fadeLength; ++i) {
        float fadeFactor = static_cast<float>(i) / fadeLength;
        samples[i] *= fadeFactor;
    }
}

void WaveGenerator::apply_fade_out(std::vector<float>& samples, int fadeLength) {
    int sampleCount = samples.size();
    for (int i = 0; i < fadeLength; ++i) {
        float fadeFactor = static_cast<float>(fadeLength - i) / fadeLength;
        samples[sampleCount - fadeLength + i] *= fadeFactor;
    }
}

void WaveGenerator::generate_sine_wave_pairs() {
    for (const auto& frequencies : _all_frequencies_to_be_played) {
        if (frequencies.size() == 2) {
            std::vector<float> samples((_sample_rate * _duration) / 1000, 0.0f);

            for (unsigned i = 0; i < samples.size(); ++i) {
                float time = static_cast<float>(i) / _sample_rate;
                samples[i] = _amplitude * (std::sin(2 * M_PI * frequencies[0] * time) +
                                           std::sin(2 * M_PI * frequencies[1] * time));
            }

            int fade_length = (_sample_rate * (_duration/10)) / 1000;  // 1/10 of the duration
            apply_fade_in(samples, fade_length);
            apply_fade_out(samples, fade_length);

            _audio_data.insert(_audio_data.end(), samples.begin(), samples.end());

            // Add silence between tones
            std::vector<float> silence((_sample_rate * _time_between_sounds) / 1000, 0.0f);
            _audio_data.insert(_audio_data.end(), silence.begin(), silence.end());
        }
    }
}

void WaveGenerator::play_sounds() {
    generate_sine_wave_pairs();

    // Initialize PortAudio
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
        return;
    }

    PaStream* stream;
    err = Pa_OpenDefaultStream(&stream, 0, 1, paFloat32, _sample_rate, paFramesPerBufferUnspecified,
                               nullptr, nullptr);
    if (err != paNoError) {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
        Pa_Terminate();
        return;
    }

    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
        Pa_CloseStream(stream);
        Pa_Terminate();
        return;
    }

    // Play audio
    err = Pa_WriteStream(stream, _audio_data.data(), _audio_data.size());
    if (err != paNoError) {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(err) << std::endl;
    }

    // Close stream
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
}

// void WaveGenerator::save_to_wav_file(const std::string& filename) {
//     SF_INFO sfinfo;
//     sfinfo.channels = 1;  // Mono
//     sfinfo.samplerate = _sample_rate;
//     sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

//     SNDFILE* outFile = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
//     if (!outFile) {
//         std::cerr << "Error opening file: " << sf_strerror(outFile) << std::endl;
//         return;
//     }

//     sf_count_t written = sf_write_float(outFile, _audio_data.data(), _audio_data.size());
//     if (written != _audio_data.size()) {
//         std::cerr << "Error writing to file: " << written << " samples written." << std::endl;
//     }

//     sf_close(outFile);
//     std::cout << "Audio saved to " << filename << std::endl;
// }
