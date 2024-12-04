#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <portaudio.h>
#include <chrono>
#include <iomanip>
#include <cmath>
#include "algorithms/goertzel.h"

#define NUM_CHANNELS (1)
#define SAMPLE_TYPE paFloat32
typedef float SAMPLE;

namespace Globals {
    const float thresh_hold = 0.5;
    const std::vector <int> preamble = {14, 0};
    const int esc_tone = 15;
    static std::vector <float> buffer;
}

struct TestResult{
    std::vector <std::string> error;
    int success;
    int failure;
};

struct MicSample {
    std::vector<std::vector<SAMPLE>> recorded_samples;
    std::vector<int> recorded_DTMF_tones;
    GoertzelResult result_in_mic;
    int iterator;
    bool pre_success;
    bool stop;  
    bool pre_1_flag;
};

class AudioInput {
public:
    AudioInput(int sample_rate, int frames_per_buffer, int time_ms=0);
    ~AudioInput();
    void list_audio_devices();
    void record_audio(int input_device);
    void save_to_wav(const std::string &fileName);
    void audio_open();
    void audio_close();
    void save_to_textfile(const std::string &fileName);
    void read_from_file(const std::string &fileName);
    void check(bool print, std::vector<int> &test_sequence);
    void check_sequence(TestResult &result, std::vector<int> &tones, std::vector<int> &test_sequence);
    std::vector<int> get_recorded_DTMF_tones();

private:
    int _sample_rate;
    int _frames_per_buffer;
    int _ms;
    PaStream *_stream = nullptr;
    PaError _err;
    PaStreamParameters _input_parameters;
    MicSample _mic_data;
    void initialise_flags();

};

static int read_mic_callback(const void *input_buffer, void *output_buffer,
                             unsigned long frames_per_buffer,
                             const PaStreamCallbackTimeInfo* time_info,
                             PaStreamCallbackFlags status_flags,
                             void *userData);

#endif
