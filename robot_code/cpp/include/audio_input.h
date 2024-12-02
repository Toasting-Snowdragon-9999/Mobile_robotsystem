#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <portaudio.h>
#include <chrono>
#include <iomanip>
#include "goertzel.h"

#define MILLISECONDS (4000)
#define NUM_CHANNELS (1)
#define SAMPLE_TYPE paFloat32

typedef float SAMPLE;

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
    AudioInput(int sample_rate, int frames_per_buffer);
    ~AudioInput();
    void list_audio_devices();
    void record_audio(int input_device);
    void save_to_wav(const std::string &fileName);
    void audio_open();
    void audio_close();
    void save_to_textfile(const std::string &fileName);
    void read_from_file(const std::string &fileName);
    void initialise_flags();
    void check(bool print, std::vector<int> &test_sequence);
    TestResult check_sequence(std::vector<int> &tones, std::vector<int> &test_sequence);

private:
    int _sample_rate;
    int _frames_per_buffer;
    PaStream *_stream = nullptr;
    PaError _err;
    PaStreamParameters _input_parameters;
    MicSample _mic_data;
      
};

static int read_mic_callback(const void *input_buffer, void *output_buffer,
                             unsigned long frames_per_buffer,
                             const PaStreamCallbackTimeInfo* time_info,
                             PaStreamCallbackFlags status_flags,
                             void *userData);

#endif
