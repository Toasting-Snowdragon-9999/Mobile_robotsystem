#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <portaudio.h>
#include "goertzel.h"

#define NUM_SECONDS (3)
#define NUM_CHANNELS (1)
#define SAMPLE_TYPE paFloat32

typedef float SAMPLE;

struct MicSample {
    std::vector<SAMPLE> recorded_samples;
    bool success;
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
