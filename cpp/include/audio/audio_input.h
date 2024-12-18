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
#include <mutex>
#include <thread>
#include "algorithms/goertzel.h"

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
    bool hyperx;
};

class AudioInput {
public:

    /**
     * @brief Construct a new Audio Input object
     */
    AudioInput();
    /**
     * @brief Construct a new Audio Input object
     * @param sample_rate The sample rate of the signal
     * @param frames_per_buffer The frames per buffer
     */
    AudioInput(int sample_rate, int frames_per_buffer);
    /**
     * @brief Destroy the Audio Input object
     */
    ~AudioInput();
    /**
     * @brief List the avaliable audio devices
     */
    void list_audio_devices();
    /**
     * @brief Record the audio from the microphone
     * @param input_device The input device to record from
     * @param hyperx The flag to indicate if the signal is captured from a HyperX Quadcast microphone, this changes the threshholds
     */
    void record_audio(int input_device, bool hyperx);
    /**
     * @brief Save the recorded audio to a wav file
     * @param fileName The name of the file to save to
     */
    void save_to_wav(const std::string &fileName);
    /**
     * @brief This opens a port audio stream
     */
    void audio_open();
    /**
     * @brief This closes the port audio stream
     */
    void audio_close();
    /**
     * @brief This function is used to save the recorded audio to a text file
     * @param fileName The name of the file to save to
     */
    void save_to_textfile(const std::string &fileName);
    /**
     * @brief This function is used to read the recorded audio from a text file
     * @param fileName The name of the file to read from
     */
    void read_from_file(const std::string &fileName);
    /**
     * @brief This function is used to initialise the flags
     * @param hx The flag to indicate if the signal is captured from a HyperX Quadcast microphone
     */
    void initialise_flags(bool hx);
    /**
     * @brief This function is used to check the recorded DTMF tones against a known sequence
     * @param print The flag to indicate if the result should be printed
     * @param test_sequence The recorded sequence
     * @return int The number of errors
     */
    int check(bool print, std::vector<int> &test_sequence);
    /**
     * @brief This function is used to check the recorded DTMF tones against a known sequence
     * @param result The result of the test
     * @param tones The recorded DTMF tones
     * @param test_sequence The known sequence
     */
    void check_sequence(TestResult &result, std::vector<int> &tones, std::vector<int> &test_sequence);
    /**
     * @brief This function is used to get the recorded DTMF tones
     * @return std::vector<int> The recorded DTMF tones
     */
    std::vector<int> get_recorded_DTMF_tones();
    /**
     * @brief This function is used to set the stop flag, from outside
     * @param stop The flag to indicate if the recording should stop
     */
    void set_stop_flag(bool stop);
    /**
     * @brief This function is used to set the sample rate and frames per buffer
     * @param sample_rate The sample rate of the signal
     * @param frames_per_buffer The frames per buffer
     */
    void set_values(int sample_rate, int frames_per_buffer);

private:
    int _sample_rate;
    int _frames_per_buffer;
    PaStream *_stream = nullptr;
    PaError _err;
    PaStreamParameters _input_parameters;
    MicSample _mic_data;
    std::mutex _stop_mutex;
};

/**
 * @brief This function is used to read the audio from the microphone, this is called by port audio if a stream is open and when data is ready
 * @param input_buffer The input buffer
 * @param output_buffer The output buffer
 * @param frames_per_buffer The frames per buffer
 * @param time_info The time info
 * @param status_flags The status flags
 * @param userData The user data
 * @return int The status of the callback, wheather or not it should continue
 */
static int read_mic_callback(const void *input_buffer, void *output_buffer,
                             unsigned long frames_per_buffer,
                             const PaStreamCallbackTimeInfo* time_info,
                             PaStreamCallbackFlags status_flags,
                             void *userData);

#endif
