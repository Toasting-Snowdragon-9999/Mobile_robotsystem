#include "fft.h"
#include "goertzel.h"
#include "dft.h"
#include "signal_processing.h"
#include "audio_input.h"
#include <unistd.h>
#include <iostream>
#include <fstream>
#define SAMPLE_RATE (16000)
#define FRAMES_PER_BUFFER (480)
#define INPUT_DEVICE (15)  /* Device index for the input device 5 default*/

int main() {
    std::vector<int> test_sequence = {14, 0, 5, 3, 5, 7, 11, 2, 8, 1, 
                                    6, 4, 10, 9, 13, 12, 11, 14, 7, 
                                    6, 3, 8, 5, 2, 10, 11, 9, 0, 12, 
                                    4, 13, 1, 15, 14, 0, 2, 5, 8, 11, 
                                    10, 9, 0, 13, 12, 3, 4, 15, 1, 14, 0}; //50 elements

    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.save_to_textfile("../dtmf_sounds/output.txt");
    audio_input.check(false, test_sequence);
    audio_input.audio_close();
	return 0;
}
