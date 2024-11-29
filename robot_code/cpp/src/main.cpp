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
#define INPUT_DEVICE (14)  /* Device index for the input device 5 default*/

int main() {

    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.save_to_textfile("../dtmf_sounds/output.txt");
    //audio_input.read_from_file("../dtmf_sounds/output.txt");
    audio_input.audio_close();
	return 0;
}
