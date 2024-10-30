#include "audio_input.h"
#include <unistd.h>
#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (256)
#define INPUT_DEVICE (14)  /* Device index for the input device 5 default*/


int main(){

    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    sleep(2);
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.save_to_textfile("../dtmf_sounds/output.txt");
    audio_input.read_from_file("../dtmf_sounds/output.txt");
    audio_input.audio_close();
	return 0;
}
