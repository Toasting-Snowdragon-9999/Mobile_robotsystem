#include "audio_input.h"

#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (256)
#define INPUT_DEVICE (14)  /* Device index for the input device 5 default*/


int main(){

    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.audio_close();
	return 0;
}
