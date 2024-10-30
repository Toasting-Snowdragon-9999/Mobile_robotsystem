// #include <iostream>
// #include <fstream>
// #include "algorithms.h"
#include "audio_input.h"

#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (256)
#define INPUT_DEVICE (14)  /* Device index for the input device 5 default*/


int main(){

	// std::vector<std::complex<double>> a = {{1, 2}, {3, 4}, {5, 6}, {7, 8}};
	// SP::Algorithms<double> A(a);
	// SP::Algorithms<double> B;
	// A.print_data();

	// std::cout << "\nFFT: " << std::endl;
	// B.FFT(a);
	// B.print_data();

	// std::cout << "\nGoertzel: " << std::endl;
	// A.goertzel();

    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.audio_close();
	return 0;
}
