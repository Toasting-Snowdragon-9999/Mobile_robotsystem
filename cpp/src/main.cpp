#include <iostream>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"

int main(){
	SharedData sd;
	std::vector<std::vector<uint16_t>> sequence1;
	try{
		sequence1 = sd.read_shared_data();
		sd.print();
	}
	catch(SharedDataException &e){
		if (e.error_code() == 21){
		}
		else{std::cout << "[Error] " << e.what() << std::endl;}
	}

	WaveGenerator sounds(sequence1);
	sounds.play_sounds();
	std::string filename = "../dtmf_sounds/dtmf_sounds.wav";
	sounds.save_to_wav_file(filename);

	return 0;
}