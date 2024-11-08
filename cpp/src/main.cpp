#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

#include "read_shared_data.h"
#include "wave_generator.h"

int main(){
	// std::filesystem::path currentPath = std::filesystem::current_path();
    // std::cout << "Current working directory: " << currentPath << std::endl;
	// return 0;
	std::string path_gui = "../Docs/shared_file.json";
	std::string path_debug = "../../Docs/shared_file.json";
	SharedData sd(path_debug);
	std::vector<std::vector<uint16_t>> sequence1;
	try{
		sd.read_json();
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