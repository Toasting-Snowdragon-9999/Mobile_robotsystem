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

	std::vector<uint16_t> initial_test = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

	std::vector<int> test_sequence1 = // 50 numbers
		{14, 0, 5, 3, 5, 7, 11, 2, 8, 1,
		6, 4, 10, 9, 13, 12, 11, 14, 7,
		6, 3, 8, 5, 2, 10, 11, 9, 0, 12,
		4, 13, 1, 15, 14, 0, 2, 5, 8, 11,
		10, 9, 0, 13, 12, 3, 4, 15, 1, 14, 0};

	std::vector<int> test_sequence2 = // 100 numbers
		{14, 0, 5, 3, 5, 7, 11, 2, 8, 1,
		6, 4, 10, 9, 13, 12, 15, 14, 7,
		6, 3, 8, 5, 2, 10, 11, 9, 0, 12,
		4, 13, 1, 15, 14, 0, 2, 5, 8, 11,
		10, 9, 0, 13, 12, 3, 4, 15, 1, 7,
		8, 7, 5, 10, 6, 3, 11, 14, 2, 9,
		4, 13, 7, 0, 8, 12, 6, 15, 1, 2,
		10, 11, 9, 3, 5, 13, 7, 8, 6, 12,
		4, 11, 0, 1, 14, 7, 5, 3, 8, 10,
		1, 2, 11, 13, 12, 9, 6, 15, 0, 14, 0};

std::cout << test_sequence2.size() << std::endl;

try{
	sd.read_json();
	sd.print();
}
catch(SharedDataException &e){
	if (e.error_code() == 21){
	}
	else{std::cout << "[Error] " << e.what() << std::endl;}
}

	WaveGenerator sounds(initial_test);
	sounds.play_sounds();
	std::string filename = "../dtmf_sounds/dtmf_sounds.wav";
	sounds.save_to_wav_file(filename);

	return 0;
}