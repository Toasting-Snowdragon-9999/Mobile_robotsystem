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
	std::vector<std::vector<uint16_t>> sequence1 = {{14,1,2},{3,4,5},{6,7,8},{9,10,11},{12,13,14}};
	std::vector<std::vector<uint16_t>> lauras_seq = {{14,10,10},{11,1,0}, {2, 3, 4}, {3, 3, 7}, {10, 12, 1}, {0, 0, 8}, {13, 12, 1}, {15, 1, 2}, {0, 9, 8}, {1, 6, 5}, {12, 3, 3}, {2, 1, 15},{12,10,10}, {11,1,0}, {2, 3, 4}, {3, 3, 7}, {10, 12, 1}, {0, 0, 8}, {13, 12, 1}, {15, 1, 2}, {0, 9, 8}, {1, 6, 5}, {12, 3, 3}, {2, 1, 14}};
	std::vector<std::vector<uint16_t>> sarahsequence = {{1,2,3}, {14,0,1}, {2,3,4}, {5,6,6}, {7,8,8}, {9,10,11}, {12,13,14}};
	std::vector<std::vector<uint16_t>> pure_tone = {{14, 15, 15},{15, 15, 15},{15, 15, 15},{15, 15, 15},{15, 15, 15},{15, 15, 15},{15, 15, 15},{15, 15, 14}};
	std::vector<std::vector<uint16_t>> test_esc = {{14,	0,10}, {11,1,0}, {15,14,10}, 
												   {13,9,0}, {1,2,3}, {15,14,9}, 	
												   {4,5,6}, {7,8,9}, {15,14,2}, 
												   {3,3,7}, {15,15,15}, {8,14,0}};


 	std::vector<std::vector<uint16_t>> chadsequence = {
		{14, 5, 3}, {12, 7, 0}, {1, 15, 9}, 
		{10, 6, 2}, {4, 13, 8}, {3, 11, 1}, 
		{2, 0, 15}, {5, 9, 7}, {12, 4, 8}, 
		{6, 3, 13}, {11, 2, 10}, {7, 1, 0}, 
		{15, 9, 5}, {4, 6, 12}, {3, 8, 13}, 
		{2, 10, 1}, {0, 15, 7}, {9, 5, 4},
		{6, 12, 11}, {8, 13, 3}, {7, 1, 2},
		{13, 0, 10}, {15, 9, 4}, {13, 5, 6},
		{3, 4, 5}, {6, 7, 8}, {9, 10, 11},
		{6, 3, 13}, {11, 2, 10}, {7, 1, 0}, 
		{15, 9, 5}, {4, 6, 12}, {3, 8, 13}, //l
		{2, 10, 1}, {0, 15, 7}, {9, 5, 4}, //l
		{6, 12, 11}, {8, 13, 3}, {7, 1, 2},
		/*{14, 0, 10}, {15, 9, 4}, {13, 5, 6},
		{3, 4, 5}, {6, 7, 8}, {9, 10, 11},
		{6, 3, 13}, {11, 2, 10}, {7, 1, 0}, 
		{15, 9, 5}, {4, 6, 12}, {3, 8, 13}, 
		{2, 10, 1}, {0, 15, 7}, {9, 5, 4},
		{6, 12, 11}, {8, 13, 3}, {7, 1, 2},
		{14, 0, 10}, {15, 9, 4}, {13, 5, 6},
		{3, 4, 5}, {6, 7, 8}, {9, 10, 11},
		{6, 3, 13}, {11, 2, 10}, {7, 1, 0}, 
		{15, 9, 5}, {4, 6, 12}, {3, 8, 13}, 
		{2, 10, 1}, {0, 15, 7}, {9, 5, 4},
		{6, 12, 11}, {8, 13, 3}, {7, 1, 2},
		{14, 0, 10}, {15, 9, 4}, {13, 5, 6},
		{3, 4, 5}, {6, 7, 8}, {9, 10, 11},
		{6, 3, 13}, {11, 2, 10}, {7, 1, 0},*/
		/*{2, 10, 1},*/{7, 8, 14}
	};

	try{
		sd.read_json();
		sd.print();
	}
	catch(SharedDataException &e){
		if (e.error_code() == 21){
		}
		else{std::cout << "[Error] " << e.what() << std::endl;}
	}

	WaveGenerator sounds(test_esc);
	sounds.play_sounds();
	std::string filename = "../dtmf_sounds/dtmf_sounds.wav";
	sounds.save_to_wav_file(filename);

	return 0;
}