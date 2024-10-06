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
	//lyde
	// std::vector<std::vector<int>> sequence1 = {{1,2,3},{4,5,6},{7,8,9},{10,11,12},{13,14,15},{16,0,0}}; //missing 16
    // std::vector<std::vector<int>> sequence2 = {{2,7,4},{16,1,2},{14,8,6},{1,3,9},{15,3,4},{9,16,1}};
    // std::vector<std::vector<int>> sequence3 = {{16,16,16},{1,1,1},{16,16,16},{1,1,1},{16,16,16}};
    // std::vector<std::vector<int>> sequence4 = {{1,2,3}};

	WaveGenerator sound;
	sounds.play_sounds(sequence1);
	//lyde ended

	return 0;
}
