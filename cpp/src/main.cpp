#include "wave_generator.h"

int main() {

	std::vector<std::vector<int>> sequence1 = {{1,2,3},{4,5,6},{7,8,9},{10,11,12},{13,14,15},{16,0,0}}; //missing 16
    std::vector<std::vector<int>> sequence2 = {{2,7,4},{16,1,2},{14,8,6},{1,3,9},{15,3,4},{9,16,1}};
    std::vector<std::vector<int>> sequence3 = {{16,16,16},{1,1,1},{16,16,16},{1,1,1},{16,16,16}};
    std::vector<std::vector<int>> sequence4 = {{1,2,3}};

	WaveGenerator sounds(sequence1);
	sounds.play_sounds();

    return 0;
}
