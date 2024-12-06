#include "communication_protocol/physical_layer.h"

PhysicalLayer::PhysicalLayer(){
}

PhysicalLayer::~PhysicalLayer(){

}


void PhysicalLayer::yell(std::vector<int> message){
    
    WaveGenerator sounds(message);
    sounds.play_sounds();
}

std::vector<int> PhysicalLayer::listen(bool hyperx){
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
    std::vector <int> ack = {14, 0 , 1, 1, 1, 1, 0, 14};
    try{
        AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
        audio_input.audio_open();
        audio_input.list_audio_devices();
        std::cout << "Recording audio..." << std::endl;
        audio_input.record_audio(INPUT_DEVICE, hyperx);
        audio_input.audio_close();

        if (audio_input.check(false, test_sequence2) == 0){
            yell(ack);
        };
        return audio_input.get_recorded_DTMF_tones();
    }
    catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
        return {-1};
    }
}