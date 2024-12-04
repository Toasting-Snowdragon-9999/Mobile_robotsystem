#include "audio/physical_layer.h"

PhysicalLayer::PhysicalLayer(){
}

PhysicalLayer::~PhysicalLayer(){

}


void PhysicalLayer::yell(std::vector<int> message){
    WaveGenerator sounds(message);
    sounds.play_sounds();
}

std::vector<int> PhysicalLayer::listen(int timer_ms){
    try{
        AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER, timer_ms);
        audio_input.audio_open();
        audio_input.list_audio_devices();
        // return {-1};
        audio_input.record_audio(INPUT_DEVICE);
        audio_input.audio_close();
        return audio_input.get_recorded_DTMF_tones();
    }
    catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
        return {-1};
    }
}