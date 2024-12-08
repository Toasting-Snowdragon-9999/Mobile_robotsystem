#include "communication_protocol/physical_layer.h"

PhysicalLayer::PhysicalLayer(){
}

PhysicalLayer::PhysicalLayer(int sample_rate, int input_device): _sample_rate(sample_rate), _device(input_device){
}


PhysicalLayer::~PhysicalLayer(){

}


void PhysicalLayer::yell(std::vector<int> message){
    
    WaveGenerator sounds(message, _sample_rate);
    sounds.play_sounds();
}

std::vector<int> PhysicalLayer::list(bool hyperx){
    if(hyperx){}
    AudioInput audio_input(_sample_rate, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    return {0};
}

std::vector<int> PhysicalLayer::listen(bool hyperx){
    try{
        AudioInput audio_input(_sample_rate, FRAMES_PER_BUFFER);
        audio_input.audio_open();
        audio_input.list_audio_devices();
        std::cout << "Recording audio..." << std::endl;
        audio_input.record_audio(_device, hyperx);
        audio_input.audio_close();
        return audio_input.get_recorded_DTMF_tones();
    }
    catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
        return {-1};
    }
}