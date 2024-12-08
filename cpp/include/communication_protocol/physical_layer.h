#ifndef PHYSICALLAYER_H
#define PHYSICALLAYER_H

#include <vector>
#include <iostream>
#include "audio/wave_generator.h"
#include "audio/audio_input.h"
#define FRAMES_PER_BUFFER (480)

class PhysicalLayer{

    private:
        AudioInput _audio_input;
        int _sample_rate;
        int _device;

    public:
        PhysicalLayer();
        PhysicalLayer(int sample_rate, int device);
        ~PhysicalLayer();
        void yell(std::vector<int> message);
        std::vector<int> list(bool hyperx);
        std::vector<int> listen(bool hyperx);
        void stop_audio(bool stop);

};
#endif