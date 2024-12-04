#ifndef PHYSICALLAYER_H
#define PHYSICALLAYER_H

#include <vector>
#include <iostream>
#include "audio/wave_generator.h"
#include "audio/audio_input.h"
#define SAMPLE_RATE (16000)
#define FRAMES_PER_BUFFER (480)
#define INPUT_DEVICE (13) 

class PhysicalLayer{

    private:

    public:
        PhysicalLayer();
        ~PhysicalLayer();
        void yell(std::vector<int> message);
        std::vector<int> listen(int timer_ms = 0);


};
#endif