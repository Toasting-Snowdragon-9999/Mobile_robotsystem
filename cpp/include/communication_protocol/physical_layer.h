#ifndef PHYSICALLAYER_H
#define PHYSICALLAYER_H

#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include "audio/wave_generator.h"
#include "audio/audio_input.h"
#define FRAMES_PER_BUFFER (480)

class PhysicalLayer{

    private:
        AudioInput _audio_input;
        int _sample_rate;
        int _device;

    public:
        /**
         * @brief Construct a new Physical Layer object
         */
        PhysicalLayer();
        /**
         * @brief Construct a new Physical Layer object
         * @param sample_rate The sample rate of the signal
         * @param device The device to use
         */
        PhysicalLayer(int sample_rate, int device);
        /**
         * @brief Destroy the Physical Layer object
         */
        ~PhysicalLayer();
        /**
         * @brief This function is used to yell / play out a given dtmf sequence
         */
        void yell(std::vector<int> message);
        /**
         * @brief This function is used to list the available audio devices
         * @param hyperx not used
         */
        std::vector<int> list(bool hyperx);
        /**
         * @brief This function is used to capturing the a dtmf sequence
         * @param hyperx not used
         */
        std::vector<int> listen(bool hyperx);
        /**
         * @brief This function is used to stop the audio, from the outside
         * @param stop The flag to indicate if the audio should stop
         */
        void stop_audio(bool stop);

};
#endif