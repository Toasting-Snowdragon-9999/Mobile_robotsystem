// Standard libraries
#include <unistd.h>
#include <iostream>
#include <fstream>

// Own files
#include "move_turtlebot.cpp"  // Include header for movement control
#include "goertzel.h"
#include "audio_input.h"
#include "wave_generator.h"

// Definitions
#define SAMPLE_RATE (16000) // Value chosen because of microphone/drivers
#define FRAMES_PER_BUFFER (480)
#define INPUT_DEVICE (2) // Insert device #

int main(int argc, char *argv[]) {

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

    std::vector<int> ja = {14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 0};

    // -- Play sounds --
    //WaveGenerator sounds(test_sequence2);
    //sounds.play_sounds();


    // --- Audio recording and signal processing ---
    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    audio_input.record_audio(INPUT_DEVICE);
    //audio_input.save_to_wav("../dtmf_sounds/output.wav");
    //audio_input.save_to_textfile("../dtmf_sounds.txt");
    //audio_input.check(true, test_sequence2);
    audio_input.audio_close();

    // --- Turtlebot control ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtlebot>();

    // 12: forward -> {"-fw", "distance"}
    // 13: backward -> {"-bw", "distance"}
    // 14: right -> {"-r", "degree"}
    // 15: left -> {"-l", "degree"}

    /*std::vector<std::vector<int>> sequence = {{12, 2, 0}, {13, 2, 0}, {14, 9, 0}, {15, 9, 0}};
    std::vector<std::vector<std::string>> table_sequence = {{"-fw", "40"}, {"-l", "90"}, {"-l", "45"}, {"-fw", "25"}, {"-r", "90"}, {"-fw", "30"}};

    node->run_path(table_sequence);*/

    // --- Play sounds ---
    if(!audio_input.check(true, ja)){
        WaveGenerator sounds(ja);
        sounds.play_sounds();
    }

    rclcpp::shutdown();
    return 0;
}
