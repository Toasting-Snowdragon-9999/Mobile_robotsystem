// Standard libraries
#include <unistd.h>
#include <iostream>
#include <fstream>

// Own files
#include "move_turtlebot.cpp"  // Include header for movement control
#include "goertzel.h"
#include "audio_input.h"

// Definitions
#define SAMPLE_RATE (16000) // Value chosen because of microphone/drivers
#define FRAMES_PER_BUFFER (400)
#define INPUT_DEVICE (2) // Insert device #

int main(int argc, char *argv[]) {

    // --- Audio recording and signal processing ---
    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.save_to_textfile("../dtmf_sounds.txt");
    //audio_input.read_from_file("../dtmf_sounds.txt");
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

    rclcpp::shutdown();
    return 0;
}
