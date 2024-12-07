// Standard libraries
#include <unistd.h>
#include <iostream>
#include <fstream>

// Own files
#include "move_turtlebot.cpp"  // Include header for movement control
#include "algorithms/goertzel.h"
#include "audio/audio_input.h"
#include "audio/wave_generator.h"

#include "communication_protocol/application_layer.h"
#include "communication_protocol/data_link_layer.h"
#include "communication_protocol/transport_layer.h"
#include "communication_protocol/physical_layer.h"

#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"
#include "interfaces/signal_processing.h"

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
    std::vector<int> tihi =
        {14, 0, 15, 15, 4, 1, 14, 15, 15, 5, 8, 15, 15,
        5, 1, 9, 2, 3, 5, 10, 1, 5, 12, 0, 10, 14, 0};

    robot_command r1("-fw", "325");
	robot_command r2("-l", "6");
	robot_command r3("-r", "21");
	robot_command r4("-bw", "15");
	robot_command r5("-r", "3");

	std::vector<robot_command> test_bit_vec = {r1, r2, r3, r4, r5};

    // -- Play sounds --
    //WaveGenerator sounds(test_sequence2);
    //sounds.play_sounds();

/*

    // --- Audio recording and signal processing ---
    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    audio_input.record_audio(INPUT_DEVICE, false);
    //audio_input.save_to_wav("../dtmf_sounds/output.wav");
    //audio_input.save_to_textfile("../dtmf_sounds.txt");
    //audio_input.check(true, test_sequence2);
    audio_input.audio_close();

*/

    std::vector<int> ack = {14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 0};
	PhysicalLayer pl;
	std::vector<int> dtmf_sounds = pl.listen(false);

	SignalProcessing sp(dtmf_sounds);
	std::string binary_msg = sp.message_str_binary();

	DataLinkLayer dl_layer(binary_msg);
	std::string package = dl_layer.get_data_from_package();
/*
	TlToDll i_tl;
	i_tl.add_segment_to_buffer(package);
	std::string segment = i_tl.take_segment_from_buffer();

	AlToTl i_al;
	i_al.add_string_to_buffer(segment);
	std::string buffer = i_al.get_buffer();
*/
std::cout << "Package: \n" << package << std::endl;
	ApplicationLayer app_layer;
	std::vector<robot_command> commands = app_layer.bits_to_commands(package);


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
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(auto i = 0; i < test_bit_vec.size(); i++){
        std::cout << i << + " Sent: " << test_bit_vec[i].direction << ", " << test_bit_vec[i].value << std::endl;

    }
    std::cout << commands.size() << std::endl;
    for(auto i = 0; i < commands.size(); i++){
        std::cout << i << " Received: " << commands[i].direction << ", " << commands[i].value << std::endl;
    }

    bool ack_check = false;
/*
    if(test_bit_vec.size() == commands.size()){
        for(auto i = 0; i < commands.size(); i++){
            if((test_bit_vec[i].direction == commands[i].direction) && (test_bit_vec[i].value == commands[i].value)){
                ack_check = true;
            } else{
                ack_check = false;
                break;
            }
        }
    } else{
        std::cout << "Not the same size" << std::endl;
    }
*/
    if(!ack_check){
        pl.yell(ack);
    }
    rclcpp::shutdown();
    return 0;
}
