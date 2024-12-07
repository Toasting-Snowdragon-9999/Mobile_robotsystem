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

    robot_command r1("-fw", "325");
	robot_command r2("-l", "6");
	robot_command r3("-r", "21");
	robot_command r4("-bw", "15");
	robot_command r5("-r", "3");

	std::vector<robot_command> test_bit_vec = {r1, r2, r3, r4, r5};

    std::vector<int> ack = {14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 0};
	PhysicalLayer pl;
	std::vector<int> dtmf_sounds = pl.listen(false);
	
	SignalProcessing sp(dtmf_sounds);
	std::string binary_msg = sp.message_str_binary();
	
	DataLinkLayer dl_layer;
	std::string package = dl_layer.get_data_from_package(binary_msg);

    if(!package.empty()){
		TlToDll i_tl;
		i_tl.add_segment_to_buffer(package);
		std::string segment = i_tl.take_segment_from_buffer();

		AlToTl i_al;
		i_al.add_string_to_buffer(segment);
		std::string buffer = i_al.get_buffer();	

		Transport_Layer tp_layer;
		std::string package_no_header = tp_layer.remove_header_and_unstuff(buffer);
		
		ApplicationLayer app_layer;
		std::string final_package = app_layer.check_crc(package_no_header);
		if(final_package.empty()){
			std::cerr << "CRC check failed" << std::endl;
		}
		else{
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			std::vector<robot_command> comd2 = app_layer.bits_to_commands(final_package);
			pl.yell(ack);
		}
	}


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

    rclcpp::shutdown();
    return 0;
}
