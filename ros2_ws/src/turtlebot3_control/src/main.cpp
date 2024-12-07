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
#define LENGTH_FACTOR 10

int main(int argc, char *argv[]) {

	// --- R
	robot_command p1("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p2("-r", "90");
	robot_command p3("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p4("-r", "90");
	robot_command p5("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p6("-r", "90");
	robot_command p7("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p8("-r", "45");
	robot_command p9("-fw", std::to_string(2.8*LENGTH_FACTOR));
	// ---
	robot_command p10("-l", "45");
	robot_command p11("-bw", std::to_string(1*LENGTH_FACTOR));
	robot_command p12("-fw", "90");
	// --- O
	robot_command p13("-bw", std::to_string(4*LENGTH_FACTOR));
	robot_command p14("-r", "90");
	robot_command p15("-bw", std::to_string(2*LENGTH_FACTOR));
	robot_command p16("-r", "90");
	robot_command p17("-bw", std::to_string(4*LENGTH_FACTOR));
	robot_command p18("-r", "90"); 
	robot_command p19("-bw", std::to_string(2*LENGTH_FACTOR));
	// ---
	robot_command p20("-fw", std::to_string(3*LENGTH_FACTOR));
	robot_command p21("-l","90");
	// --- B
	robot_command p22("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p23("-r", "90");
	robot_command p24("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p25("-r", "90");
	robot_command p26("-fw", std::to_string(1*LENGTH_FACTOR));
	robot_command p27("-r", "45");
	robot_command p28("-fw", std::to_string(1.4*LENGTH_FACTOR));
	robot_command p29("-r", "45");
	robot_command p30("-fw", std::to_string(1*LENGTH_FACTOR));
	robot_command p31("-bw", std::to_string(1*LENGTH_FACTOR));
	robot_command p32("-r","45");
	robot_command p33("-bw", std::to_string(1.4*LENGTH_FACTOR));
	robot_command p34("-r", "45");
	robot_command p35("-bw", std::to_string(1*LENGTH_FACTOR));
	robot_command p36("-r", "90");
	robot_command p37("-bw", std::to_string(2*LENGTH_FACTOR));
	// ---
	robot_command p38("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p39("-l", "90");
	// --- T
	robot_command p40("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p41("-l", "90");
	robot_command p42("-fw", std::to_string(1*LENGTH_FACTOR));
	robot_command p43("-bw", std::to_string(2*LENGTH_FACTOR));
	// ---
	robot_command p44("-bw", std::to_string(3*LENGTH_FACTOR));
	// --- E
	robot_command p45("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p46("-l", "90");
	robot_command p47("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p48("-l","90");
	robot_command p49("-fw", std::to_string(1*LENGTH_FACTOR));
	robot_command p50("-bw", std::to_string(1*LENGTH_FACTOR));
	robot_command p51("-l", "90");
	robot_command p52("-bw", std::to_string(2*LENGTH_FACTOR));
	// ---
	robot_command p53("-bw", std::to_string(1*LENGTH_FACTOR));
	robot_command p54("-r", "90");
	// --- K
	robot_command p55("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p56("-bw", std::to_string(2*LENGTH_FACTOR));
	robot_command p57("-r", "45");
	robot_command p58("-fw", std::to_string(2.8*LENGTH_FACTOR));
	robot_command p59("-bw", std::to_string(2.8*LENGTH_FACTOR));
	robot_command p60("-r", "90");
	robot_command p61("-fw", std::to_string(2.8*LENGTH_FACTOR));
	
	std::vector<robot_command> robtek = {
		p1,  p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9,  p10,
		p11, p12, p13, p14, p15, p16, p17, p18, p19, p20,
		p21, p22, p23, p24, p25, p26, p27, p28, p29, p30,
		p31, p32, p33, p34, p35, p36, p37, p38, p39, p40,
		p41, p42, p43, p44, p45, p46, p47, p48, p49, p50,
		p51, p52, p53, p54, p55, p56, p57, p58, p59, p60,
		p61
	};
/*
        robot_command r1("-fw", "325");
	robot_command r2("-l", "6");
	robot_command r3("-r", "21");
	robot_command r4("-bw", "15");
	robot_command r5("-r", "3");

	std::vector<robot_command> test_bit_vec = {r1, r2, r3, r4, r5};

        std::vector<int> ack = {14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 0};
	PhysicalLayer pl(SAMPLE_RATE, INPUT_DEVICE);
	std::vector<int> dtmf_sounds = pl.listen(false);
	
	SignalProcessing sp(dtmf_sounds);
	std::string binary_msg = sp.message_str_binary();
	
	DataLinkLayer dl_layer;
	std::string package = dl_layer.get_data_from_package(binary_msg);

	
	std::vector<robot_command> commands;
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
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	    commands = app_layer.bits_to_commands(final_package);
			pl.yell(ack);
		}
	}

*/
    // --- Turtlebot control ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtlebot>();

    // 12: forward -> {"-fw", "distance"}
    // 13: backward -> {"-bw", "distance"}
    // 14: right -> {"-r", "degree"}
    // 15: left -> {"-l", "degree"}

    /*std::vector<std::vector<int>> sequence = {{12, 2, 0}, {13, 2, 0}, {14, 9, 0}, {15, 9, 0}};
    std::vector<std::vector<std::string>> table_sequence = {{"-fw", "40"}, {"-l", "90"}, {"-l", "45"}, {"-fw", "25"}, {"-r", "90"}, {"-fw", "30"}};
*/
    node->run_path(robtek);

    // --- Print commands ---
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
/*
    for(auto i = 0; i < test_bit_vec.size(); i++){
        std::cout << i << + " Sent: " << test_bit_vec[i].direction << ", " << test_bit_vec[i].value << std::endl;

   }
    std::cout << commands.size() << std::endl;
    for(auto i = 0; i < commands.size(); i++){
        std::cout << i << " Received: " << commands[i].direction << ", " << commands[i].value << std::endl;
    }

*/
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
