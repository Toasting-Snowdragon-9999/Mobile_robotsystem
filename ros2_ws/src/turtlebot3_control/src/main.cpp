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
/*
	// --- R
	robot_command p1("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p2("-r", "90");
	robot_command p3("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p4("-r", "90");
	robot_command p5("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p6("-r", "90");
	robot_command p7("-fw", std::to_string(2*LENGTH_FACTOR));
	robot_command p8("-r", "45");
	robot_command p9("-bw", std::to_string(2.8*LENGTH_FACTOR));
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
	robot_command p53("-l", "90");
	robot_command p54("-bw", std::to_string(2*LENGTH_FACTOR));
	// ---
	robot_command p55("-bw", std::to_string(1*LENGTH_FACTOR));
	robot_command p56("-r", "90");
	// --- K
	robot_command p57("-fw", std::to_string(4*LENGTH_FACTOR));
	robot_command p58("-bw", std::to_string(2*LENGTH_FACTOR));
	robot_command p59("-r", "45");
	robot_command p60("-fw", std::to_string(2.8*LENGTH_FACTOR));
	robot_command p61("-bw", std::to_string(2.8*LENGTH_FACTOR));
	robot_command p62("-r", "90");
	robot_command p63("-fw", std::to_string(2.8*LENGTH_FACTOR));
	
	std::vector<robot_command> r = {p1, p2, p3, p4, p5, p6, p7, p8 ,p9};
	std::vector<robot_command> o = {p13, p14, p15, p16, p17, p18, p19};
	std::vector<robot_command> b = {p22, p23, p24, p25, p26, p27, p28, p29 ,p30, p31, p32, p33, p34, p35, p36, p37};
	std::vector<robot_command> t = {p40, p41, p42, p43};
	std::vector<robot_command> e = {p45, p46, p47, p48, p49, p50, p51, p52 ,p53, p54};
	std::vector<robot_command> k = {p57, p58, p59, p60, p61, p62, p63};
	
	std::vector<robot_command> robtek = {
		p1,  p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9,  p10,
		p11, p12, p13, p14, p15, p16, p17, p18, p19, p20,
		p21, p22, p23, p24, p25, p26, p27, p28, p29, p30,
		p31, p32, p33, p34, p35, p36, p37, p38, p39, p40,
		p41, p42, p43, p44, p45, p46, p47, p48, p49, p50,
		p51, p52, p53, p54, p55, p56, p57, p58, p59, p60,
		p61
	};
*/

	Transport_Layer tlR;

	std::vector<std::vector<std::string>> turtlebot_move_ass;

	while (!tlR.get_combined_msg_flag())
	{
		PhysicalLayer pl(16000, 2);
		PhysicalLayer pl2(48000, 2);
		std::cout << "Listening.." << std::endl;
		Timer timer;
		timer.start_timer(&pl);
		std::vector<int> dtmf_sounds = pl.listen(false);

		SignalProcessing sp(dtmf_sounds);
		std::string binary_msg = sp.message_str_binary();

		DataLinkLayer dllR; // Initialize DataLinkLayer with the current segment
		// Creating empty object to send ACK later on
		DataLinkLayer dllack;

		std::string received_package = dllR.receiver_side_get_data_from_package(binary_msg);

		// Send ACK to Computer again if received msg is correct
		if (dllR.get_is_msg_correct())
		{
			std::string ack = dllack.ack_protocol_structure();
			std::cout << "Ack string: " << ack << std::endl;

			std::vector<int> ack_dtmf = sp.convert_to_dtmf(ack);
			for(auto ayman : ack_dtmf){
				std::cout << "Ack tone: " << ayman << " $" << std::endl;
			}

			std::vector<int> ack_dtmf = sp.convert_to_dtmf(ack);

			std::vector<int> ack_dtmf = sp.convert_to_dtmf(ack);
			pl2.yell(ack_dtmf);
		}

		// If received msg was a duplicate, the empty() will catch it here
		if (!received_package.empty())
		{
			TlToDll inter_dll_tl;

			inter_dll_tl.add_segment_to_buffer(received_package);
			tlR.add_segment(inter_dll_tl.take_segment_from_buffer());

			std::string combined_string = tlR.combine_segments_to_string();

			if (tlR.is_combined_msg_complete(combined_string))
			{
				inter_dll_tl.set_all_segments_received(true);
			}

			if (!inter_dll_tl.get_all_segments_received())
			{

			}
			else if (inter_dll_tl.get_all_segments_received())
			{

				combined_string = tlR.remove_header_and_unstuff(combined_string);

				AlToTl altlr;

				altlr.add_string_to_buffer(combined_string);

				std::string msg_for_robot = altlr.get_buffer();

				ApplicationLayer AlR;

				if (AlR.is_msg_correct(msg_for_robot))
				{
					msg_for_robot = AlR.remove_msg_crc(msg_for_robot);
				}
				else
				{
					std::cerr << "Message received is wrong, CRC-check failed" << std::endl;
				}

				auto final_final_commands_vec = AlR.bits_to_commands(msg_for_robot);

				AlR.print_robot_commands(final_final_commands_vec);

				turtlebot_move_ass = AlR.cpp_to_robot(final_final_commands_vec);
			}
			else
			{
				std::cout << "ERROR: LENGTH NOT FOUND, SEGMENTATION COUNT WRONG";
			}
		}
	}

    // --- Turtlebot control ---
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveTurtlebot>();

    // 12: forward -> {"-fw", "distance"}
    // 13: backward -> {"-bw", "distance"}
    // 14: right -> {"-r", "degree"}
    // 15: left -> {"-l", "degree"}

    node->run_path(turtlebot_move_ass);

    // --- Print commands ---
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    rclcpp::shutdown();
    return 0;
}
