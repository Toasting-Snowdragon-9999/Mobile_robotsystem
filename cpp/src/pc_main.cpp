#include <iostream>
#include <fstream>
#include <bitset>
#include <algorithm>
#include <string>
#include <filesystem>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <thread>
// ===================================================
#include "read_shared_data.h"
#include "communication_protocol/crc.h"

// =======================Communication Layers============================
#include "communication_protocol/application_layer.h"
#include "communication_protocol/data_link_layer.h"
#include "communication_protocol/transport_layer.h"
#include "communication_protocol/physical_layer.h"
#include "communication_protocol/timer.h"

// =======================Interfaces============================
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"
#include "interfaces/signal_processing.h"

#define SAMPLING_FREQ 16000
#define DEVICE_SPEAKER 15
#define DEVICE_MIC 16

int main()
{
	// ======================================================
	// SENDER
	// ======================================================

	// Application Layer start	
	std::string path_gui = "../Docs/shared_file.json";
	std::string path_debug = "../../Docs/shared_file.json";

	SharedData shared_json(path_gui);	
	std::vector<std::vector<std::string>> python_path;
	try{
		python_path= shared_json.read_json();
	}
	catch(SharedDataException &e){
		if (e.error_code() == 21){
		}
		else{std::cout << "[Error] " << e.what() << std::endl;}
	}


	

	ApplicationLayer Alc;

	std::vector<robot_command> commands = Alc.python_to_cpp(python_path);
	// robot_command p1("-fw", "10");
	// robot_command p2("-r", "90");
	// robot_command p3("-fw", "40");
	// robot_command p4("-l", "45");
	// robot_command p5("-fw", "10");
	// // robot_command p6("-r", "90");

	// std::vector<robot_command> commands = {p1, p2, p3, p4, p5};

	Alc.print_robot_commands(commands);
	std::string test_bits = Alc.command_vector_to_bitstream(commands);

	std::string encoded_test_bits = Alc.encode_message(test_bits);

	// Application Layer end

	AlToTl inter_1;
	inter_1.add_string_to_buffer(encoded_test_bits);

	// Transport Layer start

	Transport_Layer tl;

	std::string tl_header_test_bits = tl.add_header(inter_1.get_buffer());

	auto segment_vector = tl.segment_msg(tl_header_test_bits);

	// Transport Layer end

	TlToDll inter_2;

	inter_2.add_segments_to_buffer(segment_vector);

	int i = 0;

	for (auto segment : segment_vector)
	{
		std::cout << "Segment" << i << " " << segment << std::endl;

		i++;
	}



	Timer timer;

	int j = 0;

	while (!inter_2.get_segment_buffer().empty())
	{
		std::cout << "How many segments left in buffer before iteration: " << inter_2.get_segment_buffer().size() << std::endl;
		DataLinkLayer dll(inter_2.get_first_segment_from_buffer());
		std::cout << "HERE COMES THE DATA LINK LAYER FRAME" << std::endl;
		std::string msg_to_send = dll.seq_protocol_structure();
		std::cout << "Receiver side get data from package: " << dll.receiver_side_get_data_from_package(msg_to_send) << std::endl;

		std::cout << "Sequence " << j << std::endl;
		j++;

		SignalProcessing sp;
		std::vector<int> dtmf_tone = sp.convert_to_dtmf(msg_to_send);
		PhysicalLayer pl_speaker(SAMPLING_FREQ, DEVICE_SPEAKER);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Avoid busy waiting

		for(auto v: dtmf_tone){
			std::cout << v << " ";
		}
		pl_speaker.yell(dtmf_tone);

		PhysicalLayer pl_mic(SAMPLING_FREQ, DEVICE_MIC);

		timer.start_timer(&pl_mic);

		while (!timer.get_timeout())
		{
			std::vector<int> ack_dtmf = pl_mic.listen(true);
			
			if(ack_dtmf.size() > 1){
				for (auto v : ack_dtmf){
					std::cout << v << " ";
				}
				SignalProcessing sp_ack(ack_dtmf);					// TODO: Change to not initialised with vector
				std::string binary_ack = sp_ack.message_str_binary();
				dll.sender_side_get_data_from_package(binary_ack);
				if (dll.get_ack_received()){
					std::cout << "Acknowledge received" << std::endl;
					inter_2.remove_first_segment_from_buffer();

					timer.~Timer();
					break;
				}
			}
			else{

			}
			
			std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Avoid busy waiting
		}
		timer.~Timer();
	}
	return 0;
	
}
