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

// =======================Interfaces============================
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"
#include "communication_protocol/timer.h"
#include "interfaces/signal_processing.h"

int main()
{
	TlToDll i_tl;
	std::string ack_commands = "1";
	for(int i = 0; i < 2; i++){
		PhysicalLayer pl(16000, 2);
		std::vector<int> dtmf_sounds = pl.listen(false);

		SignalProcessing sp(dtmf_sounds);
		std::string binary_msg = sp.message_str_binary();
		
		DataLinkLayer dl_layer;
		std::string package = dl_layer.get_data_from_package(binary_msg);

		if(!package.empty()){
			PhysicalLayer transmitter(48000, 2); 
			i_tl.add_segment_to_buffer(package);
		
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			DataLinkLayer dll_ack(ack_commands);
			dll_ack.protocol_structure();
			std::string message_ready = dll_ack.get_ready_for_pl_path();
			SignalProcessing sp;
			std::vector<int> dtmf_ack = sp.convert_to_dtmf(message_ready);
			transmitter.yell(dtmf_ack);
		}
	}
	Transport_Layer tp_layer;

	// ======================================================
	// SENDER
	// ======================================================

	robot_command r1("-fw", "325");
	robot_command r2("-l", "6");
	robot_command r3("-r", "21");
	robot_command r4("-bw", "15");
	robot_command r5("-r", "3");

	std::vector<robot_command> test_bit_vec = {r1, r2, r3, r4, r5};

	ApplicationLayer Alc;

	std::string test_bits = Alc.command_vector_to_bitstream(test_bit_vec);

	std::string encoded_test_bits = Alc.encode_message(test_bits);

	std::cout << "Test bits length" << encoded_test_bits.length() << std::endl;

	Transport_Layer testertl;

	std::cout << "Tester length: " << testertl.find_length(encoded_test_bits) << std::endl;

	// Interface from Application Layer to Transport Layer

	AlToTl inter_1;

	inter_1.add_string_to_buffer(encoded_test_bits);

	Transport_Layer tl;

	std::string tl_header_test_bits = tl.add_header(inter_1.get_buffer());

	auto segment_vector = tl.segment_msg(tl_header_test_bits);

	// Interface from Transport Layer to Data Link Layer

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

	std::string msg_to_send = "";

	// while (!inter_2.get_segment_buffer().empty())
	// {
	// 	std::cout << "How many segments left in buffer before iteration: " << inter_2.get_segment_buffer().size() << std::endl;
	// 	DataLinkLayer dll(inter_2.get_first_segment_from_buffer());
	// 	msg_to_send = dll.seq_protocol_structure();
	// LISTEN FOR NEXT SEGMENT
	// LISTEN FOR NEXT SEGMENT
	// LISTEN FOR NEXT SEGMENT
	// 	std::cout << "Sequence " << j << std::endl;
	// 	j++;

	// 	// Alt andet for at sende til Physical Layer
	// 	// Alt andet for at sende til Physical Layer
	// 	// Alt andet for at sende til Physical Layer

	// 	timer.start_timer();

	// 	while (!timer.get_timeout())
	// 	{
	// 		// LISTENING PHYSICAL LAYER FOR ACK
	// 		// LISTENING PHYSICAL LAYER FOR ACK
	// 		// LISTENING PHYSICAL LAYER FOR ACK

	// 		dll.set_ack_received(true);

	// 		if (dll.get_ack_received())
	// 		{
	// 			inter_2.remove_first_segment_from_buffer();

	// 			timer.~Timer();
	// 			break;
	// 		}
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Avoid busy waiting
	// 	}
	// 	timer.~Timer();
	// }

	int k = 0;

	Transport_Layer tlR;

	for (const auto &segment : segment_vector)
	{
		DataLinkLayer dlls1(segment); // Initialize DataLinkLayer with the current segment
		// Creating empty object to send ACK later on
		DataLinkLayer dllack;

		std::cout << "Test 0 " << std::endl;
		std::string package_to_send = dlls1.seq_protocol_structure();
		std::cout << "Package to send is: " << package_to_send << std::endl;

		std::cout << "Test 1 " << std::endl;

		DataLinkLayer dllr1(package_to_send);
		std::string received_package = dllr1.receiver_side_get_data_from_package(package_to_send);

		std::cout << "Test 2 " << std::endl;

		// Send ACK to Computer again if received msg is correct
		if (dllr1.get_is_msg_correct())
		{
			std::string ack = dllack.ack_protocol_structure();

			std::cout << "Test 2.1 " << std::endl;

			dllack.change_ack_indx_sender_sider();
			std::cout << "Test 2.2 " << std::endl;

			// Change ACKno on sender side

			// YELL(ACK)
			// YELL(ACK)
			// YELL(ACK)
			std::cout << "Test 3 " << std::endl;
		}

		// If received msg was a duplicate, the empty() will catch it here
		if (!received_package.empty())
		{
			TlToDll inter_dll_tl;
			std::cout << "Test 4 " << std::endl;

			inter_dll_tl.add_segment_to_buffer(received_package);
			tlR.add_segment(inter_dll_tl.take_segment_from_buffer());

			std::cout << "Test 5 " << std::endl;

			std::cout << "Printed segment vector: " << std::endl;

			std::string combined_string = tlR.combine_segments_to_string();

			std::cout << "Test 5.1 " << std::endl;

			std::cout << "Temporarily combined string in Transport Layer: " << combined_string << "	| Length of temporary combined string: " << tlR.get_length_from_header(combined_string) << std::endl;

			if (tlR.is_combined_msg_complete(combined_string))
			{
				std::cout << "Test 5.1.2 " << std::endl;

				inter_dll_tl.set_all_segments_received(true);
			}

			if (!inter_dll_tl.get_all_segments_received())
			{
				std::cout << "Test 5.2 " << std::endl;

				// LISTEN FOR NEXT SEGMENT
				// LISTEN FOR NEXT SEGMENT
				// LISTEN FOR NEXT SEGMENT
			}
			else if (inter_dll_tl.get_all_segments_received())
			{

				combined_string = tlR.remove_header_and_unstuff(combined_string);

				AlToTl altlr;

				std::cout << "Combined string " << combined_string << std::endl;

				altlr.add_string_to_buffer(combined_string);
				std::cout << "Test 6 " << std::endl;

				std::string msg_for_robot = altlr.get_buffer();

				std::cout << "Test 6.1 " << std::endl;

				ApplicationLayer AlR;

				if (AlR.is_msg_correct(msg_for_robot))
				{
					std::cout << "Test 6.2 " << std::endl;

					AlR.remove_msg_crc(msg_for_robot);
					std::cout << "Test 6.3 " << std::endl;
				}
				else
				{
					std::cerr << "Message received is wrong, CRC-check failed" << std::endl;
				}

				std::cout << "msg_for_robot: " << msg_for_robot << std::endl;

				auto final_final_commands_vec = AlR.bits_to_commands(msg_for_robot);

				std::cout << "Dir 0 " << final_final_commands_vec[0].direction << std::endl;
				std::cout << "Val 0 " << final_final_commands_vec[0].value << std::endl;

				std::cout << "Test 7 " << std::endl;

				AlR.print_robot_commands(final_final_commands_vec);
			}
			else
			{
				std::cout << "ERROR: LENGTH NOT FOUND, SEGMENTATION COUNT WRONG";
			}
		}
	}

	std::cout << "Everything is hopefully above " << std::endl;


	std::vector<std::vector<std::string>> commands = app_layer.cpp_to_robot(final_final_commands_vec);
	for(auto command : commands){
		std::cout << "Command: " << command[0] << "   Value: " << command[1] << std::endl;
	}

	return 0;
}
