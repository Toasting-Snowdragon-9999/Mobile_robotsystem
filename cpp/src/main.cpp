#include <iostream>
#include <fstream>
#include <bitset>
#include <algorithm>
#include <string>
#include "crc.h"
#include <filesystem>
#include <string>
#include <vector>
#include "read_shared_data.h"
#include "audio/wave_generator.h"
#include "communication_protocol/application_layer.h"
#include "communication_protocol/data_link_layer.h"
#include "communication_protocol/transport_layer.h"
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"
#include "communication_protocol/timer.h"

int main()
{

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

	// Interface from Application Layer to Transport Layer

	AlToTl inter_1;

	inter_1.add_string_to_buffer(encoded_test_bits);

	Transport_Layer tl;

	std::string tl_header_test_bits = tl.add_header(inter_1.get_buffer());

	auto segment_vector = tl.segment_msg(tl_header_test_bits);

	// Interface from Transport Layer to Data Link Layer

	TlToDll inter_2;

	inter_2.add_segments_to_buffer(segment_vector);

	Timer timer;

	int j = 0;

	std::string msg_to_send = "";

	while (!inter_2.get_segment_buffer().empty())
	{
		std::cout << "How many segments left in buffer before iteration: " << inter_2.get_segment_buffer().size() << std::endl;
		DataLinkLayer dll(inter_2.get_first_segment_from_buffer());
		msg_to_send = dll.seq_protocol_structure();

		std::cout << "Sequence " << j << std::endl;
		j++;

		// Alt andet for at sende til Physical Layer
		// Alt andet for at sende til Physical Layer
		// Alt andet for at sende til Physical Layer

		timer.start_timer();

		while (!timer.get_timeout())
		{
			// LISTENING PHYSICAL LAYER FOR ACK
			// LISTENING PHYSICAL LAYER FOR ACK
			// LISTENING PHYSICAL LAYER FOR ACK

			dll.set_ack_received(true);

			if (dll.get_ack_received())
			{
				inter_2.remove_first_segment_from_buffer();

				timer.~Timer();
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Avoid busy waiting
		}
		timer.~Timer();
	}

	// Interface from Data Link Layer to Physical Layer

	// ======================================================
	// SENDER
	// ======================================================

	// ======================================================
	// Receiver
	// ======================================================

	// DataLinkLayer dllr(msg_to_send);

	// std::string tr_with = dllr.ack_protocol_structure();

	// std::string received_frame = dllr.get_data_from_package(tr_with);
	// std::cout << "Received frame: " << received_frame << std::endl;

	DataLinkLayer dlls1(segment_vector[0]);
	std::string package_to_send = dlls1.seq_protocol_structure();
	std::cout << "Package to send is: " << package_to_send << std::endl;

	DataLinkLayer dllr1(package_to_send);
	std::string received_package = dllr1.get_data_from_package(package_to_send);
	std::cout << "Is message correct: " << dllr1.get_is_msg_correct() << std::endl;
	// Send ACK to Computer again if received msg is correct
	if(dllr1.get_is_msg_correct()){
	}

	std::cout << "Received package is: " << received_package << std::endl;

	return 0;
}
