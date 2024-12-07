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

	while (!inter_2.get_segment_buffer().empty())
	{
		std::cout << "How many segments left in buffer before iteration: " << inter_2.get_segment_buffer().size() << std::endl;
		DataLinkLayer dll(inter_2.get_first_segment_from_buffer());
		std::string msg_to_send = dll.protocol_structure();

		// Alt andet for at sende til Physical Layer
		// Alt andet for at sende til Physical Layer
		// Alt andet for at sende til Physical Layer

		timer.start_timer();

		while (!timer.get_timeout())
		{
			if (dll.is_ack_received())
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

	return 0;
}
