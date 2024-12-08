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

#define SAMPLING_FREQ 16000
#define DEVICE_SPEAKER 7
#define DEVICE_MIC 18

int main()
{

	// ======================================================
	// SENDER
	// ======================================================

	// Application Layer start

	robot_command r1("-fw", "325");
	robot_command r2("-l", "6");
	robot_command r3("-r", "21");
	robot_command r4("-bw", "15");
	robot_command r5("-r", "3");

	std::vector<robot_command> test_bit_vec = {r1, r2, r3, r4, r5};

	ApplicationLayer Alc;

	std::string test_bits = Alc.command_vector_to_bitstream(test_bit_vec);

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
		std::string msg_to_send = dll.seq_protocol_structure();

		std::cout << "Sequence " << j << std::endl;
		j++;

		SignalProcessing sp;
		std::vector<int> dtmf_tone = sp.convert_to_dtmf(msg_to_send);
		PhysicalLayer pl_speaker(SAMPLING_FREQ, DEVICE_SPEAKER);
		pl_speaker.yell(dtmf_tone);

		PhysicalLayer pl_mic(SAMPLING_FREQ, DEVICE_SPEAKER);

		timer.start_timer();

		while (!timer.get_timeout())
		{
			std::vector<int> ack_dtmf = pl_mic.listen(true);
			SignalProcessing sp_ack(ack_dtmf);					// TODO: Change to not initialised with vector
			std::string binary_ack = sp_ack.message_str_binary();
			dll.sender_side_get_data_from_package(binary_ack);
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
	return 0;
	
}
