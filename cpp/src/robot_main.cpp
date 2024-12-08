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
	Transport_Layer tlR;
	
	while (!tlR.get_combined_msg_flag())
	{
		PhysicalLayer pl(16000, 2);
		PhysicalLayer pl2(48000, 2);
<<<<<<< HEAD
		std::cout << "Listening.." << std::endl;
=======
>>>>>>> 3a472a7acc291108ac3e90a1b0aa05eba618acfc
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
<<<<<<< HEAD
			std::cout << "Ack string: " << ack << std::endl;

			std::vector<int> ack_dtmf = sp.convert_to_dtmf(ack);
			for(auto ayman : ack_dtmf){
				std::cout << "Ack tone: " << ayman << " $" << std::endl;
			}
=======

			std::vector<int> ack_dtmf = sp.convert_to_dtmf(ack);
>>>>>>> 3a472a7acc291108ac3e90a1b0aa05eba618acfc
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

				AlR.cpp_to_robot(final_final_commands_vec);
			}
			else
			{
				std::cout << "ERROR: LENGTH NOT FOUND, SEGMENTATION COUNT WRONG";
			}
		}
	}

	return 0;
}
