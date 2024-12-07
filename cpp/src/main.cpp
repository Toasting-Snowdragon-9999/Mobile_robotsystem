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
#include "interfaces/signal_processing.h"

int main()
{
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
	return 0;
}
