#include <iostream>
#include <fstream>
#include <bitset>
#include <algorithm>
#include <string>
#include <filesystem>
#include <string>
#include <vector>

// ===================================================
#include "read_shared_data.h"
// #include "audio/wave_generator.h"

// =======================Communication Layers============================
#include "communication_protocol/application_layer.h"
#include "communication_protocol/data_link_layer.h"
#include "communication_protocol/transport_layer.h"
#include "communication_protocol/physical_layer.h"
#include "communication_protocol/crc.h"

// =======================Inerfaces============================
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"
#include "interfaces/signal_processing.h"
#define LENGTH_FACTOR 10

int main(){

	std::string ack = "1";
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
	std::vector<std::vector<std::string>>test = {{"-fw","2"}};

	ApplicationLayer app_layer;
	std::vector<robot_command> commands = app_layer.python_to_cpp(test);
	app_layer.print_robot_commands(commands);
	std::string test_bits = app_layer.command_vector_to_bitstream(commands);

	std::string encoded_test_bits = app_layer.encode_message(test_bits);

	AlToTl i_al_tl;

	i_al_tl.add_string_to_buffer(encoded_test_bits);

	Transport_Layer tl;

	std::string tl_header_test_bits = tl.add_header(i_al_tl.get_buffer());

	auto segment_vector = tl.segment_msg(tl_header_test_bits);

	TlToDll i_tl_dll;

	i_tl_dll.add_segments_to_buffer(segment_vector);

	while(!i_tl_dll.is_buffer_empty()){
		
		std::string segment = i_tl_dll.take_segment_from_buffer();

		DataLinkLayer dll(segment);

		dll.protocol_structure();

		DllToPl i_dl_pl;

		i_dl_pl.add_ready_msg(dll.get_ready_for_pl_path());

		std::string msg_to_send = i_dl_pl.get_ready_msg();

		std::cout << "Message to send: " << msg_to_send << std::endl;

		SignalProcessing sp;
		std::vector<int> dtmf = sp.convert_to_dtmf(msg_to_send);
		for(auto v : dtmf){
			std::cout << v << " ";
		}
		PhysicalLayer pl(16000, 7);
		pl.yell(dtmf);
		
		PhysicalLayer pl2(16000, 18);
		std::vector <int> samples = pl2.listen(true);
		SignalProcessing sp_ack(samples);
		std::string binary_msg = sp_ack.message_str_binary();
		DataLinkLayer dl_layer;
		std::string package = dl_layer.get_data_from_package(binary_msg);

		if(package == ack){	
			std::cout << "Acknowledge received" << std::endl;
			i_tl_dll.remove_segment_from_buffer();
		}

	}
	return 0;
}


