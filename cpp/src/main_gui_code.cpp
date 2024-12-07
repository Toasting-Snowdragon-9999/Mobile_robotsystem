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

int main(){
	/*
	std::string path_gui = "../Docs/shared_file.json";
	std::string path_debug = "../../Docs/shared_file.json";
	SharedData sd(path_debug);

	try{
		sd.read_json();
		sd.print();
	}
	catch(SharedDataException &e){
		if (e.error_code() == 21){
		}
		else{std::cout << "[Error] " << e.what() << std::endl;}
	}

	std::vector<int> test_sequence2 = // 100 numbers
		{14, 0, 5, 3, 5, 7, 11, 2, 8, 1,
		6, 4, 10, 9, 13, 12, 15, 14, 7,
		6, 3, 8, 5, 2, 10, 11, 9, 0, 12,
		4, 13, 1, 15, 14, 0, 2, 5, 8, 11,
		10, 9, 0, 13, 12, 3, 4, 15, 1, 7,
		8, 7, 5, 10, 6, 3, 11, 14, 2, 9,
		4, 13, 7, 0, 8, 12, 6, 15, 1, 2,
		10, 11, 9, 3, 5, 13, 7, 8, 6, 12,
		4, 11, 0, 1, 14, 7, 5, 3, 8, 10,
		1, 2, 11, 13, 12, 9, 6, 15, 0, 14, 0};

	std::vector<int> test = {14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 0};
	*/
	std::string aymans_tal = "325";
	robot_command r1("-fw", aymans_tal);
	robot_command r2("-l", "6");
	robot_command r3("-r", "21");
	robot_command r4("-bw", "15");
	robot_command r5("-r", "3");

	std::vector<robot_command> test_bit_vec = {r1, r2, r3, r4, r5};

	ApplicationLayer Alc;

	// FYI *There exists both command_to_bits and command_vector_to_bitstream*

	std::string test_bits = Alc.command_vector_to_bitstream(test_bit_vec);

	std::cout
		<< "Command_vector_to_bitstream: " << test_bits + "\n"
		<< std::endl;

	std::string encoded_test_bits = Alc.encode_message(test_bits);
	std::cout
		<< "CRC encoded message: " << encoded_test_bits + "\n"
		<< std::endl;

	// Interface from Application Layer to Transport Layer

	AlToTl inter_1;

	inter_1.add_string_to_buffer(encoded_test_bits);

	Transport_Layer tl;

	std::string tl_header_test_bits = tl.add_header(inter_1.get_buffer());

	// auto segment_vector = tl.segment_msg(tl_header_test_bits);

	// TlToDll inter_2;

	// inter_2.add_segments_to_buffer(segment_vector);

	// DataLinkLayer dll(inter_2.take_segment_from_buffer());

	DataLinkLayer dll(tl_header_test_bits);


	dll.protocol_structure();

	DllToPl i_dl_pl;

	i_dl_pl.add_ready_msg(dll.get_ready_for_pl_path());

	std::string msg_to_send = i_dl_pl.get_ready_msg();

	std::cout << "Message to send: " << msg_to_send << std::endl;

	SignalProcessing sp;
	std::vector<int> dtmf = sp.convert_to_dtmf(msg_to_send);
	
	PhysicalLayer pl;
    pl.yell(dtmf);
	return 0;
}