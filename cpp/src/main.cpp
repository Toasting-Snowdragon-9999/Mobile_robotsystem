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
#include "audio/wave_generator.h"
#include "crc.h"

// =======================Communication Layers============================
#include "communication_protocol/application_layer.h"
#include "communication_protocol/data_link_layer.h"
#include "communication_protocol/transport_layer.h"
#include "communication_protocol/physical_layer.h"

// =======================Inerfaces============================
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"
#include "interfaces/signal_processing.h"

int main()
{

// ======================================================
// SENDER
// ======================================================
	std::vector<int> ack = {14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 0};
	PhysicalLayer pl;
	std::vector<int> dtmf_sounds = pl.listen(false);
	
	SignalProcessing sp(dtmf_sounds);
	std::string binary_msg = sp.message_str_binary();
	
	DataLinkLayer dl_layer(binary_msg);
	std::string package = dl_layer.get_data_from_package();

	TlToDll i_tl;
	i_tl.add_segment_to_buffer(package);
	std::string segment = i_tl.take_segment_from_buffer();

	AlToTl i_al;
	i_al.add_string_to_buffer(segment);
	std::string buffer = i_al.get_buffer();	

	ApplicationLayer app_layer;
	std::vector<robot_command> commands = app_layer.bits_to_commands(buffer);

	pl.yell(ack);

// ======================================================
// SENDER
// ======================================================



	// SharedData sd; static_cast<unsigned long>

	// std::cout << std::endl;    int i;
	// std::string test_path = "0110001111100";
	// std::cout << "The path to be sent: \"" << test_path << "\" (length = " << test_path.size() << ")" << std::endl;

	// ComProtocol test_package(test_path);
	// std::string full_package_string = test_package.protocol_structure();
	// std::cout << "Full package: " << full_package_string << std::endl;

	// test_package.get_data_from_package(full_package_string);

	// std::cout << "Converted bits_to_command, the correct answer is -fw:		"; Alc.print_robot_commands(Alc.bits_to_commands("1010000100001011100011010010"));

	// // py to cpp
	// while (1)
	// {
	// 	try
	// 	{
	// 		sd.read_shared_data();
	// 		sd.print();
	// 	}
	// 	catch (SharedDataException &e)
	// 	{
	// 		if (e.error_code() == 21)
	// 		{
	// 		}
	// 		else
	// 		{
	// 			std::cout << "[Error] " << e.what() << std::endl;
	// 		}
	// 	}
	// }
	// py to cpp ended
/*
	// std::filesystem::path currentPath = std::filesystem::current_path();
    // std::std::cout << "Current working directory: " << currentPath << std::std::endl;
	// return 0;
	std::string path_gui = "../Docs/shared_file.json";
	std::string path_debug = "../../Docs/shared_file.json";
	SharedData sd(path_debug);

	std::vector<uint16_t> initial_test = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

	std::vector<int> test_sequence1 = // 50 numbers
		{14, 0, 5, 3, 5, 7, 11, 2, 8, 1,
		6, 4, 10, 9, 13, 12, 11, 14, 7,
		6, 3, 8, 5, 2, 10, 11, 9, 0, 12,
		4, 13, 1, 15, 14, 0, 2, 5, 8, 11,
		10, 9, 0, 13, 12, 3, 4, 15, 1, 14, 0};

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

	std::std::cout << test_sequence2.size() << std::std::endl;

	try{
		sd.read_json();
		sd.print();
	}
	catch(SharedDataException &e){
		if (e.error_code() == 21){
		}
		else{std::std::cout << "[Error] " << e.what() << std::std::endl;}
	}

	
*/
	return 0;
}
