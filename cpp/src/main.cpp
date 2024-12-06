#include <iostream>
#include <fstream>
#include <bitset>
#include <algorithm>
#include <string>
#include "crc.h"
#include <filesystem>
#include <string>

#include "read_shared_data.h"
#include "wave_generator.h"
#include "application_layer.h"
#include "data_link_layer.h"
#include "transport_layer.h"
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"
#include "interfaces/dll_to_pl.h"

using std::cout;
using std::endl;
using std::string;

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

	// FYI *There exists both command_to_bits and command_vector_to_bitstream*

	string test_bits = Alc.command_vector_to_bitstream(test_bit_vec);

	cout
		<< "Command_vector_to_bitstream: " << test_bits + "\n"
		<< endl;

	string encoded_test_bits = Alc.encode_message(test_bits);
	cout
		<< "CRC encoded message: " << encoded_test_bits + "\n"
		<< endl;

	// Interface from Application Layer to Transport Layer

	AlToTl inter_1;

	inter_1.add_string_to_buffer(encoded_test_bits);

	Transport_Layer tl;

	string tl_header_test_bits = tl.add_header(inter_1.get_buffer());

	auto segment_vector = tl.segment_msg(tl_header_test_bits);

	// cout << "Segmented msg:" << endl;
	// tl.print_segment_vector(segments_vector);

	// Interface from Transport Layer to Data Link Layer

	TlToDll inter_2;

	inter_2.add_segments_to_buffer(segment_vector);

	DataLinkLayer dll(inter_2.take_segment_from_buffer());

	dll.protocol_structure();

	DllToPl inter_3;

	// 	while (dll.is_ack_received() == false)
	// {
	// 	//inter_3.add_ready_msg(dll.get_ready_for_pl_path());

	// }

	inter_3.add_ready_msg(dll.get_ready_for_pl_path());

	string msg_to_send = inter_3.get_ready_msg();



// ======================================================
// SENDER
// ======================================================

	string bitstuff_test = "1111101011111010111111";

	cout << "Max consecutive ones of " << bitstuff_test << ": " << tl.find_max_ones(bitstuff_test)
		 << endl;

	cout << "Bit stuffing of " << bitstuff_test << ": 		" << tl.bit_stuff(bitstuff_test) << endl;
	cout << "\nBitstream for test:	";
	string header_test = "1010111110";
	cout << header_test << endl;

	cout << "Step 0: Bit stuffing of " << header_test << ": 		" << tl.bit_stuff(header_test) << endl;

	string test_with_header = tl.add_header(header_test);

	cout << "Step 1: Add header after bitstuffing of " << header_test << ": 		" << test_with_header << endl;
	cout << "Step 2: Length of header " << test_with_header << " | Should be 10: 		" << tl.get_length_from_header(test_with_header) << endl;

	cout << "Step 3: Remove after unstuffing of " << test_with_header << ": 		" << tl.remove_header_and_unstuff(test_with_header) << endl;

	std::string test_string_dll = "101011110110011101110";
	DataLinkLayer lucas_dll(test_string_dll);
	std::string sending_package = lucas_dll.protocol_structure();
	lucas_dll.get_data_from_package(sending_package);

	// SharedData sd; static_cast<unsigned long>

	// cout << endl;    int i;
	// std::string test_path = "0110001111100";
	// cout << "The path to be sent: \"" << test_path << "\" (length = " << test_path.size() << ")" << endl;

	// ComProtocol test_package(test_path);
	// std::string full_package_string = test_package.protocol_structure();
	// cout << "Full package: " << full_package_string << endl;

	// test_package.get_data_from_package(full_package_string);

	// cout << "Converted bits_to_command, the correct answer is -fw:		"; Alc.print_robot_commands(Alc.bits_to_commands("1010000100001011100011010010"));

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
	// 			cout << "[Error] " << e.what() << endl;
	// 		}
	// 	}
	// }
	// py to cpp ended
/*
	// std::filesystem::path currentPath = std::filesystem::current_path();
    // std::cout << "Current working directory: " << currentPath << std::endl;
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

	std::cout << test_sequence2.size() << std::endl;

	try{
		sd.read_json();
		sd.print();
	}
	catch(SharedDataException &e){
		if (e.error_code() == 21){
		}
		else{std::cout << "[Error] " << e.what() << std::endl;}
	}

	WaveGenerator sounds(test_sequence2);
	sounds.play_sounds();
	std::string filename = "../dtmf_sounds/dtmf_sounds.wav";
	sounds.save_to_wav_file(filename);
*/
	return 0;
}