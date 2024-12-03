#include <iostream>
#include <fstream>
#include <bitset>
#include <algorithm>
#include <string>
#include "crc.h"
#include "read_shared_data.h"
#include "wave_generator.h"
#include "application_layer.h"
#include "data_link_layer.h"
#include "transport_layer.h"
#include "interfaces/al_to_tl.h"
#include "interfaces/tl_to_dll.h"

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

	// FYI *There exists both command_to_bits and command_vector_to bitstream*

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

	// Interface from Transport Layer to Data Link Layer



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

	return 0;
}
