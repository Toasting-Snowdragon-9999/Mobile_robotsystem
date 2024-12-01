#include <iostream>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"
#include "com_protocol.h"
#include "al_to_dll.h"
#include "application_layer.h"
#include <bitset>
#include <algorithm>
#include <string>
#include "crc.h"
#include "transport_layer.h"

using std::cout;
using std::endl;
using std::string;

int main()
{
	robot_command r1("-fw", "100");
	robot_command r2("-l", "45");
	robot_command r3("-r", "180");
	robot_command r4("-bw", "300000");
	robot_command r5("-r", "30");

	ApplicationLayer Alc;

	cout << "Command to bits: " << Alc.command_to_bits(r1) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r2) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r3) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r4) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r5) + "\n"
		 << endl;

	string test_bits = Alc.command_to_bits(r1) + Alc.command_to_bits(r2) + Alc.command_to_bits(r3) + Alc.command_to_bits(r4) + Alc.command_to_bits(r5);

	cout
		<< "Complete string: " << test_bits + "\n"
		<< endl;
	cout << "The correct commands: -fw 100 , -l 45 , -r 180 , -bw 300000 , -r 30\nConverted  commands: \n";
	Alc.print_robot_commands(Alc.bits_to_commands(test_bits));

	Transport_Layer tl;

	string appended_test = tl.add_header(test_bits);

	std::cout << "With B&E:		" << appended_test << std::endl;
	std::cout << "Without B&E:		" << tl.remove_header(appended_test) << std::endl;

	tl.segment_msg(appended_test);
	auto segments_vector = tl.get_segments_vector();
	std::cout << "Segmented msg:" << std::endl;
	tl.print_segment_vector(segments_vector);

	if (tl.combine_segments_to_string() == appended_test)
		std::cout << "SUCCESS!!" << std::endl;
	std::cout << "Combined msg:" << tl.combine_segments_to_string() << std::endl;

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

	return 0;
}
