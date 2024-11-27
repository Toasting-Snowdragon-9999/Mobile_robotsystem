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

using std::cout;
using std::endl;
using std::string;

int main()
{
	robot_command r1("-fw", "100");
	robot_command r2("-l", "45");
	robot_command r3("-r", "180");
	robot_command r4("-fw", "300000");
	robot_command r5("-r", "30");

	ApplicationLayer Alc;

	cout << "Command to bits should be 1010000100000000 :		 " << Alc.command_to_bits(r1) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r2) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r3) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r4) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r5) + "\n"
		 << endl;

	string testBits = Alc.command_to_bits(r1) + Alc.command_to_bits(r2) + Alc.command_to_bits(r3) + Alc.command_to_bits(r4) + Alc.command_to_bits(r5);

	cout
		<< "Complete string: " << testBits + "\n"
		<< endl;
	cout << "The correct commands: -fw 100 , -l 45 , -r 180 , -fw 3000 , -r 30\nConverted  commands: ";
	Alc.print_robot_commands(Alc.bits_to_commands(testBits));

	// SharedData sd; static_cast<unsigned long>

	std::cout << std::endl;
	std::string test_path = "0110001111100";
	std::cout << "The path to be sent: \"" << test_path << "\" (length = " << test_path.size() << ")" << std::endl;

	ComProtocol test_package(test_path);
	std::string full_package_string = test_package.protocol_structure();
	std::cout << "Full package: " << full_package_string << std::endl;

	test_package.get_data_from_package(full_package_string);

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
