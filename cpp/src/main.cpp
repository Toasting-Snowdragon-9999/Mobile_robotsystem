#include <iostream>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"
#include "com_protocol.h"
#include "applicationlayer_conversion.h"
#include <bitset>
#include <algorithm>
#include <string>

using std::cout;
using std::endl;
using std::string;

int main()
{
	robot_command s("s");
	robot_command r1("-fw", "100");
	robot_command r2("-l", "45");
	robot_command r3("-r", "180");
	robot_command r4("-fw", "3000");
	robot_command r5("-r", "30");

	ApplicationlayerConversion Alc;

	cout << "Command to bits: " << Alc.command_to_bits(r1) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(s) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r2) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(s) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r3) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(s) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r4) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(s) << endl;
	cout << "Command to bits: " << Alc.command_to_bits(r5) + "\n"
		 << endl;

	string testBits = Alc.command_to_bits(r1) + Alc.command_to_bits(s) + Alc.command_to_bits(r2) + Alc.command_to_bits(s) + Alc.command_to_bits(r3) + Alc.command_to_bits(s) + Alc.command_to_bits(r4) + Alc.command_to_bits(s) + Alc.command_to_bits(r5);

	cout
		<< "Complete string: " << testBits + "\n"
		<< endl;
	cout << "Correct commands: -fw 100 , -l 45 , -r 180 , -fw 3000 , -r 30\nConvert commands: ";
	Alc.print_robot_commands(Alc.bits_to_commands(testBits));

	// SharedData sd; static_cast<unsigned long>

	// std::vector<std::vector<int>> bigPath = {
	// 	{1, 5, 3, 7, 9, 2, 8, 4, 6, 0},
	// 	{10, 14, 12, 11, 15, 13, 1, 5, 7, 3},
	// 	{8, 2, 4, 6, 10, 12, 14, 11, 9, 0},
	// 	{15, 13, 1, 4, 7, 10, 3, 2, 5, 8},
	// 	{9, 0, 12, 6, 14, 11, 13, 15, 1, 7},
	// 	{3, 10, 4, 5, 8, 6, 9, 2, 12, 11},
	// 	{14, 0, 7, 13, 15, 3, 5, 10, 1, 8},
	// 	{6, 2, 9, 11, 4, 12, 14, 0, 7, 13},
	// 	{3, 1, 15, 5, 8, 6, 10, 9, 2, 11},
	// 	{4, 14, 7, 12, 0, 13, 1, 5, 9, 10},
	// 	{2, 8, 11, 3, 15, 6, 4, 7, 14, 12},
	// 	{13, 0, 1, 9, 8, 10, 2, 3, 5, 11},
	// 	{7, 4, 15, 6, 13, 12, 9, 1, 10, 3},
	// 	{8, 14, 5, 11, 7, 0, 6, 2, 12, 9},
	// 	{13, 4, 14, 3, 1, 10, 15, 8, 5, 7},
	// 	{2, 6, 12, 11, 9, 0, 3, 14, 5, 8},
	// 	{10, 7, 1, 13, 15, 4, 9, 12, 11, 2},
	// 	{6, 8, 0, 5, 3, 14, 7, 10, 1, 13},
	// 	{12, 9, 11, 2, 4, 0, 8, 6, 15, 7},
	// 	{3, 5, 14, 1, 12, 9, 10, 13, 4, 11}
	// };

	/* TEST SHOULD BE REMOVED */
	// std::vector<std::vector<uint16_t>> pathTest = {{13,1,5}};
	// ComProtocol packageVec(pathTest);
	
	// std::vector<std::vector<uint16_t>> package = packageVec.protocol_structure();
	// package.erase(package.begin(), package.begin() + 2); // Removes preamble, length, and postamble from vector
	// package.erase(package.end() - 1);

	// for (const auto& innerVec : package) {
    //     for (const auto& elem : innerVec) {
    //         std::cout << elem << " ";
    //     }
    //     std::cout << std::endl; // Newline after each inner vector
    // }

	// std::string encoded = packageVec.decimal_seq_to_binary_msg(package);
	// std::cout << "Encoded: " <<  encoded << std::endl;
	// std::string binaryDecodedMsg = packageVec.crc4_decode(encoded);
	// std::cout << "Decoded: " << binaryDecodedMsg << std::endl;

    // if (packageVec.find_remainder(binaryDecodedMsg) == "0000") // If message is correct return it
    // {
    //     binaryDecodedMsg.erase(binaryDecodedMsg.end() - (4 * 2), binaryDecodedMsg.end()); // Removes CRC from end of message
	// 	std::cout << binaryDecodedMsg << std::endl;
    // }
	// else
	// {
	// 	std::cout << "NOT CORRECT" << std::endl;
	// }
	/* END OF TEST */


	
	std::vector<std::vector<uint16_t>> robotPath = {{13, 2, 0}, {12, 5, 0}, {13, 7, 5}, {13, 4, 5}};
	ComProtocol testPackage(robotPath);

	std::string binaryMsg = testPackage.decimal_seq_to_binary_msg(robotPath);

	std::string encodedMsg = testPackage.crc4_encode(binaryMsg);
	std::string encodedMsgRemainder = testPackage.find_remainder(encodedMsg);

	std::string decodedMsg = testPackage.crc4_decode(encodedMsg);
	std::string decodedMsgRemainder = testPackage.find_remainder(decodedMsg);

	testPackage.print_nested_vector(testPackage.protocol_structure(), "Test Package");
	std::cout << "Converted binary robot path: " << binaryMsg << endl;
	std::cout << "CRC4-encoded robot path: " << encodedMsg << " | CRC-remainder: " << encodedMsgRemainder << std::endl;
	std::cout << "CRC4-decoded robot path: " << decodedMsg << " | CRC-remainder: " << decodedMsgRemainder << std::endl;

	if (testPackage.is_message_correct(decodedMsg))
	{
		std::cout << "Recieved package IS correct!" << std::endl;
	}
	else
	{
		std::cout << "Recieved package IS NOT correct!" << std::endl;
	}

	std::vector<std::vector<uint16_t>> fullPackageVector = testPackage.protocol_structure();
	std::cout << std::endl;
	WaveGenerator testPackageWave(fullPackageVector);
	testPackageWave.play_sounds();

	std::vector<uint16_t> concatenated;
    for (const auto& innerVec : fullPackageVector) {
        concatenated.insert(concatenated.end(), innerVec.begin(), innerVec.end());
    }
	std::vector<std::vector<uint16_t>> finalVec = {concatenated};

	std:string receivedMsg = testPackage.get_binary_message_from_package(finalVec);
	std::cout << "Received message: " << receivedMsg << std::endl;
	

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
