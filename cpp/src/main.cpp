#include <iostream>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"
#include "com_protocol.h"
#include <bitset>
#include <algorithm>
#include <string>

using std::cout;
using std::endl;
using std::string;

/*THIS MIGHT BE DELETED*/
// /**
//  * @brief Binary conversion for CRC-check
//  *
//  * @param decimalSequence The original tone sequence in decimal, divided in vectors of vectors
//  * @return The sequence in a long binary string
//  */
// string decimalSeqToBinaryMsg(const std::vector<std::vector<int>> &decimalSequence)
// {
// 	string binaryConvertedTone = "";
// 	std::string tmpTone;

// 	for (auto pair : decimalSequence)
// 	{
// 		for (auto tone : pair)
// 		{
// 			int toneCopy = tone;
// 			if (tone == 0)
// 			{
// 				tmpTone += '0'; // If a tone is zero, no conversion is needed

// 				continue;
// 			}

// 			while (toneCopy > 0)
// 			{
// 				tmpTone += (toneCopy % 2) ? '1' : '0';

// 				toneCopy /= 2;
// 			}

// 			std::reverse(tmpTone.begin(), tmpTone.end());
// 			binaryConvertedTone += tmpTone;
// 			tmpTone.clear();
// 		}
// 	}

// 	return binaryConvertedTone;
// }

// /// @brief Performs XOR-operation on 2 strings characterwise
// /// @param a
// /// @param b
// /// @return Result of XOR-operation
// string ExclusiveORStrings(string a, string b)
// {
// 	string xorresult = "";
// 	if (a.size() != b.size())
// 	{
// 		throw std::invalid_argument("Strings of XOR-operation are not same size");
// 	}
// 	for (int i = 0; i < a.size(); i++)
// 	{
// 		xorresult += (a[i] == b[i]) ? '0' : '1';
// 	}
// 	return xorresult;
// }

// /// @brief Encode binary dataword with CRC4-Codeword
// /// @param binaryDataword
// /// @return Binary dataword with CRC4-Codeword appended
// string CRC4Encode(string binaryDataword)
// {
// 	string codeword = "10011";
// 	string encodedBinaryData = binaryDataword;

// 	int crcDegree = codeword.length() - 1;

// 	int selectionPlusOneIdx = codeword.length();
// 	int codewordLength = codeword.length();

// 	string binaryDatawordWithZeroes = binaryDataword + string(crcDegree, '0'); // Append CRC-Degree zeroes to data

// 	string selection = binaryDatawordWithZeroes.substr(0, codeword.length());
// 	int datawordLength = binaryDatawordWithZeroes.length();

// 	while (selectionPlusOneIdx < datawordLength) // Binary-division
// 	{
// 		if (selection[0] == '1')
// 		{
// 			selection = ExclusiveORStrings(selection, codeword);
// 		}

// 		selection = selection.substr(1) + binaryDatawordWithZeroes[selectionPlusOneIdx];
// 		selectionPlusOneIdx++;
// 	}

// 	if ((selection[0] == '1')) // XOR the last selection with the codeword if needed
// 	{
// 		selection = ExclusiveORStrings(selection, codeword);
// 	}

// 	string remainder = selection.substr(1); // Return substring since codeword is CRC-Degree+1 in size

// 	return encodedBinaryData = binaryDataword + remainder;
// }

// // Function to print a vector of chars
// void printNestedVector(const std::vector<std::vector<int>> &preambleSeq, const std::string &name = "Preamble Sequence")
// {
// 	std::cout << name << ":" << std::endl;
// 	std::cout << "[" << std::endl;
// 	for (const std::vector<int> &innerVec : preambleSeq)
// 	{
// 		std::cout << "  [ ";
// 		for (int num : innerVec)
// 		{
// 			std::cout << num << " ";
// 		}
// 		std::cout << "]" << std::endl;
// 	}
// 	std::cout << "]" << std::endl;
// }
/*END OF DELETION*/

int main()
{
	SharedData sd;

	// lyde
	// std::vector<std::vector<int>> sequence1 = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}, {13, 14, 15}, {16, 0, 0}}; // missing 16
	// std::vector<std::vector<int>> sequence2 = {{2, 7, 4}, {16, 1, 2}, {14, 8, 6}, {1, 3, 9}, {15, 3, 4}, {9, 16, 1}};
	// std::vector<std::vector<int>> sequence3 = {{16, 16, 16}, {1, 1, 1}, {16, 16, 16}, {1, 1, 1}, {16, 16, 16}};
	// std::vector<std::vector<int>> sequence4 = {{11, 8, 10}};
	// std::vector<std::vector<int>> testSequence1 = {{10, 10, 10}, {13, 2, 0, 0}, {10, 10, 10}};

	// WaveGenerator sounds(testSequence1);
	// sounds.play_sounds();

	// WaveGenerator preamble(preambleSeq);
	// preamble.play_sounds();
	// lyde ended

	std::vector<std::vector<int>> testSequenceFULL = {{13, 2, 0, 0}, {12, 5, 9}};
	ComProtocol testPackage(testSequenceFULL);

	testPackage.print_nested_vector(testPackage.protocol_structure(), "Test Package");
	std::cout << "Converted binary robot path: " << testPackage.decimal_seq_to_binary_msg(testSequenceFULL) << endl;
	std::cout << "CRC4-encoded robot path: " << testPackage.crc4_encode(testPackage.decimal_seq_to_binary_msg(testSequenceFULL))
			  << " | CRC-remainder: " << testPackage.crc4_encode(testPackage.decimal_seq_to_binary_msg(testSequenceFULL)).substr(testPackage.crc4_encode(testPackage.decimal_seq_to_binary_msg(testSequenceFULL)).size() - 4) << std::endl;

	WaveGenerator testPackageWave(testPackage.protocol_structure());
	testPackageWave.play_sounds();

	// py to cpp
	while (1)
	{
		try
		{
			sd.read_shared_data();
			sd.print();
		}
		catch (SharedDataException &e)
		{
			if (e.error_code() == 21)
			{
			}
			else
			{
				std::cout << "[Error] " << e.what() << std::endl;
			}
		}
	}
	// py to cpp ended

	return 0;
}
