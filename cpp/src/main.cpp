#include <iostream>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"
#include <bitset>
#include <algorithm>
#include <string>

using std::string;

/**
 * @brief Binary conversion for CRC-check
 *
 * @param decimalSequence The original tone sequence in decimal, divided in vectors of vectors
 * @return The sequence in a long binary string
 */
string decimalSeqToBinaryMsg(std::vector<std::vector<int>> decimalSequence)
{
	int binaryConvertedTone = 0;
	std::string binaryConvertedToneStr;
	std::vector<int> buffer;

	for (auto pair : decimalSequence)
	{
		for (auto tone : pair)
		{
			int toneCopy = tone;
			if (tone == 0)
				binaryConvertedTone = binaryConvertedTone << 1 | 0; // If a tone is zero, no conversion is needed

			while (tone != 0)
			{
				buffer.push_back(tone % 2);
				tone /= 2;
			}

			if (toneCopy > 0) // Reverse buffer to get MSB to LSB
			{
				std::reverse(buffer.begin(), buffer.end());
				binaryConvertedTone = binaryConvertedTone << 1 | buffer[0];

				for (int i = 0; i < buffer.size() - 1; i++)
				{
					binaryConvertedTone = binaryConvertedTone << 1 | buffer[i + 1];
				}
			}
			buffer.clear();
		}
	}

	int actualBitLength = 0;
	int temp = binaryConvertedTone;
	while (temp > 0)
	{ // Find extra zeroes in binary conversion
		temp >>= 1;
		actualBitLength++;
	}

	std::bitset<32> redundantConvertedTone(binaryConvertedTone);
	binaryConvertedToneStr = redundantConvertedTone.to_string().substr(32 - actualBitLength);

	return binaryConvertedToneStr;
}

/// @brief Performs XOR-operation on 2 characters
/// @param a
/// @param b
/// @return Result of XOR-operation
string ExclusiveOR(char a, char b)
{
	if (a == b)
		return "0";
	else
		return "1";
}

/// @brief Encode binary data sequence with CRC4-Codeword
/// @param uncodedBinarySequence
/// @return Binary sequence with CRC4-Codeword appended
string CRCencode(string uncodedBinarySequence)
{
	string CRCFourCodeword = "10011";
	string CRCFourZeroes = "00000";
	string CRCremainder = "";
	string &CRCremainderRef = CRCremainder;
	int codewordLength = CRCFourCodeword.length();
	int datawordLength = uncodedBinarySequence.length();
	std::string::iterator subsIndexIt = uncodedBinarySequence.begin();
	int subsIndexInt = std::distance(uncodedBinarySequence.begin(), subsIndexIt);
	int subsIndexInt2 = codewordLength;
	int lengthWithoutCodeword = std::distance(uncodedBinarySequence.begin(), (uncodedBinarySequence.end() - codewordLength));
	int bitshiftCounter = 0;
	int firstCounter = 0;

	uncodedBinarySequence.append(CRCFourZeroes); // Append zeroes to data

	while (subsIndexInt <= lengthWithoutCodeword) // Binary-division
	{

		for (int j = 0; j < CRCFourCodeword.length(); j++) // XOR on Substring
		{
			if (firstCounter <= 0) // XOR on the first string
			{
				CRCremainderRef.append(uncodedBinarySequence.substr(subsIndexInt2, bitshiftCounter));
				CRCremainderRef.append(ExclusiveOR(uncodedBinarySequence.substr(subsIndexInt, subsIndexInt2)[j], CRCFourCodeword[j]));
			} else {


			}



			bitshiftCounter = 0;
		}

		while (CRCremainderRef[0] != '1') // Bit-shift until all zeroes from the beginning of the remainder
		{
			int CRCremainderInt = std::stoi(CRCremainder, nullptr);
			CRCremainderInt <<= 1;
			CRCremainderRef = std::to_string(CRCremainderInt);
			bitshiftCounter++;
			if (firstCounter == 0)
				firstCounter++;
		};
	}

	return CRCremainder;
}

int main()
{
	SharedData sd;

	// lyde
	std::vector<std::vector<int>> sequence1 = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}, {13, 14, 15}, {16, 0, 0}}; // missing 16
	std::vector<std::vector<int>> sequence2 = {{2, 7, 4}, {16, 1, 2}, {14, 8, 6}, {1, 3, 9}, {15, 3, 4}, {9, 16, 1}};
	std::vector<std::vector<int>> sequence3 = {{16, 16, 16}, {1, 1, 1}, {16, 16, 16}, {1, 1, 1}, {16, 16, 16}};
	std::vector<std::vector<int>> sequence4 = {{11, 8, 10}};
	std::vector<std::vector<int>> testSequence1 = {{1}};
	std::vector<std::vector<int>> preambleSeq = {{10, 10, 15}};

	// WaveGenerator sounds(testSequence1);
	// sounds.play_sounds();

	// WaveGenerator preamble(preambleSeq);
	// preamble.play_sounds();

	// int crcDivisor = 0b10011;

	// Test af binary conversion
	std::cout << decimalSeqToBinaryMsg(preambleSeq) << std::endl;

	string teststring1 = "101010";
	string teststring2 = "010101";
	string xorresult;

	for (int i = 0; i < teststring1.length(); i++)
	{
		xorresult.append(ExclusiveOR(teststring1[i], teststring2[i]));
	}

	CRCencode(decimalSeqToBinaryMsg(testSequence1));

	// lyde ended

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
