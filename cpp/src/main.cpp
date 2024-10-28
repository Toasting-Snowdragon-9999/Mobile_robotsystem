#include <iostream>
#include <fstream>
#include "read_shared_data.h"
#include "wave_generator.h"
#include <bitset>
#include <algorithm>

// Binary conversion for CRC-check
int decimalSeqToBinaryMsg(std::vector<std::vector<int>> decimalSequence)
{
	int binaryConvertedTone;

	for (auto row : decimalSequence)
	{
		std::vector<int> binaryConversionBuffer;
		int toneBitCounter = 0;
		for (auto tone : row)
		{
			int firstCounter = 0;
			int toneCopy = tone;
			while (toneCopy != 0)
			{
				binaryConversionBuffer.push_back(toneCopy % 2);
				toneCopy /= 2; // Integer division
				toneBitCounter++;
			}

			if (firstCounter < 1)
			{
				binaryConvertedTone = binaryConversionBuffer.back(); // Very First exception
				firstCounter++;
			}
			for (int i = 1; i <= toneBitCounter; i++)
			{
				binaryConvertedTone = binaryConvertedTone << 1 | (binaryConversionBuffer.back() - 1); // Left-Bitshift and OR operation
				binaryConversionBuffer.pop_back();
			}
			binaryConversionBuffer.clear();
		}
	}
			return binaryConvertedTone;

}

		int main()
		{
			SharedData sd;

			// lyde
			std::vector<std::vector<int>> sequence1 = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}, {13, 14, 15}, {16, 0, 0}}; // missing 16
			std::vector<std::vector<int>> sequence2 = {{2, 7, 4}, {16, 1, 2}, {14, 8, 6}, {1, 3, 9}, {15, 3, 4}, {9, 16, 1}};
			std::vector<std::vector<int>> sequence3 = {{16, 16, 16}, {1, 1, 1}, {16, 16, 16}, {1, 1, 1}, {16, 16, 16}};
			std::vector<std::vector<int>> sequence4 = {{11, 8, 10}};
			std::vector<std::vector<int>> testSequence1 = {{1, 1, 1}};
			std::vector<std::vector<int>> preambleSeq = {{10, 10, 15}};

			// WaveGenerator sounds(testSequence1);
			// sounds.play_sounds();

			// WaveGenerator preamble(preambleSeq);
			// preamble.play_sounds();

			// int crcDivisor = 0b10011;

			// Test af binary conversion
			std::cout << decimalSeqToBinaryMsg(preambleSeq) << std::endl;

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
