// #include <iostream>
// #include <string>
// #include <vector>
// #include <algorithm>

// using std::cout;
// using std::string;
// using std::endl;
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
