#include <iostream>
#include <fstream>
#include "algorithms.h"

int main(){

	std::vector<std::complex<double>> a = {{1, 2}, {3, 4}, {5, 6}, {7, 8}};
	SP::Algorithms<double> A(a);
	SP::Algorithms<double> B;
	A.print_data();

	std::cout << "\nFFT: " << std::endl;
	B.FFT(a);
	B.print_data();

	std::cout << "\nGoertzel: " << std::endl;
	A.goertzel();

	return 0;
}
