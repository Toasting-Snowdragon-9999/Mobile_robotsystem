#include "goertzel.h"
#include "dft.h"
#include "algorithms.h"
#include <chrono>

int main() {

    std::string file = "output.txt";

    auto start_DFT = std::chrono::high_resolution_clock::now();
    DFT dft;
    dft.read_from_file(file);
    dft.init();
    auto stop_DFT = std::chrono::high_resolution_clock::now();
    auto duration_DFT = std::chrono::duration_cast<std::chrono::microseconds>(stop_DFT-start_DFT);
    std::cout << "Time for DFT: " << duration_DFT.count() << " microseconds " << std::endl;

    auto start_goertzel = std::chrono::high_resolution_clock::now();
    Goertzel goertzel;
    goertzel.read_from_file(file);
    goertzel.init();
    auto stop_goertzel = std::chrono::high_resolution_clock::now();
    auto duration_goertzel = std::chrono::duration_cast<std::chrono::microseconds>(stop_goertzel-start_goertzel);
    std::cout << "Time for Goertzel: " << duration_goertzel.count() << " microseconds " << std::endl;

    return 0;
}




