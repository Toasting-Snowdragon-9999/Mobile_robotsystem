#include "fft.h"
#include "goertzel.h"
#include "dft.h"

int main() {

    /*-------------------------------DFT-Run_Code----------------------------------*/

//        std::string file = "output.txt";
//        DFT dft;
//        dft.read_from_file(file);
//        dft.frequencies_of_signal();

    /*-------------------------------For-loop for time_testing----------------------------------*/

    //    for(int i = 0 ; i < 30 ; ++i){
    //        auto start_DFT = std::chrono::high_resolution_clock::now();
    //        auto stop_DFT = std::chrono::high_resolution_clock::now();
    //        auto duration_DFT = std::chrono::duration_cast<std::chrono::microseconds>(stop_DFT-start_DFT);
    //        std::cout << duration_DFT.count() << std::endl;
    //    }

    /*------------------------------------------------------------------------------*/

    /*-------------------------------Goertzel-Run_Code----------------------------------*/

             std::string file = "output.txt";
             int sum = 0;

             for(int i = 0 ; i < 30 ; ++i){
             auto start_goertzel = std::chrono::high_resolution_clock::now();

             DFT dft;
             dft.read_from_file(file);

             //Goertzel goertzel;
             //goertzel.read_from_file(file);
             //goertzel.translate_signal_goertzel();
             auto stop_goertzel = std::chrono::high_resolution_clock::now();
             auto duration_goertzel = std::chrono::duration_cast<std::chrono::microseconds>(stop_goertzel-start_goertzel);

             sum += duration_goertzel.count();
             // std::cout << duration_goertzel.count() << std::endl;
             }
             std::cout << "Time for Goertzel: " << sum/30 << " microseconds " << std::endl;

    /*-------------------------------For-loop for time_testing----------------------------------*/

    //    for(int i = 0 ; i < 30 ; ++i){
    //      auto start_goertzel = std::chrono::high_resolution_clock::now();
    //      auto stop_goertzel = std::chrono::high_resolution_clock::now();
    //      auto duration_goertzel = std::chrono::duration_cast<std::chrono::microseconds>(stop_goertzel-start_goertzel);
    //      std::cout << duration_goertzel.count() << std::endl;
    //    }
    //       std::cout << "Time for Goertzel: " << duration_goertzel.count() << " microseconds " << std::endl;


    //  }

     /*--------------------------------------------------------------------------------------------*/


    /*-------------------------------FFT-Run_Code----------------------------------*/

//    std::string file = "output.txt";
//    for(int i = 0 ; i < 30 ; ++i){
//        auto start_DFT = std::chrono::high_resolution_clock::now();
//    FFT fftProcessor;
//    fftProcessor.read_from_file(file);
//    fftProcessor.perform_fft();

//        auto stop_DFT = std::chrono::high_resolution_clock::now();
//        auto duration_FFT = std::chrono::duration_cast<std::chrono::microseconds>(stop_DFT-start_DFT);
//        std::cout << duration_FFT.count() << std::endl;
//    }

    /*--------------------------------------------------------------------------------------------*/

     /*-------------------------------For-loop for time_testing----------------------------------*/
       /*
        for(int i = 0 ; i < 30 ; ++i){
            auto start_DFT = std::chrono::high_resolution_clock::now();
            auto stop_DFT = std::chrono::high_resolution_clock::now();
            auto duration_FFT = std::chrono::duration_cast<std::chrono::microseconds>(stop_DFT-start_DFT);
            std::cout << duration_FFT.count() << std::endl;

        }
        */
     /*--------------------------------------------------------------------------------------------*/

    return 0;
}
