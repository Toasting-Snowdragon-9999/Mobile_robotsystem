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
    //    int sum = 0;

    //    for(int i = 0 ; i < 30 ; ++i){
    //        auto start_DFT = std::chrono::high_resolution_clock::now();
    //        auto stop_DFT = std::chrono::high_resolution_clock::now();
    //        auto duration_DFT = std::chrono::duration_cast<std::chrono::microseconds>(stop_DFT-start_DFT);
    //        std::cout << duration_DFT.count() << std::endl;
    //    }

    /*------------------------------------------------------------------------------*/

    /*-------------------------------Goertzel-Run_Code----------------------------------*/

            std::string sti = "../cpp/dtmf_sounds/Signals_for_Size_test/";

            std::vector<std::string> file = {   //"buffer_400.txt",
                                            "output_500.txt", "output_600.txt", "output_700.txt",
                                            "output_800.txt", "output_900.txt", "output_1000.txt", 
                                            "output_2000.txt", "output_3000.txt", "output_4000.txt", 
                                            "output_5000.txt", "output_6000.txt", "output_7000.txt", 
                                            "output_8000.txt", "output_9000.txt", "output_10000.txt"
            };

            

            for (int j = 0 ; j < file.size(); j++){
                std::string full_path_file = sti + file[j];

               //DFT dft;
                //Goertzel goertzel;

 FFT fftProcessor;

                int sum = 0;
                for(int i = 0 ; i < 30 ; ++i){

                    auto start_goertzel = std::chrono::high_resolution_clock::now();
                    //goertzel.read_from_file(full_path_file);
                    //goertzel.translate_signal_goertzel();

                    //dft.read_from_file(full_path_file);
                    //dft.frequencies_of_signal();


                    fftProcessor.read_from_file(full_path_file);
                    fftProcessor.perform_fft();

                    auto stop_goertzel = std::chrono::high_resolution_clock::now();
                    auto duration_goertzel = std::chrono::duration_cast<std::chrono::microseconds>(stop_goertzel-start_goertzel);

                    sum += duration_goertzel.count();
            }
                //std::cout << "Time for Goertzel: " << sum/30 << " microseconds " << std::endl;
            std::cout << sum/30 << std::endl;
            }
 
    /*-------------------------------For-loop for time_testing----------------------------------*/

    //    for(int i = 0 ; i < 30 ; ++i){
    //      auto start_goertzel = std::chrono::high_resolution_clock::now();
    //      auto stop_goertzel = std::chrono::high_resolution_clock::now();
    //      auto stop_goertzel = std::chrono::high_resolution_clock::now();
    //      auto duration_goertzel = std::chrono::duration_cast<std::chrono::microseconds>(stop_goertzel-start_goertzel);

    //      sum += duration_goertzel.count();
    //    }
    //    std::cout << "Time for Goertzel: " << sum/30 << " microseconds " << std::endl;



    //  }

     /*--------------------------------------------------------------------------------------------*/


    /*-------------------------------FFT-Run_Code----------------------------------*/

//    std::string file = "../dtmf_sounds/Signal_For_FFT_Test.txt";
//    for(int i = 0 ; i < 30 ; ++i){
//        auto start_DFT = std::chrono::high_resolution_clock::now();

//      FFT fftProcessor;
//      fftProcessor.read_from_file(file);
//      fftProcessor.perform_fft();

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
