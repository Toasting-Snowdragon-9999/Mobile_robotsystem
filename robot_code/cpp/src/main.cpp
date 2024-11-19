#include "fft.h"
#include "goertzel.h"
#include "dft.h"
#include "signal_processing.h"
#include "audio_input.h"
#include <unistd.h>
#include <iostream>
#include <fstream>
#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (1024)
#define INPUT_DEVICE (14)  /* Device index for the input device 5 default*/

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

            //std::string sti = "../dtmf_sounds/Signals_for_Size_test/";

            //std::vector<std::string> file = {//"output.txt"
       /*Goertzel er ustabil ved 400 Dog er DFT*/                                     
  //                                          "output_400.txt", "output_500.txt", "output_600.txt", "output_700.txt", "output_800.txt", "output_900.txt",
  //                                          "output_1000.txt", "output_2000.txt", "output_3000.txt", "output_4000.txt",
  //                                          "output_5000.txt","output_6000.txt", "output_7000.txt",
  //                                          "output_8000.txt", "output_9000.txt", "output_10000.txt"
    /*For FFT Grænsen 16394*/        //
                                             //"output_16000.txt", "output_17000.txt",
                                            //"output_18000.txt", "output_19000.txt", "output_20000.txt", "output_21000.txt", "output_22000.txt"

            //};

            // for (int j = 0 ; j < file.size(); j++){
            //     std::string full_path_file = sti + file[j];

            //     int sum = 0;

                //for(int i = 0 ; i < 100 ; ++i){

                    //DFT dft;
                    //Goertzel goertzel;
                    //FFT fftProcessor;
                    
                    //goertzel.read_from_file(full_path_file);
                    //dft.read_from_file(full_path_file);
                    //fftProcessor.read_from_file(full_path_file);
                    //auto start_goertzel = std::chrono::high_resolution_clock::now();

                    //goertzel.translate_signal_goertzel();
                    //dft.frequencies_of_signal();
                    //fftProcessor.perform_fft();

                    // auto stop_goertzel = std::chrono::high_resolution_clock::now();
                    // auto duration_goertzel = std::chrono::duration_cast<std::chrono::microseconds>(stop_goertzel-start_goertzel);

                    // sum += duration_goertzel.count();
               // }
                //std::cout << "Time for Goertzel: " << sum/30 << " microseconds " << std::endl;
            //std::cout << sum/100 << std::endl;
            //}
 
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



SIGNAL_PROCESSING processor;
std::vector<int> message = {10, 1, 1, 1, 1, 1, 1, 10};

std::vector<int> binary_representation1 = processor.pre_postamble_remover(message);

std::string binary_representation3 = processor.message_str_binary();
std::cout << binary_representation3 << std::endl;

std::string binary_representation = processor.message_str_binary();
std::cout << binary_representation << std::endl;



    return 0;

    AudioInput audio_input(SAMPLE_RATE, FRAMES_PER_BUFFER);
    audio_input.audio_open();
    audio_input.list_audio_devices();
    std::cout << "Recording audio..." << std::endl;
    audio_input.record_audio(INPUT_DEVICE);
    audio_input.save_to_wav("../dtmf_sounds/output.wav");
    audio_input.save_to_textfile("../dtmf_sounds/output.txt");
    audio_input.read_from_file("../dtmf_sounds/output.txt");
    audio_input.audio_close();
	return 0;
}
