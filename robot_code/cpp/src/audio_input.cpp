#include "audio_input.h"

AudioInput::AudioInput(int sample_rate, int frames_per_buffer): _sample_rate(sample_rate), _frames_per_buffer(frames_per_buffer){}

AudioInput::~AudioInput(){}

void AudioInput::list_audio_devices(){
    int numDevices = Pa_GetDeviceCount();
    const PaDeviceInfo *deviceInfo;

    std::cout << "Available audio devices:\n";
    for (int i = 0; i < numDevices; i++) {
        deviceInfo = Pa_GetDeviceInfo(i);
        std::cout << "Device #" << i << ": " << deviceInfo->name << "\n";
        std::cout << "Max input channels: " << deviceInfo->maxInputChannels << "\n";
        std::cout << "Max output channels: " << deviceInfo->maxOutputChannels << "\n";
    }

    int defaultInputDevice = Pa_GetDefaultInputDevice();
    std::cout << "\nDefault input device: " << defaultInputDevice << " - " << Pa_GetDeviceInfo(defaultInputDevice)->name << "\n";

}

void AudioInput::save_to_wav(const std::string &fileName){
    std::ofstream outFile(fileName, std::ios::binary);

    // WAV file header parameters
    int dataSize = _mic_data.recorded_samples.size() * sizeof(SAMPLE);
    int subChunk1Size = 16;  // PCM header size
    short audioFormat = 3;   // PCM float (3 for float, 1 for PCM integer)
    short numChannels = NUM_CHANNELS;
    int byteRate = _sample_rate * numChannels * sizeof(SAMPLE);
    short blockAlign = numChannels * sizeof(SAMPLE);
    short bitsPerSample = 8 * sizeof(SAMPLE);
    int chunkSize = 36 + dataSize;
    //double denormalize = 2147483647;
    // Write the RIFF header
    outFile.write("RIFF", 4);
    outFile.write(reinterpret_cast<const char *>(&chunkSize), 4);
    outFile.write("WAVE", 4);

    // Write the fmt subchunk
    outFile.write("fmt ", 4);
    outFile.write(reinterpret_cast<const char *>(&subChunk1Size), 4);
    outFile.write(reinterpret_cast<const char *>(&audioFormat), 2);
    outFile.write(reinterpret_cast<const char *>(&numChannels), 2);
    outFile.write(reinterpret_cast<const char *>(&_sample_rate), 4);
    outFile.write(reinterpret_cast<const char *>(&byteRate), 4);
    outFile.write(reinterpret_cast<const char *>(&blockAlign), 2);
    outFile.write(reinterpret_cast<const char *>(&bitsPerSample), 2);

    // Write the data subchunk
    outFile.write("data", 4);
    outFile.write(reinterpret_cast<const char *>(&dataSize), 4);
    outFile.write(reinterpret_cast<const char *>(_mic_data.recorded_samples.data()), dataSize);

    outFile.close();
}

void AudioInput::record_audio(int input_device){
    _mic_data.recorded_samples.clear();
    _mic_data.success = false;
    _mic_data.iterator = 0;
    _mic_data.stop = false;
    _mic_data.result_in_mic.dtmf_tone = -5;
    _mic_data.result_in_mic.garbage_flag = false;
    _mic_data.result_in_mic.tone_flag = false;
    _input_parameters.device = input_device;  // Explicitly select Device #2, or you can use another device index (like 8)
    if (_input_parameters.device == paNoDevice) {
        std::cerr << "Error: No input device.\n";
        return;
    }

    _input_parameters.channelCount = 1;               // Mono input
    _input_parameters.sampleFormat = SAMPLE_TYPE;       // 32-bit floating point
    _input_parameters.suggestedLatency = Pa_GetDeviceInfo(_input_parameters.device)->defaultLowInputLatency;
    _input_parameters.hostApiSpecificStreamInfo = NULL;
    
    _err = Pa_OpenStream(
            &_stream,
            &_input_parameters,
            NULL, // No output
            _sample_rate,
            _frames_per_buffer,
            paClipOff, // No clipping
            read_mic_callback,
            &_mic_data);

    if (_err != paNoError) {
        std::cerr << "PortAudio error: " << Pa_GetErrorText(_err) << std::endl;
        Pa_Terminate();
        return;
    }

    _err = Pa_StartStream( _stream );
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
        Pa_Terminate();
        return;
    }
    auto start = std::chrono::high_resolution_clock::now();

    while (_mic_data.stop == false){
        // Pa_Sleep(MILLISECONDS); 
    }
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "[AFTER] Time difference: " << duration.count() << " microseconds" << std::endl;


    _err = Pa_StopStream( _stream );
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
    }

    _err = Pa_CloseStream( _stream );
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
    }

    for(auto a: _mic_data.recorded_DTMF_tones){
        std::cout << a << " " << std::endl;
    }

}

void AudioInput::audio_close(){
    _err = Pa_Terminate();
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
    }
}

void AudioInput::audio_open(){
    _err = Pa_Initialize();
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
    }
}

static int read_mic_callback( const void *input_buffer, void *output_buffer,
                           unsigned long frames_per_buffer,
                           const PaStreamCallbackTimeInfo* time_info,
                           PaStreamCallbackFlags status_flags,
                           void *userData ){

    auto start = std::chrono::high_resolution_clock::now();
    MicSample *data = (MicSample*)userData; 
    const float *in = (const float*)input_buffer; /* Audio input data */
    unsigned int i;
    std::vector <float> buffer;
    (void) output_buffer; /* Prevent unused variable warning. */
    float thresh_hold = 0.012;
    if( input_buffer == NULL ){
        std::cout << "Input buffer is NULL" << std::endl;
        return paContinue;
    }

    for( i = 0; i < frames_per_buffer; i++ ){
        float mono_in = *in++;  /* Mono channel input */
        if (mono_in < thresh_hold){
            mono_in = 0;
        }
		buffer.push_back(mono_in);
    }

    if(data->success){
        Goertzel algo;
        algo.load_data(buffer);
        algo.translate_signal_goertzel(data->result_in_mic);
        std::cout << "Tone Flag: " << data->result_in_mic.tone_flag << std::endl;
        std::cout << "Garbage Flag: " << data->result_in_mic.garbage_flag << std::endl;
        if (data->result_in_mic.tone_flag && !data->result_in_mic.garbage_flag){
            data->recorded_DTMF_tones.push_back(data->result_in_mic.dtmf_tone);
            std::cout << "Detected DTMF tone: " << data->result_in_mic.dtmf_tone << std::endl;
            if(algo.detect_start_bit("stop")){
                data->stop = true;
                return paComplete;
            }
        }
    }
    else{
        Goertzel algo;
        algo.load_data(buffer);
        algo.translate_signal_goertzel(data->result_in_mic);
        if(algo.detect_start_bit("start")){
            data->success = true;
            data->recorded_DTMF_tones.push_back(data->result_in_mic.dtmf_tone);
        }
        else{
            buffer.clear();
            data->success = false;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    //std::cout << "Time difference: " << duration.count() << " microseconds" << std::endl;

    return paContinue;
}

// int AudioInput::dissect(){

//     Goertzel algo;
//     algo.load_data(_mic_data.recorded_samples[1]);
//     algo.translate_signal_goertzel();
//     if(algo.detect_start_bit("start")){
//         std::vector<SAMPLE> temp = _mic_data.recorded_samples[1];
//         for(int i = 0; i < 3; i++){
//             if (_check_second_half(temp)){
                 
//             }
//             else{
//                 return temp.size();
//             }
            
//         }
//     }
//     else{
//         algo.load_data(_mic_data.recorded_samples[0]);
//         algo.translate_signal_goertzel();
//         if(algo.detect_start_bit("start")){
            
//         }
//     }
// }

// bool AudioInput::_check_second_half(std::vector<SAMPLE>& vec){
//     std::vector<SAMPLE> second_half; 
//     for(int i = vec.size()/2; i < vec.size(); i++){
//         second_half.push_back(vec[i]);
//     }

//     Goertzel algo;
//     algo.load_data(second_half);
//     algo.translate_signal_goertzel();
//     if(algo.detect_start_bit("start")){
//         vec = second_half;
//         return true;
//     }
//     else{
//         return false;
//     }
// }

void AudioInput::save_to_textfile(const std::string &fileName){
    std::ofstream outFile(fileName);

    for (auto &sample : _mic_data.recorded_samples) {
        for (auto &value : sample) {
            outFile << value << "\n";
        }
    }

    outFile.close();

}

void AudioInput::read_from_file(const std::string &fileName){
    std::ifstream inFile(fileName);
    std::string line;

    while (std::getline(inFile, line)) {
        printf("%s\n", line.c_str());
    }

    inFile.close();
}