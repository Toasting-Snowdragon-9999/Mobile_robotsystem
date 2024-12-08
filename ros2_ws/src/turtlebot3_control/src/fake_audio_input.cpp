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

void AudioInput::initialise_flags(){
    _mic_data.recorded_samples.clear();
    _mic_data.iterator = 0;
    _mic_data.pre_success = false;
    _mic_data.pre_1_flag = false;
    _mic_data.stop = false;
    
    _mic_data.result_in_mic.dtmf_tone = -5;
    _mic_data.result_in_mic.garbage_flag = false;
    _mic_data.result_in_mic.tone_flag = false;
    _mic_data.result_in_mic.esc_flag = false;
}

void AudioInput::record_audio(int input_device){
    initialise_flags();
    std::cout << "SATTE INPUT DEVICE: " << input_device << std::endl;
    _input_parameters.device = 2;//input_device;  // Explicitly select Device #2, or you can use another device index (like 8)
    if (_input_parameters.device == paNoDevice) {
        std::cerr << "Error: No input device.\n";
        return;
    }

    _input_parameters.channelCount = NUM_CHANNELS;               // Mono input
    _input_parameters.sampleFormat = SAMPLE_TYPE;       // 32-bit floating point
    _input_parameters.suggestedLatency = Pa_GetDeviceInfo(_input_parameters.device)->defaultLowInputLatency;
    _input_parameters.hostApiSpecificStreamInfo = NULL;

    std::string filepath = "/home/pi/ros2_ws/dtmf_sounds/magnitudes.json";
    std::ofstream outFile(filepath);
    if (outFile.is_open()) {
        outFile << "";
        outFile.close();
    } else {
        std::cerr << "Error: Could not open file for writing." << std::endl;
    }

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
        // break;
    }
    save_to_textfile("/home/pi/ros2_ws/dtmf_sounds/output.txt");
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

    //auto start = std::chrono::high_resolution_clock::now();
    MicSample *data = (MicSample*)userData; 
    const float *in = (const float*)input_buffer; /* Audio input data */
    unsigned int i;
    std::vector <float> buffer;
    (void) output_buffer; /* Prevent unused variable warning. */
    float thresh_hold = 0.5;
    std::vector <int> _preamble = {14, 0};
    int _esc_tone = 15;  

    if( input_buffer == NULL ){
        std::cout << "Input buffer is NULL" << std::endl;
        return paContinue;
    }

    for( i = 0; i < frames_per_buffer; i++ ){
        float mono_in = *in++;  /* Mono channel input */
	//std::cout <<  mono_in << std::endl;
        if (std::abs(mono_in) < thresh_hold){
            mono_in = 0;
        }

		buffer.push_back(mono_in);
    }

    data->recorded_samples.push_back(buffer);

    if(data->pre_success){
        Goertzel algo;
        algo.load_data(buffer);
        algo.translate_signal_goertzel(data->result_in_mic);
        //std::cout << "Tone Flag: " << data->result_in_mic.tone_flag << std::endl;
        //std::cout << "Garbage Flag: " << data->result_in_mic.garbage_flag << std::endl;
        //std::cout << "Esc Flag: " << data->result_in_mic.esc_flag << std::endl;
        //std::cout << "Pre 1 flag: " << data->pre_1_flag << std::endl;
        if (data->result_in_mic.tone_flag && !data->result_in_mic.garbage_flag){
            if (data->recorded_DTMF_tones.back() != _preamble[0]){
                data->pre_1_flag = false;  
            }
            data->recorded_DTMF_tones.push_back(data->result_in_mic.dtmf_tone);
            //std::cout << "Detected DTMF tone: " << data->result_in_mic.dtmf_tone << std::endl;
            if(algo.detect_bit("esc", _esc_tone)){
                data->result_in_mic.esc_flag = true;
            }
            
            if((algo.detect_bit("stop 1", _preamble[0]) || algo.detect_bit("stop 2", _preamble[1])) && !data->result_in_mic.esc_flag){
                if(data->result_in_mic.dtmf_tone == _preamble[0]){
                    data->pre_1_flag = true;
                }
                else if(data->result_in_mic.dtmf_tone == _preamble[1] && data->pre_1_flag){
                    data->stop = true;
                    return paComplete;
                }
            }
            
            if(data->recorded_DTMF_tones.back() != _esc_tone){
                data->result_in_mic.esc_flag = false;
            } 
        }
    }
    else{
        Goertzel algo;
        algo.load_data(buffer);
        algo.translate_signal_goertzel(data->result_in_mic);
        //std::cout << "pre 1 Flag: " << data->pre_1_flag << std::endl;
        if((algo.detect_bit("start 1", _preamble[0]) || algo.detect_bit("start 2", _preamble[1])) && (data->result_in_mic.tone_flag && !data->result_in_mic.garbage_flag)){
            if(data->pre_1_flag && data->result_in_mic.dtmf_tone == _preamble[1]){
                data->pre_success = true;
                data->pre_1_flag = false;
                data->recorded_DTMF_tones.push_back(data->result_in_mic.dtmf_tone);
            }
            else if(data->result_in_mic.dtmf_tone == _preamble[0]){
                data->pre_1_flag = true;
                data->recorded_DTMF_tones.push_back(data->result_in_mic.dtmf_tone);
            }

        }
        else{
           if((data->result_in_mic.dtmf_tone != _preamble[0] && data->result_in_mic.dtmf_tone != _preamble[1] && data->result_in_mic.dtmf_tone != -1)  && !data->result_in_mic.garbage_flag){
                //std::cout << "Garbage tone detected: " << data->result_in_mic.dtmf_tone << std::endl;
                //std::cout << "Garbage flag: " << data->result_in_mic.garbage_flag << std::endl;
                data->pre_1_flag = false;
                data->recorded_DTMF_tones.clear();
           }
            buffer.clear();
            data->pre_success = false;
        }
    }

    //auto end = std::chrono::high_resolution_clock::now();

    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    //std::cout << "Time difference: " << duration.count() << " microseconds" << std::endl;

    return paContinue;
}

void AudioInput::save_to_textfile(const std::string &fileName){
    
    std::ofstream outFile(fileName);
    outFile << "Start: " << "\n";
    for (auto &sample : _mic_data.recorded_samples) {
        for (auto &value : sample) {
            //std::cout << "Buffer value: " <<value << std::endl;
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

void AudioInput::check_sequence(TestResult &result, std::vector<int> &tones, std::vector<int> &test_sequence){
    // if (tones.size() != test_sequence.size()){
    //     result.error.push_back("Invalid sequence length");
    //     result.failure = 1;
    //     result.success = 0;        
    //     return;
    // }
    
    int zero_padding;
    zero_padding = abs(tones.size()-test_sequence.size());
    if (zero_padding != 0){
        if(tones.size() > test_sequence.size()){
            for(int i = 0; i < zero_padding; i++){
                test_sequence.push_back(0);
            }
        }
        else{
            for(int i = 0; i < zero_padding; i++){
                tones.push_back(0);
            }
        }
    }
    for (int i = 0; i < tones.size(); i++){
        if(tones[i] == test_sequence[i]){
            result.error.push_back("Valid: " + std::to_string(test_sequence[i]) + ", " + std::to_string(tones[i]));
            result.success++;
        }
        else{
            result.error.push_back("Invalid. Correct " + std::to_string(test_sequence[i]) + " but got " + std::to_string(tones[i]));
            result.failure++;
        }
    }
}

bool AudioInput::check(bool full_output, std::vector<int> &test_sequence){

    TestResult result;
    result.error.clear();
    result.success = 0;
    result.failure = 0;

    check_sequence(result, _mic_data.recorded_DTMF_tones, test_sequence);
    if(full_output){
        for(auto& a: result.error){
            std::cout << a << std::endl;
        }
    }

    std::cout << "Amount of Success: " << result.success << std::endl;
    std::cout << "Amount of Failure: " << result.failure << std::endl;
    return result.failure;
}


