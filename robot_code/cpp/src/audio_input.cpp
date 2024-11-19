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

    while (_mic_data.stop == false){
        Pa_Sleep(MILLISECONDS); 
    }
    

    _err = Pa_StopStream( _stream );
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
    }

    _err = Pa_CloseStream( _stream );
    if( _err != paNoError ) {
        std::cout << "PortAudio error: " << Pa_GetErrorText( _err ) << std::endl;
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


    MicSample *data = (MicSample*)userData; 
    const float *in = (const float*)input_buffer; /* Audio input data */
    unsigned int i;
    std::vector <float> buffer;
    (void) output_buffer; /* Prevent unused variable warning. */

    if( input_buffer == NULL ){
        std::cout << "Input buffer is NULL" << std::endl;
        return paContinue;
    }

    for( i = 0; i < frames_per_buffer; i++ ){
        float mono_in = *in++;  /* Mono channel input */
		buffer.push_back(mono_in);
    }
    if(data->success){
        data->recorded_samples.push_back(buffer);
        if (data->iterator > 1){
            Goertzel algo;
            algo.load_data(buffer);
            bool success = algo.translate_signal_goertzel();
            if(algo.detect_start_bit()){
                data->stop = true;
                data->recorded_samples.push_back(buffer);
            }
        }
        data->iterator++;
    }
    else{
        Goertzel algo;
        algo.load_data(buffer);
        bool success = algo.translate_signal_goertzel();
        if(algo.detect_start_bit()){
            data->success = true;
            data->recorded_samples.push_back(buffer);
        }
        else{
            buffer->clear();
            data->success = false;
        }
    }

    return paContinue;
}

void AudioInput::save_to_textfile(const std::string &fileName){
    std::ofstream outFile(fileName);

    for (auto &sample : _mic_data.recorded_samples) {
        outFile << sample << "\n";
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