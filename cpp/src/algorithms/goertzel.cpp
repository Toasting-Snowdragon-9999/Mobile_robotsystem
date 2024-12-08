#include "algorithms/goertzel.h"

Goertzel::Goertzel(){}

Goertzel::Goertzel(bool hyperx, int sample_rate): _hyperx(hyperx), _sample_freq(sample_rate){}

Goertzel::Goertzel(GoertzelResult& r, bool hyperx, int sample_rate): _result(r), _hyperx(hyperx), _sample_freq(sample_rate){}


Goertzel::Goertzel(const std::vector<float> data)
        : _data(data), _size_of_signal(data.size()) {}

void Goertzel::translate_signal_goertzel(GoertzelResult& r){

    _init_coefficients();

    compute_goertzel();

    sort(_magnitudes, _DTMF_freq);

    detect_DTMF(_freq_from_signals[0], _freq_from_signals[1], r);

}

void Goertzel::_init_coefficients()  {
    _coefficients.resize(_DTMF_freq.size());

    for (int i = 0; i < _DTMF_freq.size(); ++i) {
        double omega = ( (2.0 * M_PI) / _size_of_signal ) * (0.5 + (2.0 * M_PI * _DTMF_freq[i]) / _sample_freq );
        _coefficients[i] = 2.0 * std::cos(omega);
    }
}

void Goertzel::compute_goertzel() {
    _magnitudes.resize(_DTMF_freq.size());

    for (int freq_index = 0; freq_index < _DTMF_freq.size(); ++freq_index) {
        
        //double omega = (2.0 * M_PI * _DTMF_freq[freq_index]) / _sample_freq;
        double omega = (2.0 * M_PI) / _size_of_signal * (0.5 + (_size_of_signal * _DTMF_freq[freq_index] / _sample_freq) );
        double cosine = std::cos(omega);
        double coeff = 2.0 * cosine;
        double signal = 0.0;
        double last_signal = 0.0;
        double before_last_signal = 0.0;

        for (int i = 0; i < _size_of_signal; ++i) {

            signal = coeff * last_signal - before_last_signal + _data[i];
            before_last_signal = last_signal;
            last_signal = signal;
        }
        double last_signal_square = last_signal * last_signal;
        double before_last_signal_square = before_last_signal * before_last_signal;

        _magnitudes[freq_index] = std::sqrt(last_signal_square + before_last_signal_square - last_signal * before_last_signal * coeff);
        //std::cout << "Magnitude: " << std::setprecision(15) << _magnitudes[freq_index] << std::endl;

    }
}


void Goertzel::read_from_file(const std::string &file_name){
    std::ifstream in_file(file_name);
    std::string line;

    while (std::getline(in_file, line)) {
        double value = std::stod(line);
        _data.push_back(value);

    }

    _size_of_signal = _data.size();

    in_file.close();
}

void Goertzel::load_data(const std::vector<float> &data){
    _data = data;
    _size_of_signal = data.size();
}

void Goertzel::sort(std::vector <double> &x, std::vector <int> &y){

    bool swap;

    for(int i = 0; i < (x.size())/2-1; i++){

        swap = false;

        for (int j = 0; j < (x.size())/2-1; j++){

            if (x[j] < x[j+1]){
                std::swap(x[j], x[j+1]);
                std::swap(y[j],y[j+1]);

                swap = true;
            }
        }
        if (!swap){
            break;
        }
    }

    for(int i = (x.size())/2; i < x.size()-1; i++){

        swap = false;

        for (int j = (x.size())/2; j < x.size()-1; j++){

            if (x[j] < x[j+1]){
                std::swap(x[j], x[j+1]);
                std::swap(y[j],y[j+1]);

                swap = true;
            }
        }
        if (!swap){
            break;
        }
    }
    if(_hyperx){
        multiplier = 1.5;
    }
    else{
        multiplier = 0;
    }
    float freq_mag_threshhold_high = 15.0/(1+multiplier); // 30.0
    float freq_mag_threshhold_low = 25.0/(1+multiplier); // 50.0
    
    if((x[0] != 0) && (x[4] != 0)){
        if((x[0] > freq_mag_threshhold_low && x[4] > freq_mag_threshhold_high)){
            _freq_from_signals.push_back(y[0]);
            _freq_from_signals.push_back(y[4]);
            //std::cout << "if statement virker " << x[0] << ", " << x[4] << std::endl;
        }
        else{
            _freq_from_signals.push_back(697);
            _freq_from_signals.push_back(852);
            //std::cout << "if statement virker ikke " << x[0] << ", " << x[4] << std::endl;
        }
    }
    else{
        _freq_from_signals.push_back(697);
        _freq_from_signals.push_back(770);
    }
    //std::cout << "Frequency found: " << _freq_from_signals[0] << " and " << _freq_from_signals[1] << std::endl;

}




void Goertzel::detect_DTMF(int freq_1, int freq_2, GoertzelResult& r) {
    if (freq_1 > freq_2) {
       std::swap(freq_1, freq_2);
    }

    auto DTMF_freq = _DTMF_mapping.find({freq_1, freq_2});

    if (DTMF_freq != _DTMF_mapping.end()) {
        //std::cout << "DTMF_Freq found: " << DTMF_freq->second << std::endl;
        if (DTMF_freq->second == -1){
            r.garbage_flag = false;
            r.tone_flag = false;
        }
        else{
            std::cout << "DTMF_Freq found: " << DTMF_freq->second << std::endl;
            r.garbage_flag = false;
            if(r.tone_flag){
                r.garbage_flag = true;
            }
            r.dtmf_tone = DTMF_freq->second;
            // save_to_json(DTMF_freq->second);
            r.tone_flag = true;
            //std::cout << "Tone flag: " << r.tone_flag << std::endl;
        }
    }
    else {
        
        //std::cout<< " DTMF_Freq not found " << std::endl;
    }

     /* -------------------- FOR DEBUG: -----------------------*/

    //std::cout << "Size of message vector: "<< _message_vec.size() << std::endl;

    //std::cout << "Content of message vector: "<< _message_vec[0] << std::endl;

}

std::vector<int> Goertzel::get_message_vec(){

    return _message_vec;
}

bool Goertzel::detect_bit(std::string type, int dtmf_tone){
    // Tone 14
    std::pair<int, int> freq_pair = _reverse_DTMF_mapping[dtmf_tone];
    int freq_1 = freq_pair.first;
    int freq_2 = freq_pair.second;
    //std::cout << "Freq 1: " << freq_1 << " Freq 2: " << freq_2 << std::endl;
    if ((_freq_from_signals[0] == freq_1 && _freq_from_signals[1] == freq_2) || (_freq_from_signals[0] == freq_2 && _freq_from_signals[1] == freq_1)) {
        std::cout << type <<" bit detected" << std::endl;
        return true;
    }
    return false;
}

std::string Goertzel::generate_json_string(const int& key) {
    // if (values.size() != 8) {
    //     throw std::invalid_argument("The vector must contain exactly 8 doubles.");
    // }

    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"key\": \"" << key << "\",\n";
    oss << "  \"magnitudes\": [";

    // Serialize the vector of doubles
    for (size_t i = 0; i < _magnitudes.size(); ++i) {
        oss << std::fixed << std::setprecision(6) << _magnitudes[i];
        if (i < _magnitudes.size() - 1) {
            oss << ", ";
        }
    }

    oss << "]\n";
    oss << "}";
    
    return oss.str();
}

void Goertzel::save_to_json(const int& key) {
    std::string jsonOutput = generate_json_string(key);
    std::string filePath = "../dtmf_sounds/magnitudes.json";

    // Read existing content
    std::ifstream inFile(filePath);
    std::ostringstream existingContent;
    if (inFile.is_open()) {
        existingContent << inFile.rdbuf();
        inFile.close();
    }

    std::string content = existingContent.str();

    // Handle JSON formatting
    std::ostringstream updatedContent;
    if (content.empty()) {
        // No existing content, start a new JSON array
        updatedContent << "[\n" << jsonOutput << "\n]";
    } else {
        // Remove the closing bracket of the existing array
        size_t closingBracketPos = content.find_last_of(']');
        if (closingBracketPos != std::string::npos) {
            content.erase(closingBracketPos);
        }

        // Append the new JSON object
        updatedContent << content;
        if (content.back() != '[') {
            updatedContent << ",\n";
        }
        updatedContent << jsonOutput << "\n]";
    }

    // Write back the updated JSON array
    std::ofstream outFile(filePath);
    if (outFile.is_open()) {
        outFile << updatedContent.str();
        outFile.close();
        std::cout << "JSON appended to " << filePath << std::endl;
    } else {
        std::cerr << "Error: Could not open file for writing." << std::endl;
    }
}
