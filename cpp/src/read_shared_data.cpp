#include "read_shared_data.h"

SharedData::SharedData(std::string fname): _fname(fname){}

std::vector<std::vector<uint16_t>> SharedData::read_shared_data(){
    this->read_data();
	this->calculate_path();
    return _path;
}

std::vector<std::vector <std::string>> SharedData::read_json(){
    std::vector<std::vector <std::string>> sorted_data;
    nlohmann::json jsonData;

    std::filesystem::path cwd = std::filesystem::current_path();
    std::cout << "Current working directory: " << cwd << std::endl;    std::ifstream inputFile(_fname);

    if (!inputFile.is_open()) {
        std::cerr << "Failed to open the JSON file" << std::endl;
    }

    try {
        inputFile >> jsonData;
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "Parsing error: " << e.what() << std::endl;
    }

    inputFile.close();

    std::cout << "JSON data loaded successfully:" << std::endl;

    for (size_t i = 0; i < jsonData.size(); ++i) {
        std::vector <std::string> temp;
        temp.push_back(jsonData[i][0]);
        int value = jsonData[i][1];
        temp.push_back(std::to_string(value)); 
        sorted_data.push_back(temp);
    }

    return sorted_data;
}


void SharedData::print(){
    for(auto a: _path){
        std::cout << "[ ";
        std::cout << a[0] << " ";
        std::cout << a[1] << " ";
        std::cout << a[2] << " ";
        std::cout << "]" << std::endl;
    }
}

void SharedData::read_data(){
    _path.clear();
	const char* fname = _fname.c_str();
	std::string s;
    std::fstream(fname, std::ios::in) >> s;
    if(s.empty()){
        throw SharedDataException("File is empty", 21);
    }
    system("clear");
    _data = this->string_to_uint128(s);
    //std::cout << s << '\n';
	std::string ss = " ";
	std::fstream(fname, std::ofstream::out | std::ofstream::trunc) >> ss;
}

__uint128_t SharedData::string_to_uint128(const std::string& str) {
    __uint128_t result = 0;
    for (char c : str) {
        if (c < '0' || c > '9') {
            throw SharedDataException("Invalid character in string");
        }
        result = result * 10 + (c - '0');
    }
    return result;
}

void SharedData::calculate_path(){
    std::vector<uint16_t> bit_chunks;
    __uint128_t copied_data = _data;
    while (copied_data > 0){
        uint16_t chunk = copied_data & 0xFFF;
        //std::cout << chunk << std::endl;
        bit_chunks.push_back(chunk);
        copied_data = copied_data >> 12;
    }
    // std::cout << bit_chunks.size() << std::endl;
    for(int i = 0; i < bit_chunks.size(); i++){
        uint16_t temp = bit_chunks[i];
        std::vector<uint16_t> temp_vec;
        temp_vec.push_back(temp & 0x000f);
        temp = temp >> 4;
        temp_vec.push_back(temp & 0x000f);
        temp = temp >> 4;
        temp_vec.push_back(temp & 0x00f);
        this->push_to_front(_path, temp_vec);
        temp_vec.clear();
    }
    this->sort(_path);
}

void SharedData::sort(std::vector<std::vector<uint16_t>>& unsorted){
    for(auto& a: unsorted){
        std::swap(a[0], a[2]);
    }
}

void SharedData::push_to_front(std::vector<std::vector<uint16_t>>& vec, std::vector<uint16_t> value) {
    vec.insert(vec.begin(), value);
}

