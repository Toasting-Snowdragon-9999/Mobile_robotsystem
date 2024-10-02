#include "read_shared_data.h"

SharedData::SharedData(){}

std::vector<std::vector<uint16_t>> SharedData::read_shared_data(){
    this->read_data();
	this->calculate_path();
    return _path;
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
	const char* fname = "../../Docs/shared_file.txt";
	std::string s;
    std::fstream(fname, std::ios::in) >> s;
    if(s.empty()){
        throw SharedDataException("File is empty", 21);
    }
    _data = std::stod(s);
    std::cout << s << '\n';
	std::string ss = " ";
	std::fstream(fname, std::ofstream::out | std::ofstream::trunc) >> ss;
}

void SharedData::calculate_path(){
    std::vector<uint16_t> bit_chunks;
    uint64_t copied_data = _data;
    while (copied_data > 0){
        uint16_t chunk = copied_data & 0x0FFF;
        std::cout << chunk << std::endl;
        bit_chunks.push_back(chunk);
        copied_data = copied_data >> 12;
    }
    std::cout << "we are here " << std::endl;
    std::cout << bit_chunks.size() << std::endl;
    for(int i = 0; i < bit_chunks.size(); i++){
        uint16_t temp = bit_chunks[i];
        std::vector<uint16_t> temp_vec;
        temp_vec.push_back(temp & 0x000f);
        temp = temp >> 4;
        temp_vec.push_back(temp & 0x000f);
        temp = temp >> 4;
        temp_vec.push_back(temp & 0x00f);
        this->push_to_front(_path, temp_vec);
        //_path.push_forward(temp_vec);
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

