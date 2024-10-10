#include "read_shared_data.h"
#include "socket_connection.h"

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

void SharedData::print_bin(int num, int size) {
    switch (size) {
        case 4: {
            std::bitset<4> binary(num & 0b1111); // Mask to limit to 4 bits
            std::cout << binary << std::endl;
            break;
        }
        case 8: {
            std::bitset<8> binary(num & 0b11111111); // Mask to limit to 8 bits
            std::cout << binary << std::endl;
            break;
        }
        case 16: {
            std::bitset<16> binary(num & 0b1111111111111111); // Mask to limit to 16 bits
            std::cout << binary << std::endl;
            break;
        }
        case 32: {
            std::bitset<32> binary(num & 0xFFFFFFFF); // Mask to limit to 32 bits
            std::cout << binary << std::endl;
            break;
        }
        default:
            std::cout << "Invalid size" << std::endl;
            break;
    }
}

void SharedData::read_data(){
    char exspected_start = 0b11111110;
    char exspected_stop = 0b11111101;
    char data[1024];
    int port = 62908;
    SocketConnection socket;
    if (socket.connect(port)){
        throw SharedDataException("Socket connection failed", 81);
    }
    ReadResult received = socket.read();  // Use a pointer to receive data
    std::copy(received.data, received.data + received.length, data); // Copy the actual received data
    int end_of_data = 0;
    std::cout << "Recieved length: " << received.length << std::endl;
    print_bin(data[0]);
    if (data[0] == exspected_start){
        for(int i = 1; i <= received.length; i++){
            if(data[i] == exspected_stop){
                std::cout << "Stop bit found at: " << i << std::endl;
                end_of_data = i;
                break;
            }
        }
    }
    else{
        if(data[received.length-1] == exspected_start){
            std::cout << "It is backwards" << std::endl;
        }
        std::cout << "Start byte was not the first byte" << std::endl;
    }

}

void SharedData::read_data_file(){
    _path.clear();
	const char* fname = "../../Docs/shared_file.txt";
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

