#include "../include/algorithms.h"

namespace SP {

template <typename T>
Algorithms<T>::Algorithms(){

}

template <typename T>
Algorithms<T>::Algorithms(T data){
    _data.push_back(data);
}

template <typename T>
Algorithms<T>::Algorithms(std::vector<T> data): _data(data){

}

template <typename T>
void Algorithms<T>::print(){
    std::cout << _data[0] << std::endl;
}

template <typename T>
void Algorithms<T>::FFT(){
    std::cout << _data[0] << std::endl;
}

template <typename T>
void Algorithms<T>::goertzel(){
    std::cout << _data[0] << std::endl;
}

}