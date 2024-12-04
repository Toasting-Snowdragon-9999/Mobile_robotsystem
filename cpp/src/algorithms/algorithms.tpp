#include "../include/algorithms.h"

namespace SP {

template <typename T>
Algorithms<T>::Algorithms(){

}

template <typename T>
Algorithms<T>::Algorithms(std::vector<std::complex<T>> data): _data(data){

}

template <typename T>
void Algorithms<T>::print_data(){
    for(int i = 0; i < _data.size(); i++){
        std::cout << _data[i];
        if(i < _data.size()-1){
            std::cout << ", ";
        }
    }
    std::cout << std::endl;
}

template <typename T>
void Algorithms<T>::FFT(std::vector<std::complex<double>>& data) {

    int n = data.size();
    if (n <= 1) return;

    std::vector<std::complex<double>> even(n / 2);
    std::vector<std::complex<double>> odd(n / 2);

    // Split x into even and odd parts
    for (int i = 0; i < n / 2; ++i) {
        even[i] = data[i * 2];    // even indexed elements
        odd[i] = data[i * 2 + 1]; // odd indexed elements
    }

    // Recursive FFT on even and odd parts
    FFT(even);
    FFT(odd);

    // Combine results
    for (int k = 0; k < n / 2; ++k) {
        std::complex<double> t = std::polar(1.0, -2 * M_PI * k / n) * odd[k]; // W_n^k
        data[k] = even[k] + t;            // First half
        data[k + n / 2] = even[k] - t;    // Second half
    }
    _data = data;
}

template <typename T>
void Algorithms<T>::goertzel(){
    std::cout << "goertzel" << std::endl;
}

}