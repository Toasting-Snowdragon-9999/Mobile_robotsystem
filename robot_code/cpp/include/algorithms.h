#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <iostream>
#include <vector>
#include <complex>

namespace SP {

template <typename T>
class Algorithms {

    private:
        std::vector<std::complex<T>> _data;

    public:
        Algorithms();
        Algorithms(std::vector<std::complex<T>> data);
        void FFT(std::vector<std::complex<double>>& data);
        void print_data();
        //void FFT();
        void goertzel();

};

}

#include "../src/algorithms.tpp"

#endif