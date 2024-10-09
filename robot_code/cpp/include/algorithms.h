#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <iostream>
#include <vector>

namespace SP {

template <typename T>
class Algorithms {
private:
    std::vector<T> _data;

public:
    Algorithms();
    Algorithms(T data);
    Algorithms(std::vector<T> data);
    void print();

};

}

#include "../src/algorithms.tpp"

#endif