#ifndef HELLO_WORLD_H
#define HELLO_WORLD_H

#include <iostream>
#include <string>

class HelloWorld{
    private: 
        std::string _hello; 
    
    public:    
        HelloWorld();
        void print();

};

#endif