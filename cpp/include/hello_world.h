#ifndef HELLO_WORLD_H
#define HELLO_WORLD_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class HelloWorld{
    public:
        class HelloWorldError : public std::runtime_error {
            public:
                HelloWorldError(std::string error) : std::runtime_error(error) {}
                virtual const char* what() const throw() {
                    return "Print error";
                }
        };  
        HelloWorld();
        /**
         * @brief printing method for printing the private string.
         *
         * @param nothing.
         * @return Nothing.
         */
        void print();
        
    private: 
        std::string _hello; 

};

#endif