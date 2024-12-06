#ifndef READ_SHARED_DATA_H
#define READ_SHARED_DATA_H

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <filesystem>   
#include "communication_protocol/application_layer.h"

class SharedDataException : public std::runtime_error {
    public:
        int _error_code;

        SharedDataException(const std::string& message, int error_code = 0): std::runtime_error(message), _error_code(error_code) {}
        int error_code(){
            return _error_code;
        }
        virtual const char* what() const noexcept override {
            std::ostringstream oss;
            oss << std::runtime_error::what(); // Base error message
            if (_error_code != 0) {
                oss << " (Error code: " << _error_code << ")";
            }
            
            static std::string full_error;
            full_error = oss.str(); // Update static variable with new message
            return full_error.c_str(); // Return pointer to the message
        }
};

class SharedData{
    public:

        SharedData(std::string fname);
        /**
         * @brief This method reads from the shared_file.txt found in Docs and then makes some bit shifting and AND operation to split it into a list of lists of each 3 uint16 commands.
         *
         * @param nothing.
         * @return std::vector<std::vector<uint16_t>>
         */
        std::vector<std::vector<uint16_t>> read_shared_data();
        std::vector<std::vector <std::string> > read_json();
        /**
         * @brief printing method for printint the data vector like [[command1, command2, command3] , [command4, command5, command6]].
         *
         * @param nothing.
         * @return Nothing.
         */
        void print();
        /**
         * @brief Getter method for the data vector.
         *
         * @param nothing.
         * @return std::vector<std::vector<uint16_t>>.
         */
        std::vector<std::vector<uint16_t>> get_data();

    private:
        __uint128_t _data;
        std::vector<std::vector<uint16_t>> _path;
        std::string _fname;

        /**
         * @brief Read data method, used for reading the data from a shared files.
         *
         * @param nothing.
         * @return Nothing.
         */
        void read_data();
        /**
         * @brief Read data method, used for reading the data from a shared files.
         *
         * @param vec a list on the form std::vector<std::vector<uint16_t>>& which is a reference to a vector.
         * @param value a list on the form std::vector<uint16_t> that should be the first element in the vec.
         * @return Nothing.
         */
        void push_to_front(std::vector<std::vector<uint16_t>>& vec, std::vector<uint16_t> value);
        /**
         * @brief returning the an array of the full path.
         *
         * @param nothing.
         * @return Nothing.
         */
        void sort(std::vector<std::vector<uint16_t>>& unsorted);
        void calculate_path();
        __uint128_t string_to_uint128(const std::string& str);
};

#endif