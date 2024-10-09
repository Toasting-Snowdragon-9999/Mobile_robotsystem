#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <cstdint>
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

        SharedData();
        std::vector<std::vector<uint16_t>> read_shared_data();
        /**
         * @brief printing method.
         *
         * @param nothing.
         * @return Nothing.
         */
        void print();
        /**
         * @brief Getter method for .
         *
         * @param nothing.
         * @return Nothing.
         */
        double get_data();

    private:
        __uint128_t _data;
        std::vector<std::vector<uint16_t>> _path;

        /**
         * @brief Read data method, used for reading the data from a shared files.
         *
         * @param nothing.
         * @return Nothing.
         */
        void read_data();
        void push_to_front(std::vector<std::vector<uint16_t>>& vec, std::vector<uint16_t> value);
        /**
         * @brief returning the an array of the full path.
         *
         * @param nothing.
         * @return Nothing.
         */
        std::vector<std::vector<int>> get_path();
        void sort(std::vector<std::vector<uint16_t>>& unsorted);
        void calculate_path();
        __uint128_t string_to_uint128(const std::string& str);
};