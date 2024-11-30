#include "transport_layer.h"

Transport_Layer::Transport_Layer() {}

string Transport_Layer::add_begin_and_end(const string &binary_msg)
{

    return _begin_byte + binary_msg + _end_byte;
}

string Transport_Layer::remove_begin_and_end(const string &full_binary_msg)
{
    string stripped_msg = full_binary_msg;

    size_t length = stripped_msg.length();
    int begin_length = _begin_byte.length();
    size_t end_length = _end_byte.length();

    // Remove begin byte
    if (stripped_msg.find(_begin_byte) != 0)
    {
        std::cout << "Beginning is not the begin-byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(0, _begin_byte.length());
    }

    length = stripped_msg.length();

    // Remove end byte
    if (stripped_msg.rfind(_end_byte) != length - end_length)
    {
        std::cout << "End-byte is not final byte of transport layer message. Error has ocurred" << std::endl;
    }
    else
    {
        stripped_msg.erase(length - end_length, end_length);
    }

    return stripped_msg;
}
