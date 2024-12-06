#ifndef DLL_TO_PL_H
#define DLL_TO_PL_H

#include <string>

class DllToPl
{
private:
    std::string _ready_msg;

public:
    void add_ready_msg(const std::string &msg_to_send);

    std::string get_ready_msg();
};

#endif