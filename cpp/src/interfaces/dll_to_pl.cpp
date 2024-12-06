#include "interfaces/dll_to_pl.h"

void DllToPl::add_ready_msg(const std::string &msg_to_send)
{

    _ready_msg = msg_to_send;
}

std::string DllToPl::get_ready_msg()
{
    return _ready_msg;
}
