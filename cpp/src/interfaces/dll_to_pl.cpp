#include "dll_to_pl.h"

string DllToPl::add_ready_msg(const string &msg_to_send)
{

    _ready_msg = msg_to_send;
}

string DllToPl::get_ready_msg()
{
    return _ready_msg;
}
