#include <string>


class DllToPl
{
private:
    std::string _ready_msg;

public:
    std::string add_ready_msg(const std::string &msg_to_send);

    std::string get_ready_msg();
};