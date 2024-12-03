#include <string>

using std::string;

class DllToPl
{
private:
    string _ready_msg;

public:
    string add_ready_msg(const string &msg_to_send);

    string get_ready_msg();
};