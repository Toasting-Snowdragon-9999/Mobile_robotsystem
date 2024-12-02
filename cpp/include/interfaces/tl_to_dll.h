#include <vector>
#include <string>

using std::string;

class TlToDll
{
private:
    std::vector<string> _segment_buffer;

public:
    std::vector<string> add_segment_to_buffer(const string &encoded_segment);
};
