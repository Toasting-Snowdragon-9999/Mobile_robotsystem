#include <vector>
#include <deque>
#include <string>

using std::string;

class TlToDll
{
private:
    std::deque<string> _segment_buffer;

public:
    // Sender
    std::deque<string> add_segments_buffer(const std::vector<string> &segment_buffer);

    string take_segment_from_buffer();

    // Receiver
    void add_segment_to_buffer(const string &encoded_segment);
};
