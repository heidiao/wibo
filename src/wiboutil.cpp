#include <vector>
#include <string>
using namespace std;
vector<string> splitString(string input, string delimiter)
{
    vector<string> output;
    size_t start = 0;
    size_t end = 0;

    while (start != string::npos && end != string::npos){
        start = input.find_first_not_of(delimiter, end);
        if (start != string::npos){
            end = input.find_first_of(delimiter, start);
            if (end != string::npos){
                output.push_back(input.substr(start, end - start));
            }else{
                output.push_back(input.substr(start));
            }
        }
    }
    return output;
}
