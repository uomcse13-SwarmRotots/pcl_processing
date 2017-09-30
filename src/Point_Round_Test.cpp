#include <string> 
#include <boost/lexical_cast.hpp>

using namespace std;

float get_rounded_point(float cordinate){
    float mod_cordinate = 1000 * cordinate;
    int mod_cordinate_int = (int)mod_cordinate;
    int final_int;
    std::string mod_cordinate_str = boost::lexical_cast<std::string>(mod_cordinate_int);
    string last_characters = mod_cordinate_str.substr(mod_cordinate_str.length() - 2);
    try {
        int last_int = boost::lexical_cast<int>(last_characters);
        
        if(last_int == 25 | last_int == 75){
            final_int = mod_cordinate_int;
        }else if(last_int > 25 & last_int < 50){
            final_int = mod_cordinate_int - (last_int - 25);
        }else if((last_int >= 50 & last_int < 75)){
            final_int = mod_cordinate_int + (75 - last_int);
        }else if(last_int > 75 & last_int < 100){
            final_int = mod_cordinate_int - (last_int - 75);
        }else if(last_int >= 0 & last_int < 25){
            final_int = mod_cordinate_int + (25 - last_int);
        }

        float final_number = (float)final_int/1000;

        // printf("%f\n",final_number);
    } catch( boost::bad_lexical_cast const& ) {
        // std::cout << "Error: input string was not valid" << std::endl;
    }
    return final_number;
}

int main(){
    get_rounded_point(1.330000);
    get_rounded_point(1.382000);
    get_rounded_point(1.302000);
    get_rounded_point(1.354000);
    get_rounded_point(1.325000);
    return 0;
}
