#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <cmath>

using namespace std;
using mojafunkcja_t = function<double(vector<double>)>;
using vector_t = vector<string>;
using function_t = map<string,mojafunkcja_t>;

void display(function_t func,vector_t arguments){
    auto parameter = func[arguments.at(1)];
    vector<double> value;

    for(int i = 2; i < arguments.max_size();i++){
        value.push_back(stod(arguments.at(i)));
    }

    cout<< "Result : " << parameter(value);

}

int main(int argc,char ** argv) {
    map<string,double> mapa;
    function_t function;

    function["add"] = [](vector<double> value) {return value[0] + value[1];};
    function["mod"] = [](vector<double> value) {return fmod(value[0], value[1]);};
    function["sin"] = [](vector<double> value) {return sin(value[0]);};

    try {
        vector<string> arguments(argv, argv + argc);
        display(function, arguments);

    }
    catch (std::out_of_range aor){

        cout << "Podaj argument. Dostepne to: ";
        for (auto [k, v] : function) cout << " " << k << endl;;


    }
    catch (std::bad_function_call bfc){
       cout << "Zła nazwa fukncji" << endl;
    }


    return 0;
}