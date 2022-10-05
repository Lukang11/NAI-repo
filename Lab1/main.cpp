#include <iostream>
#include <cmath>
#include <functional>
#include <map>

int mod(int number, int division );
double add (double number,double number1);

int main(int argc , char **argv){
    using namespace std;


    map<string,int> xx;
    xx["a"] = 2;
    xx["sda"] = 8;

    cout << "w : " << xx[argv[1]] << endl;

    cout << add(argv[1],argv[2]) <<endl;
    map<string,function<double()>> my_function;

    my_function["add"] = add(argv[1],argv[2]);

    /*map<string,function<double()>> my_function;



    int result = mod(10,2);
    cout << result << endl;
    //stod()//string to double
    */
     return 0;
}

int mod(int number, int division ){
    return number % division;
}

double add (double number,double number1){
    return number + number1;
}