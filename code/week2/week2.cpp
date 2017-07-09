// make sure to compile with g++
// to compile do
//      g++ -o week2.out week2.cpp operations.cpp
// to run do
//      ./week2.out

// This line includes the week2 header file defined in this directory
#include "week2.h"

int main() {
    //TODO:
    //Sting manipulation examples with the std lib
    //Write a function and call it in main

    // This means that this code uses the std namespace, meaning all of the functions in the std namespace
    // do not require a qualifier. Generally using namespaces is considered bad practice especially in large
    // projects since it can lead to shadowing (where I think I using do_something fucntion from library A but
    // I am actually using do_something function from library B) and imrpoving readaility of the code.
    // decent stack overflow article https://stackoverflow.com/questions/1452721/why-is-using-namespace-std-considered-bad-practice
    using namespace std;

    // ***************** FUNCTIONS **************

    cout << "2 + 2 = " << add(2, 2) << endl;
    cout << "2 - 4 = " << subtract(2, 4) << endl;
    cout << "2 * 2 = " << multiply(2, 2) << endl;
    cout << "6 % 2 = " << divide(6, 2) << endl;

    // ***************** STRINGS **************
    
    // initializing a string
    string str1 = "test";
    string str2 = "test";

    cout << str1.length() << endl;
    cout << str2.size() << endl;

    // maximum length a size can reach due to system limitation. DO NOT try to reach this size.
    cout << str1.max_size() << endl;

    str1.resize(3);
    cout << str1 << endl;
    str2.resize(10, 'a');
    cout << str2 << endl;

    string str3 = "";
    if(str3.empty()) {
        cout << "str3 is empty" << endl;
    } else {
        cout << "str3 is NOT empty" << endl;
    }

    str1.clear();
    if(str1.empty()) {
        cout << "str1 is empty" << endl;
    } else {
        cout << "str1 is NOT empty" << endl;
    }

    str1.append("he was number one");
    cout << str1 << endl;
    str1.append(" 1");
    cout << str1 << endl;
    

    string lame_string = "robojackets is lame";

    int position = lame_string.find("lame");
    cout << "lame is at index " << position << endl;

    string sub_str = lame_string.substr(position, lame_string.size());
    cout << "substring = " << sub_str << endl;

    lame_string.replace(position, lame_string.size(), "cool");
    cout << lame_string << endl;

    string str4 = "Is this string only partia";
    string str5 = "lly complete?\n";
    string str6 = "not anymore";
    string str7 = str4  + str5 + str6;
    cout << str7 << endl;

    cout << "str4 length = " << str4.size() << endl;
    cout << "str5 length = " << str5.size() << endl;
    cout << "str6 length = " << str6.size() << endl;
    cout << "str7 length = " << str7.size() << endl;

    return 0;
}
