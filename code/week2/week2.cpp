// make sure to compile with g++
// to compile do
//      g++ -o week2.out week2.cpp
// to run do
//      ./week2.out

// This is including a header file for the standard library, strings, and input/output streaming
// stdlib.h contains a lot of useful function that do many things like parsing primimitives from strings and system calls 
#include <stdlib.h>
// string.h contains a lot of functions that will be covered this week ex). strncmp, strcat, ...
#include <string.h>
// iostream contains the functions cout, cin, cerr, and clog all fucntions taht deal with IO (input and output)
#include <iostream>

int main() {
    //TODO:
    //Sting manipulation examples with the std lib
    //Write a function and call it in main

    // This means that this code uses the std namespace, meaning all of the functions in the std namespace
    // do not require a qualifier. Generally using namespaces is considered bad practice especially in large
    // projects since it prevents shadowing (where I think I using do_something fucntion from library A but
    // I am actually using do_something function from library B) and imrpoving readaility of the code.
    // decent stack overflow article https://stackoverflow.com/questions/1452721/why-is-using-namespace-std-considered-bad-practice
    using namespace std;

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
    

    string str4 = "robojackets is lame";
    
    return 0;
}
