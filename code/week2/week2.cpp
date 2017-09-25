// make sure to compile with g++
// to compile do
//      g++ -o week2.out -std=c++14 *.cpp
// to run do
//      ./week2.out

// This line includes the week2 header file defined in this directory
#include "week2.h"

int main() {
    // This means that this code uses the std namespace. Now, all of the functions in the std namespace
    // do not require a qualifier. Generally using namespaces is considered bad practice especially in large
    // projects since it can lead to shadowing (where I think I am using do_something function from library A but
    // I am actually using do_something function from library B). Cutting down on extra declarations
    // improves the readability of the code.
    // decent stack overflow article: https://stackoverflow.com/q/1452721
    using namespace std;

    cout << "\n***************** FUNCTIONS **************\n\n";

    cout << "2 + 2 called \n"  << add(2, 2)  << " returned\n\n";
    cout << "2.0 + 2.0 called \n" << add(2.0, 2.0) << " returned\n\n";
    cout << "2 + 2 + 2 called \n" << add(2, 2, 2) << " returned\n\n";
    cout << "2 - 4 called \n" << subtract(2, 4) << " returned\n\n";
    cout << "2 * 2 called \n" << multiply(2, 2) <<  " returned\n\n";
    cout << "6 / 2 called \n" << divide(6, 2) <<  " returned\n\n";

    cout << "\n***************** STRINGS **************\n\n\n";

    // initializing a string
    string str1 = "test";
    string str2 = "test";

    cout << str1.length() << endl;
    cout << str2.size() << endl;

    // maximum length a size can reach due to system limitation. DO NOT try to reach this size.
    cout << str1.max_size() << endl;

    // http://www.cplusplus.com/reference/string/string/resize/
    // resize alters the size of the string
    str1.resize(3);
    cout << str1 << endl;
    str2.resize(10, 'a');
    cout << str2 << endl;

    // http://www.cplusplus.com/reference/string/string/empty/
    // empty checks if the string is empty ""
    string str3 = "";
    if(str3.empty()) {
        cout << "str3 is empty" << endl;
    } else {
        cout << "str3 is NOT empty" << endl;
    }

    // http://www.cplusplus.com/reference/string/string/clear/
    // clear replaces the string with the null string ""
    str1.clear();
    if(str1.empty()) {
        cout << "str1 is empty" << endl;
    } else {
        cout << "str1 is NOT empty" << endl;
    }

    // http://www.cplusplus.com/reference/string/string/append/
    // append adds the second string to the end of the string that it was called on
    str1.append("he was number one");
    cout << str1 << endl;
    str1.append(" 1");
    cout << str1 << endl;

    string lame_string = "robojackets is lame";

    // http://www.cplusplus.com/reference/string/string/find/
    // find locates the index in the string where the substring begins
    int position = lame_string.find("lame");
    cout << "lame is at index " << position << endl;

    // http://www.cplusplus.com/reference/string/string/substr/
    // substring returns a portion of the string as another string
    string sub_str = lame_string.substr(position, lame_string.size());
    cout << "substring = " << sub_str << endl;

    // http://www.cplusplus.com/reference/string/string/replace/
    // replace takes the string it is called on and replaces the given locations with
    // another string
    lame_string.replace(position, lame_string.size(), "cool");
    cout << lame_string << endl;

    // http://www.cplusplus.com/reference/string/string/operator+/
    // the + operator concatenates two string together into one
    string str4 = "Is this string only partia";
    string str5 = "lly complete?\n";
    string str6 = "not anymore";
    string str7 = str4  + str5 + str6;
    cout << str7 << endl;
    cout << "str4 length = " << str4.size() << endl;
    cout << "str5 length = " << str5.size() << endl;
    cout << "str6 length = " << str6.size() << endl;
    cout << "str7 length = " << str7.size() << endl;

    // converting primitives to strings
    cout << "\n\n" << endl;

    // http://www.cplusplus.com/reference/string/stoi/
    // converting string to integer
    int string_to_int = stoi("5");
    cout << string_to_int << endl;

    // http://www.cplusplus.com/reference/string/stod/
    // converting string to double
    double string_to_double = stod("5.21385");
    cout << string_to_double << endl;

    // There are many more methods to parse primitives form strings

    // http://www.cplusplus.com/reference/string/to_string/
    // converting primitves to strings
    string str8 = to_string(string_to_int);
    cout << str8 << endl;

    string str9 = to_string(string_to_double);
    cout << str9 << endl;

    return 0;
}
