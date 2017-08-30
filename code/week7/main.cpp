#include <iostream>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <list>
#include <stack>
#include <queue>
#include <memory>

#include "generic_class.h"

template<class T>
void print_data(T iterator_begin, T iterator_end, std::string name) {
    std::cout << "Printing " << name << std::endl;
    for(;iterator_begin != iterator_end; iterator_begin++) {
        std::cout << *iterator_begin << ", ";
    }
    std::cout << "\n\n";
}
/* with set<int> it would convert to something like this
void print_data(set<int>::iterator iterator_begin, set<int>::iterator iterator_end, std::string name) {
    std::cout << "Printing " << name << std::endl;
    for(;iterator_begin != iterator_end; iterator_begin++) {
        std::cout << *iterator_begin << ", ";
    }
    std::cout << "\n\n";
}
 */

template <class Foo>
Foo getMax(Foo a, Foo b) {
    if(a < b) {
        return b;
    } else {
        return a;
    }
}
/* when getMax<int> the method would look like
int getMax(int a, int b) {
    if(a < b) {
        return b;
    } else {
        return a;
    }
}
*/
/*
when getMax<double> the method would look like
double getMax(double a, double b) {
    if(a < b) {
        return b;
    } else {
        return a;
    }
}
*/

// you can have multiple templates in a function
template <class Foo, class Bar>
Bar getMin(Foo a, Bar b) {
    if(a < b) {
        return a;
    } else {
        return b;
    }
}



int main() {
    using namespace std;

    /*************** LIST ***************/
    // http://www.cplusplus.com/reference/list/list/
    // list<TYPE> name
    list<int> int_list = {1,2,3};
    print_data(int_list.begin(), int_list.end(), "int_list");

    cout << "the first element is = " << int_list.front() << endl;
    cout << "the last element is = " << int_list.back() << endl;

    /* pop removes an element and destructs it
     * push adds an element
     * front does the action to the front
     * back does the action to the back
     */
    int_list.pop_front();
    int_list.push_back(53);
    int_list.push_front(15);
    int_list.pop_back();
    print_data(int_list.begin(), int_list.end(), "int_list");

    /*************** QUEUE ***************/
    // http://www.cplusplus.com/reference/queue/
    queue<char> queue;
    queue.push('r');
    queue.push('o');
    queue.push('b');
    queue.push('o');
    queue.push('j');

    while(!queue.empty()) {
        cout << queue.front();
        queue.pop();
    }

    /*************** STACK ***************/
    // http://www.cplusplus.com/reference/stack/stack/
    stack<char> stack;
    stack.push('s');
    stack.push('t');
    stack.push('e');
    stack.push('k');
    stack.push('c');
    stack.push('a');

    while(!stack.empty()) {
        cout << stack.top();
        stack.pop();
    }
    cout << "\n\n";

    /*************** SETS ***************/
    // http://www.cplusplus.com/reference/set/set/?kw=set
    // init
    // set<TYPE> name
    set<string> ordered_set {"1", "5", "6"};

    // can insert elements into set
    ordered_set.insert("2");
    print_data(ordered_set.begin(), ordered_set.end(), "ordered_set first time");

    // If the an inserted element is determined to be the same it will not be inserted
    // and a iterator to the duplicate element in the list is returned
    ordered_set.insert("1");
    print_data(ordered_set.begin(), ordered_set.end(), "ordered_set second time");

    // elements can also be erased
    ordered_set.erase("1");
    print_data(ordered_set.begin(), ordered_set.end(), "ordered_set third time");

    /*************** UNORDERED SETS ***************/
    // http://www.cplusplus.com/reference/unordered_set/unordered_set/
    // init
    // set<TYPE> name
    unordered_set<string> unordered_set {"20","19","18","1","16"};

    // can insert elements into set
    unordered_set.insert("21");
    // order is not maintained here
    print_data(unordered_set.begin(), unordered_set.end(), "unordered_set");

    /*************** MAPS ***************/
    // http://www.cplusplus.com/reference/map/map/?kw=map
    // uses set in background
    cout << "\nmap\n";
    // init
    // map<KEY_TYPE, VALUE_TYPE>
    map<char, int> char_map;

    // inserts a pair (key, value) into the map at iterator
    char_map.insert(char_map.begin(), pair<char, int>('a', 5));
    char_map.insert(char_map.begin(), pair<char, int>('b', 10));
    char_map.insert(char_map.begin(), pair<char, int>('c', 25));

    // at gets the value at the given key
    cout << "at char 'a' value = " << char_map.at('a') << endl;

    // iterating over a map
    for(map<char,int>::iterator it = char_map.begin(); it != char_map.end(); it++) {
        cout << "key: " << it->first << " value: " << it->second << endl;
    }
    cout << "\n\n";

    /*************** UNORDERED MAPS ***************/
    // http://www.cplusplus.com/reference/unordered_map/unordered_map/
    // unordered maps have faster access time than map
    // uses unordered set in background
    unordered_map<int, string> string_map;

    string_map.insert(pair<int, string>(53, "string here"));
    string_map.insert(pair<int, string>(8669, "another string"));
    string_map.insert(pair<int, string>(753, "this is getting really long"));

    cout << "at int 53 value = " << string_map.at(53) << endl;

    // iterating over a map
    for(unordered_map<int, string>::iterator it = string_map.begin(); it != string_map.end(); it++) {
        cout << "key: " << it->first << " value: " << it->second << endl;
    }

    /*************** TEMPLATES ***************/
    cout << "\n\n";
    cout << "the max of 7,8 is = " << getMax<int>(7, 8) << endl;
    cout << "the max of 7.0, 8.0 is = " << getMax<double>(7.0, 8.0) << endl;
    cout << "the min of 7, 8.0 is = " << getMin<int, double>(7, 8.0) << endl;

    cout << "\n\n";

    unique_ptr<Example<string>> example_str = make_unique<Example<string>>("storing strings here");
    unique_ptr<Example<int>> example2 = make_unique<Example<int>>(2);

    cout << "accessing the data in example_str, data: " << example_str->getVar() << endl;
    cout << "accessing the data in example2, data: " << example2->getVar() << endl;
}
