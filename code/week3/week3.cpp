#include "week3.h"

int main() {
     using namespace std;

     // ******************** ARRAYS *********************
     // array docs http://en.cppreference.com/w/cpp/container/array

     // initialization
     // array<TYPE, SIZE> name = {elements}
     array<int, 3> int_array = {1, 2, 3};
     array<string, 3> string_array = {"arrays", "are", "cool"};

     // You can get the number fo elemnts in std::array by using .size()
     cout << "size of the array is = " << int_array.size() << endl;

     // remember arrays index 0,1,2,...
     cout << "printing int_array" << endl;
     for(int i = 0; i < int_array.size(); i++) {
         // .at(index) returns the element at that index with bounds checking at runtime
         cout << "int_array at index = " << i << " the value is = " << string_array.at(i) << endl;
         cout << "string_array at index = " << i << " the value is = " << string_array.at(i) << endl;

         // [] operator gets the value at the index without bounds checking
         cout << "int_array at index = " << i << " the value is = " << string_array[i] << endl;
         cout << "string_array at index = " << i << " the value is = " << string_array.at(i) << endl;
     }
     cout << "\n\n";

     // convenience methods exist for getting the front and back elements
     cout << "front = " << int_array.front() << endl;
     cout << "back = " << int_array.back() << endl;

     // .empty returns true if the array is empty and false otherwise
     if(int_array.empty()) {
         cout << "int_array is not empty" << endl;
     } else {
         cout << "int_array is empty" << endl;
     }

     // ******************** VECTORS *********************
     // vector docs http://en.cppreference.com/w/cpp/container/vector
     cout << "\n\n";

     //initialization
     // vector<TYPE> name = {values ...}
     vector<char> robo_vector;

     // You can add elements to a vector
     // push back appends the value to the back of the list
     robo_vector.push_back('o');

     // size returns the size
     cout << "size of robo_vector = " << robo_vector.size() << endl;

     // empty determines if the size is null
     if(!robo_vector.empty()) {
         robo_vector.push_back('j');
     }

     // elements can be accessed using at with bounds checking
     cout << "robo_vector at index 1 is = " << robo_vector.at(1) << endl;

     // elements can be accessed using [] without bounds checking
     cout << "robo_vector at index 0 is = " << robo_vector[0] << endl;

     // front and back also exist
     if(robo_vector.size() > 1 && robo_vector.front() != robo_vector.back()) {
         robo_vector.push_back('a');
     }

     // the last element can be removed using pop_back
     robo_vector.pop_back();
     cout << "size is now = " << robo_vector.size() << endl;

     // resize can be used to change the size of the vector
     // increasing the size appends the value givent
     robo_vector.resize(11, 'a');
     cout << "size is now = " << robo_vector.size() << endl;
     cout << "appended value = " << robo_vector.back() << endl;

     // decreasing the size removes up to the element
     robo_vector.resize(3);
     cout << "size is now = " << robo_vector.size() << endl;

     // capacity returns the number of elements that can be stored in the current
     // backeing array without changing its size
     cout << "robo_vector has a capacity of " << robo_vector.capacity() << " and a size of " <<
        robo_vector.size() << endl;

     // reserve is NOT equivalent to resize
     // DO NOT USE reserve
     // reserve changes the size of the static backing array that is used to store the data.
     // if resize is called on a size larger it incerases the capacity of the array
     robo_vector.reserve(30);
     cout << "after reserve 30 robo_vector has a capacity of " << robo_vector.capacity() <<
        " and a size of " << robo_vector.size() << endl;

     // if resize is called on a size smaller it does nothing
     robo_vector.reserve(10);
     cout << "after reserve 10 robo_vector has a capacity of " << robo_vector.capacity() <<
        " and a size of " << robo_vector.size() << endl;

     // the array can be shrunk down to the minimum size to fit its contents by
     // calling the shrink_to_fit method
     robo_vector.shrink_to_fit();
     cout << "after reserve 10 robo_vector has a capacity of " << robo_vector.capacity() <<
        " and a size of " << robo_vector.size() << endl;

     // ******************** ITERATORS *********************
     // iterators are ...
     // they point to a location in a data structure and can return the data at
     // that location

     // .begin gives an interator that starts at index 0 and increments up
     // TODO explain init here
     vector<char>::iterator begin_it = robo_vector.begin();

     // .end gives the iterator pointing to the last element
     vector<char>::iterator end_it = robo_vector.end();
     while(begin_it != end_it) {
         // use the * operator to get the value the itertor is pointing to
         cout << "iterator is currently pointing to " << *begin_it << endl;
         // use the ++ or -- operator to move forward or backwards an element respectively
         begin_it++;
     }

     // iterators are great for for loops
     for(vector<char>::iterator it = robo_vector.begin(); it != robo_vector.end(); it++) {
         cout << "iterator is currently pointing to " << *it << endl;
     }





     return 0;
}
