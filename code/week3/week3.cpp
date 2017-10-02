// make sure to compile with g++
// to compile do
//      g++ -o week3.out -std=c++14 *.cpp
// to run do
//      ./week3.out

#include "week3.h"

int main() {
     using namespace std;

     // ******************** ARRAYS *********************
     // array docs http://en.cppreference.com/w/cpp/container/array

     // initialization
     // array<TYPE, SIZE> name = {elements}
     array<int, 3> int_array = {1, 2, 3};
     array<string, 3> string_array = {"arrays", "are", "cool"};

     // You can get the number of elements in a std::array by using .size()
     cout << "size of the int_arrray is = " << int_array.size() << endl;

     // remember arrays index 0,1,2,...
     cout << "printing int_array" << endl;
     for(int i = 0; i < int_array.size(); i++) {
         // .at(index) returns the element at that index with bounds checking at runtime
         cout << "string_array at index = " << i << " the value is = " << string_array.at(i) << endl;

         // [] operator gets the value at the index without bounds checking
         cout << "string_array at index = " << i << " the value is = " << string_array[i] << endl;
     }
     cout << "\n\n";

     // convenience methods exist for getting the front and back elements
     cout << "int_array front = " << int_array.front() << endl;
     cout << "int_array back = " << int_array.back() << endl;

     // .empty returns true if the array is empty and false otherwise
     if(int_array.empty()) {
         cout << "int_array is empty" << endl;
     } else {
         cout << "int_array is not empty" << endl;
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
     // increasing the size appends the value given
     robo_vector.resize(11, 'a');
     cout << "size is now = " << robo_vector.size() << endl;
     cout << "appended value = " << robo_vector.back() << endl;

     // decreasing the size removes up to the element
     robo_vector.resize(3);
     cout << "size is now = " << robo_vector.size() << endl;

     // capacity returns the number of elements that can be stored in the current
     // backing array without changing its size
     cout << "robo_vector has a capacity of " << robo_vector.capacity() << " and a size of " <<
        robo_vector.size() << endl;

     // reserve is NOT equivalent to resize
     // DO NOT USE reserve
     // reserve changes the size of the static backing array that is used to store the data.
     // if resize is called on a size larger it increases the capacity of the array
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
     // http://www.cplusplus.com/reference/iterator/iterator/?kw=iterator
     // iterators objects that lets you access objects in a container
     // they point to a location in a data structure and can return the data at
     // that location

     cout << "\n\n";

     // .begin gives an interator that starts at index 0 and increments up
     // COLLECTION_TYPE<TYPE>::iterator name = vector.method_to_get_iterator();
     vector<char>::iterator begin_it = robo_vector.begin();

     // .end gives the iterator pointing to the last element
     vector<char>::iterator end_it = robo_vector.end();
     cout << "iterate forwards while loop" << endl;
     while(begin_it != end_it) {
         // use the * operator to get the value the itertor is pointing to
         cout << "iterator is currently pointing to " << *begin_it << endl;
         // use the ++ or -- operator to move forward or backwards an element respectively
         begin_it++;
     }

     // iterators are great for for loops
     cout << "iterate forwards for loop" << endl;
     for(vector<char>::iterator it = robo_vector.begin(); it != robo_vector.end(); it++) {
         cout << "iterator is currently pointing to " << *it << endl;
     }

     // using rend and rbegin you can get an interator that will go in the opposite direction
     // this will iterate backwards
     cout << "iterate backwards while loop" << endl;
     for(vector<char>::reverse_iterator it = robo_vector.rbegin(); it != robo_vector.rend(); it++) {
         cout << "iterator is currently pointing to " << *it << endl;
     }

     // You can use an iterator to insert into a vector
     robo_vector.insert(robo_vector.begin(), 'R');
     cout << "the first index is now = " << robo_vector.front() << endl;
     for(vector<char>::iterator it = robo_vector.begin(); it != robo_vector.end(); it++) {
         cout << *it;
     }
     cout << "\n\n";

     // iterator work on the std::arrays also
     for(array<string, 3>::iterator it = string_array.begin(); it != string_array.end(); it++) {
         cout << *it << " ";
     }
     cout << "\n";

     // ******************** ALGORITHMS *********************
     cout << "\n\nalgorithms" << endl;
     // algorithms is a c++ library that contains lots of commonly used functions
     // use these whenever possible since they are often more optimized than
     // something you will write and they have been extensively tested.
     // algorithms uses interators as an abstraction layer
     // http://www.cplusplus.com/reference/algorithm/
     vector<int> long_list = {6,9,5,6,5,7,6,1,10,5,1,6,9,4,1,5,8,9,0,2,0,1,7,3,6,4};

     // count returns the number
     // http://www.cplusplus.com/reference/algorithm/count/
     cout << "the number 5 occurs " << count(long_list.begin(), long_list.end(), 5) << " times in the list" << endl;

     // sort places the elements in a specified order
     // http://www.cplusplus.com/reference/algorithm/sort/
     sort(long_list.begin(), long_list.end());
     cout << "Printing sorted list" << endl;
     for(std::vector<int>::iterator it = long_list.begin(); it != long_list.end(); it++) {
         std::cout << *it << ",";
     }
     cout << "\n\n";

     // finds the first instance of 6 in the container
     // http://www.cplusplus.com/reference/algorithm/find/
     vector<int>::iterator find_iter = find(long_list.begin(), long_list.end(), 6);
     cout << "find 6 = " << *find_iter << endl;

     // returns the second argument if nothing is found
     vector<int>::iterator find_iter_dne = find(long_list.begin(), long_list.end(), 100);
     cout << "find 100 = " << *find_iter_dne << " long_list.end() = " << *long_list.end() << endl;


     vector<int> new_int_vector (4);

     // copies elements from one container to another
     // http://www.cplusplus.com/reference/algorithm/copy/
     copy(find_iter, find_iter + 4, new_int_vector.begin());
     for(std::vector<int>::iterator it = new_int_vector.begin(); it != new_int_vector.end(); it++) {
         std::cout << *it << ",";
     }
     cout << "\n\n";

     // fills in copies of the given element into the container
     // http://www.cplusplus.com/reference/algorithm/fill/
     fill(new_int_vector.begin() + 1, new_int_vector.end(), 1);
     for(std::vector<int>::iterator it = new_int_vector.begin(); it != new_int_vector.end(); it++) {
         std::cout << *it << ",";
     }
     cout << "\n\n";

     // accumulate sums all of the values in the given range
     // http://www.cplusplus.com/reference/numeric/accumulate/
     int summation = 0;
     summation = accumulate(long_list.begin(), long_list.end(), summation);
     cout << "sum of long_list = " << summation << endl;

     vector<int> iota_vector (20);

     // iota is a convenience to create a collection of increasing values
     // http://www.cplusplus.com/reference/numeric/iota/?kw=iota
     cout << "\nprinting iota vector" << endl;
     iota(iota_vector.begin(), iota_vector.end(), 0);
     for(std::vector<int>::iterator it = iota_vector.begin(); it != iota_vector.end(); it++) {
         std::cout << *it << ",";
     }
     cout << "\n\n";


     return 0;
}
