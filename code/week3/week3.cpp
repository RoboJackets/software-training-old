#include "week3.h"

int main() {
     using namespace std;

     // array docs http://en.cppreference.com/w/cpp/container/array

     // initialization
     // array<TYPE, SIZE> name = {elements}
     array<int, 3> int_array = {1, 2, 3};

     // You can get the number fo elemnts in std::array by using .size()
     cout << "size of the array is = " << int_array.size() << endl;

     // remember arrays index 0,1,2,...
     cout << "printing int_array" << endl;
     for(int i = 0; i < int_array.size(); i++) {
         // .at(index) returns the element at that index with bounds checking at runtime
         cout << "at index = " << i << " the value is = " << int_array.at(i) << endl;

         // [] operator gets the value at the index without bounds checking
         cout << "at index = " << i << " the value is = " << int_array[i] << endl;
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




     return 0;
}
