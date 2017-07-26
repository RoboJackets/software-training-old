#include <iostream>
#include <vector>

// assigns the value 5 to the passed in variable (will fail)
void no_ptr_method(int num);
// assign the value 5 to the passed in variable
void ptr_method(int* numPtr);

struct object {
    object() { std::cout << "creating object\n";  } // called when created
    ~object() { std::cout << "deleting object\n";  } // called when destroyed
};
