#include <iostream>
#include <vector>
#include <memory>

// assigns the value 5 to the passed in variable (will fail)
void no_ptr_method(int num);
// assign the value 5 to the passed in variable
void ptr_method(int* numPtr);
// a method to show the local variable is allocated on the stack
int* stack_method();
// a method to show that a heap variable will persist
int* heap_method();
// a method to show that the static variable will persist
int* static_method();

// a helper struct to print out when it is created and deleted
struct object {
    object() { std::cout << "creating object\n";  } // called when created
    ~object() { std::cout << "deleting object\n";  } // called when destroyed
};

// a helper struct to show why static cast is useful
struct A {
    // a 4 byte member
    int member1;
};

// a helper struct to show why static cast is useful
struct B {
    // 4 1 byte members
    char member1;
    char member2;
    char member3;
    char member4;
};
