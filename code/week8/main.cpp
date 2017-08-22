#include <iostream>
#include <algorithm>
#include <vector>

int addTwo(int num) {
    return num + 2;
}

int addThree(int num) {
    return num + 3;
}

int add(int num1, int num2) {
    return num1 + num2;
}

void printIntVec(std::vector<int> vec) {
    std::cout << "printing vector " << std::endl;
    for(std::vector<int>::iterator it = vec.begin();it != vec.end(); it++) {
        std::cout << *it << ", ";
    }
    std::cout << "\n\n";
}

void printInt(int i) {
    std::cout << "looking at " << i << std::endl;
}

class Parent {
public:
    virtual void do_something() {std::cout << "PARENT" << std::endl;}
};

class Child : public Parent {
public:
    virtual void do_something() {std::cout << "CHILD" << std::endl;}
};

void callDoSomething(Parent p) {
    p.do_something();
}

void callDoSomethingRef(Child c) {
    c.do_something();
}

int main() {
    using namespace std;

    /******** FUNCTION POINTERS ********/

    // functions are just lines of code stored somewhere in memory
    // that means that you can get the address of a function just like anything else using the & operator
    // RETURN_TYPE (*NAME) (PARAM_TYPES);
    int (*func_ptr) (int) = &addTwo;
    int (*name) (int, int) = &add;

    // although a function can be implicitly converted into a pointer by the compiler
    // so the & is not required
    // auto is usually helpful here
    auto func_ptr2 = addThree;

    // to use add ()
    cout << "value of addTwo(2) = " << func_ptr(2) << endl;

    /**************** IGNORE IF YOU DO NOT UNDERSTAND ****************/
    // since a function pointer is also a pointer you can dereference it but this is not necessary
    // in fact when we dereference the pointer we get the function
    // which as mentioned above can be implicitly converted into a pointer by the compiler
    // this means that we can add an arbitrary number of * and this code still works

    cout << "calling addTwo(2) = " << (*************** func_ptr)(2) << endl;

    /**************** EXAMPLE CODE RESUMED ***************************/

    /******** LAMBDAS ********/
    cout << "\n\n";

    vector<int> example_vec = {1,2,3,4,5};
    // for_each does whatever function is given to all values will not change the values
    // http://www.cplusplus.com/reference/algorithm/for_each/?kw=for_each
    for_each(example_vec.begin(), example_vec.end(), printInt);

    cout << "\n\n";

    // using lambda
    // [] (TYPE NAME, ...) -> RETURN_TYPE {CODE};
    for_each(example_vec.begin(), example_vec.end(), [](int num) -> void {std::cout << "printing from lambda " << num << std::endl;});
    // transform is like for each but it replaces the value with what is returned
    // http://www.cplusplus.com/reference/algorithm/transform/?kw=transform
    transform(example_vec.begin(), example_vec.end(), example_vec.begin(), [](int num) -> int {return num + 2;});
    printIntVec(example_vec);

    // often the return type is implied and therefore can be left off
    // [] (TYPE NAME, ...) {CODE};
    transform(example_vec.begin(), example_vec.end(), example_vec.begin(), [](int num) {return num - 2;});
    printIntVec(example_vec);

    /******** OBJECT SLICING ********/
    // object slicing is when a child is assigned to its parent and therefore loses additional features
    Parent p = Parent();
    Child c = Child();
    cout << "parent calling callDoSomething" << endl;
    callDoSomething(p);
    cout << "child calling callDoSomething" << endl;
    callDoSomething(c);
    cout << "child calling callDoSomethingRef" << endl;
    callDoSomethingRef(c);

}
