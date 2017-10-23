/*
 * to compile
 * g++ -std=c++14 -o week4.out week4.cpp
*/

#include "week4.h"

void no_ptr_method(int num) {
    num = 5;
}

// the * operator after a type means that we are creating a pointer
void ptr_method(int* numPtr) {
  // sets the memory location pointed to by num
  *numPtr = 5;
}

int main() {
    using namespace std;
	/****************** POINTERS ****************************/
    /*********************************************************
    *   OPERATOR  |               MEANING                    *
    * ------------------------------------------------------ *
    *     TYPE*   | creates a pointer of TYPE                *
    *     *VAR    | gets the value pointed to by the pointer *
    *     &VAR    | gets the address of the current value    *
    *********************************************************/
    int num = 0;

    cout << "the number starts as " << num << endl;

	cout << "The address of num is " << &num << endl;

	// now numPtr points to the memory location where 0 is stored
	int* num_ptr = &num;

    cout << "the value of the pointer variable is = " << num_ptr << endl;

	// dereferencing the pointer
	cout << "value pointed to by num_ptr is " << *num_ptr << endl;

	// you can change the value of num by using *
	*num_ptr = 1000;
	cout << "after *numPtr = 1000 the value of num is " << num << endl;

	// pass by value
    no_ptr_method(num);
    cout << "after calling no_ptr_method " << num << endl;

    // pass by reference
    ptr_method(&num);
    cout << "after calling ptr_method " << num << endl;

	// you can have pointers to data structures like vector
    vector<string> vector_a = {"I", "seats", "plane"};
	vector<string>* vector_a_ptr = &vector_a;

    /**********************************************************************
    *   OPERATOR  |               MEANING                                 *
    * ------------------------------------------------------------------- *
    *     VAR.    | accesses a class member given the actual object       *
    *     VAR->   | accesses a class member given a pointer to the object *
    ***********************************************************************/
	// to reference the underlying vector you should use the -> operator
	// it is just a short hand for (*()).
	vector_a_ptr->insert(vector_a_ptr->begin(), "too");
	// see why -> exists, Thanks Dennis Ritchie for having an eye for style
	(*(vector_a_ptr)).insert((*(vector_a_ptr)).begin(), "small");

	for(vector<string>::iterator it = vector_a.begin(); it != vector_a.end(); it++) {
	    cout << *it << endl;
	}
	// you can even have a data structure of pointer to data structures
	vector<vector<string>*> vec_a_ptr_vec {};
	vec_a_ptr_vec.insert(vec_a_ptr_vec.begin(), vector_a_ptr);
	cout << "the first element = " << vec_a_ptr_vec.at(0)->at(0) << endl;

	// This can go in indefinitely.
	// but at some point it just becomes absurd
	// if your code looks like a->b->c->d->... try a different approach

    /****************** MEMORY STRUCTURE ****************************/
    cout << "\n\n";
    int* stack_value = stack_method();
    int* heap_value = heap_method();
    int* static_value = static_method();
    /* see how the stack value changes. That is because the stack frame from the
    * stack_method method call was removed. Therefore the program has a pointer
    * to a place in memory that it no longer actively controls therefore it
    * can change at any time.
    *
    * The program still has ownership of the heap_value
    * memory location since delete has not been called on it.
    *
    * The static value is allocated in the static area of the program. This memory
    * is allocated at compile time and goes away once the program ends.
    */
    cout << "stack value should be 0 is " << *stack_value << endl;
    cout << "heap value should be 3 is " << *heap_value << endl;
    cout << "static value should be 10 is " << *static_value << endl;

    // This line tells the computer to give up control of that space on the heap.
    // Meaning that now the memory can be reallocated at any point.
    delete heap_value;

    // how can we make the program handle deallocating heap variables for us
    // smart pointers

	/****************** unique_ptr *****************************/
    cout << "\n\n";
	// the problem with the above code is memory leakage...
	// unlike other languages like java and python c++ does not always handle garabage collection
	// If I initialize some data on the heap then lose that pointer by assigning its
	// value to something else we have a YUGE problem. It is difficut to always remember to
	// free all memory. Thankfully smart pointers were invented.

    // object is a struct I have defined in the header file
    { // This defines a region of scope
        // unqiue_ptr<TYPE> NAME = make_unique<TYPE>();
        unique_ptr<object> unqiue_vec_ptr = make_unique<object>();
        cout << "ending scope" << endl;
    }

    cout << "out of scope" << endl;

    {
        cout << "creating first shared pointer" << endl;
        // shared_ptr<TYPE> NAME = make_shared<TYPE>();
        shared_ptr<object> shared_ptr1 = make_shared<object>();
        cout << "number of pointers to object = " << shared_ptr1.use_count() << endl;
        {
            cout << "creating second shared pointer" << endl;
            shared_ptr<object> shared_ptr2 = shared_ptr1;
            cout << "number of pointers to object = " << shared_ptr2.use_count() << endl;
            cout << "scope ending for second shared pointer" << endl;
        }
        cout << "scope ending for first shared pointer" << endl;
    }

    /****************** casting *****************************/
    // EXAMPLE 1
    cout << "\n\n";

    char c = 'c';
    int *p = (int*) &c;
    cout << "printing out c as an integer " << *p << endl;
    double *d = (double*) &c;
    cout << "printing out c as a double " << *d << endl;

    // static_cast http://en.cppreference.com/w/cpp/language/static_cast
    // static_cast<TYPE_TO_CAST_TO>(VAR);
    // the following line will not compile since it is not a legal cast
    // int *q = static_cast<int*>(&c);

    double my_double = 1579.78;
    int my_int = static_cast<int>(my_double);
    cout << "my_double: " << my_double << " my_int: " << my_int << endl;

    // EXAMPLE 2
    A *a = new A();
    a->member1 = 1716543349;
    B *b = (B*) a;
    // here the 4 bytes of the integer are interpreted as 4 chars packed together
    cout << "a member1 = " << a->member1 << endl;
    cout << "b member1 = " << b->member1 << endl;
    cout << "b member2 = " << b->member2 << endl;
    cout << "b member3 = " << b->member3 << endl;
    cout << "b member4 = " << b->member4 << endl;

    // will not compile since it is not a legal cast
    // static_cast<B*>(a);

    // dynamic cast
    // http://en.cppreference.com/w/cpp/language/dynamic_cast
    // will also fail to compile since it is not a legal cast
    // dynamic_cast<B*>(a);
    delete a;
}

int* stack_method() {
    // this is allocated on the stack
    int var = 0;
    // returning a pointer to a variable on the stack should never be done
    // hence the compiler warning
    return &var;
}

int* heap_method() {
    // this is allocated on the heap
    int* var = new int;
    *var = 3;
    return var;
}

int* static_method() {
    // This allocates memory in the static area
    static int var = 10;
    return &var;
}
