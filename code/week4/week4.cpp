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
    * OPERATOR    |               MEANING                    *
    * ------------------------------------------------------ *
    *     TYPE*   | create a pointer of TYPE                 *
    *     *VAR    | get the value pointed to by the pointer  *
    *     &VAR    | gets the address of the current value    *
    *********************************************************/
    int num = 0;

    cout << "the number start as " << num << endl;

	// & is used to get address
	cout << "The address of num is " << &num << endl;

	// now numPtr points to the memory location where 0 is stored
	int* num_ptr = &num;

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

	// to reference the underlying vector you should use the -> operator
	// it is just a short hand for (*())
	vector_a_ptr->insert(vector_a_ptr->begin(), "too");
	// see why -> exists, Thanks Dennis Ritchie for having an eye for sytle
	(*(vector_a_ptr)).insert((*(vector_a_ptr)).begin(), "small");

	for(vector<string>::iterator it = vector_a.begin(); it != vector_a.end(); it++) {
	    cout << *it << endl;
	}

	// you can even have a data structure of pointer to data structures
	vector<vector<string>*> vec_a_ptr_vec {};
	vec_a_ptr_vec.insert(vec_a_ptr_vec.begin(), vector_a_ptr);
	cout << "the first element = " << vec_a_ptr_vec.at(0)->at(0) << endl;

	// This can go in indefinitely
	// but at some point it just becomes absurd
	// if your code looks like a->b->c->d->... try a different approach

	/****************** unique_ptr *****************************/
    cout << "\n\n";
	// the problem with the above code is memory leakage...
	// unlike other languages like java and python c++ does not always handle garabage collection
	// If I initialize some data with a pointer then lose that pointer by assigning its
	// value to something else we have a YUGE problem. It is difficut to always remember to
	// destruct all objects. Thankfully smart pointer were invented

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
    cout << "\n\n";

    char c = 'c';
    int *p = (int*) &c;
    cout << "printing out c as an integer " << *p << endl;
    double *d = (double*) &c;
    cout << "printing out c as a double " << *d << endl;

    // the following line will not compile
    //int *q = static_cast<int*>(&c);

    // static_cast http://en.cppreference.com/w/cpp/language/static_cast
    // static_cast<TYPE_TO_CAST_TO>(VAR);
    double my_double = 1579.78;
    int my_int = static_cast<int>(my_double);
    cout << "my_double: " << my_double << " my_int: " << my_int << endl;

    // dynamic cast
    // http://en.cppreference.com/w/cpp/language/dynamic_cast




}
