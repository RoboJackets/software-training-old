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

	// This can go in indefinitely TODO verify
	// but at some point it just becomes absurd
	// if your code looks like a->b->c->d->... try a different approach

	/****************** unique_ptr *****************************/
	// the problem with the above code is memory leakage...
	// unlike other languages like java and python c++ does not always handle garabage collection
	// If I initialize some data with a pointer then lose that pointer by assigning its
	// value to something else we have a YUGE problem. It is difficut to always remember to
	// destruct all objects. Thankfully smart pointer were invented

	

    
    
}
