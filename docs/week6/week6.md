# What are we doing today?

-   Const Keyword
-   Initializer Lists
-   Header Guards
-   More STL
    -   Functors
    -   Lambdas
    -   Predicates


# Const Keyword

-   Can be used on both variables and member functions of objects
    -   Const Variable
        -   Makes the value *constant* as the name implies
        -   Must be initialized when declared
        -   This can also be applied to the arguments and return type of a function
    -   Const Member Function
        -   Makes the member function read-only
        -   It can only read and return values, it cannot change any of the members of the class


## Constant Variables

```c++
int main() {
    const int x; // This causes an error

    const int x = 0; // This is correct

    return 0;
}
```

[Example](https://ideone.com/4D2EoC)


## Constant Arguments

```c++
int constantArguments(const int x) {
    std::cout << x << std::endl; // This is fine
    x = 5; // This causes an error

    return x;
}
```


## Constant Member Functions

```c++
class Example {
public:
    int x;
    int GetX() const; //GetX is a constant member function
};
```

[Example](https://ideone.com/I854Gu)


## Why is this useful?

-   Good coding practice
    -   Makes it clear what should and should not change
    -   Prevents others from accidentally changing something
-   More Efficient
    -   The compiler can optimize aggresively around constant variables since they don't change


# Initializer Lists

<div class="NOTES">
Describe what the below code does. Want to give a basic understanding of what it does before trying to explain why they are useful

</div>

```c++
class Example {
public:
    int x;
    ClassA obj1;
    ClassB obj2;

    Example(int xValue, int arg1, int arg2) : x(xValue), obj1(arg2), obj2(arg1, arg2) {
```


## Benefits of Initializer Lists

-   Initializes the member variables before the object is constructed
    -   This allows you to have your member objects initialized before your constructor runs
    -   It also allows you to have const member variables since they would otherwise error when you try to declare them without a value


## Initiliazing Constant Member Variables

```c++
class Example {
public:
    const int x;
    Example() {
	x = 5; //This causes an error since you can't "change" the value of a const
    }
}
```

-   You can't set the value of a constant variable after it has been declared

<div class="NOTES">
Someone check me on that second statement

</div>


## Initiliazing Constant Member Variables pt. 2

```c++
class Example {
public:
    const int x;
    Example() : x(5);
}
```

-   This works because the initialization chain intializes the constant variable before the object is constructed so it runs before the value of the const is frozen
-   This is also preferred over initializing the constant in the declaration of the class (`const int x = 5`)


## Default Constructors with Custom Default Values

-   When a default constructor runs it calls the default constructors of all of the member variables
-   But sometimes you want your class to have different default values than the standard default constructors of its members


## The Inefficient way to set default values

```c++
class Example {
public:
    int x;
    ClassA object1;
    Example() {
	x = 5;
	object1.field1 = 1;
	object1.field2 = 2;
    }
}
```

-   This is inefficient because the default constructors of the objects are called first
    -   This doesn't really matter for a primitive data type like an int but it could have a larger impact when called for more complex classes


## The Efficient way to set default values

```c++
class Example {
    int x;
    ClassA object1;
    Example() : x(5), object1(1,2){}
}
```

-   This is more efficient because the member variables' constructors are called first so the default constructors are not also called
    -   This does assume that object1 has a constructor to set field1 and field 2 though


# Header Guards

<div class="NOTES">
Go to the next slide to illustrate the issue

</div>

```c++
//in file: "example.cpp"
class Example {
    int x;
}
```

```c++
//in file: "b.cpp"
#include "example.cpp"
class B {
    Example y;
}
```

```c++
//in file: "main.cpp"
#include "example.cpp"
#include "b.cpp"

int main() {
    Example obj1;
    B obj2;

    return 0;
}
```

-   Question: Why does this code cause an error?


## Header Guards pt. 2

<div class="NOTES">
Include statments aren't "run"&#x2026; anybody have a better term for this?

</div>

```c++
//in file: "main.cpp" after include statments are run

class Example {
    int x;
}

class Example {
    int x;
}

class B {
    Example y;
}

int main() {
    Example obj1;
    B obj2;

    return 0;
}
```


## Header Guards pt. 3

-   Example is defined twice and the compiler doesn't know which one to use
-   Header guards prevent header files (the .h files that we use to declare classes) from conflicting with themselves by only including the code once


## Header Guard Syntax

```c++
//in file: "example.h"
#ifndef EXAMPLE_H
#define EXAMPLE_H
class Example {
    int x;
}
#endif
```

-   `#` indicates a Preprocessor Directive. These alter the source code as it is passed into the compiler
    -   `#include` copies the code from the indicated source and pastes it where the include was called
    -   `#ifndef` checks if the given macro is not defined
    -   `#define` defines the given macro
    -   `#endif` closes the if statement started by #ifndef


## Header Guard Syntax pt. 2

-   Header guards work by only allowing the compiler to see the code inside the ifndef if the given macro is not yet defined. After the compiler has processed the .h file once the `EXAMPLE_H` macro is defined so the next time it attempts to include the file the code is hidden inside the if statement
-   Almost every compiler also supports the command `#pragma once` which does the same thing as the header guard although it is not technically part of the C++ standard


# Functors

-   A functor is basically an object with the operator() which allows it to be called like a function
-   This allows us to create a "function" that can store data or use more information than its input arguments would otherwise allow


## Functor Example

```c++
class increment {
public:
    int num;
    increment(int n) : num(n){}

    // This allows the functor to be called using () like a function
    int operator () (int numIn) const {
	return numIn + num;
    }
};

int main() {
    increment exampleFunctor(2);
    int x = 5;
    std::cout << exampleFunctor(5) << std::endl // Outputs 7
    return 0;
}
```


## Lambdas

-   A lambda expression basically writes a short inline function
-   Syntax:

```
[capture clause] (arguments) -> return-type
{
    Definition of method
}
```


## Syntax Breakdown

-   Most of the syntax works as it does in a function
-   Arguments: the list of input arguments (ex: int a, double b,&#x2026;)
-   Return-type: The type that the function returns (ex: bool)
    -   Usually the arrow and return-type can be ommited since the compiler can figure it out

```
[capture clause] (arguments) -> return-type
{
    Definition of method
}
```


## Capture Clauses

-   It is important to note that although the lambda is defined inline the code still accesses it through a normal function call
    -   This means that variables that were in the local scope before the lambda call are *not* in this scope
-   In order to access these variables we have to tell the compiler how we want to access them

| Access Type          | Capture Clause |
|-------------------- |-------------- |
| By Reference         | [&]            |
| By Value             | [=]            |
| Individually specify | [varA, &varB]  |


## Lambda Example

```c++
#include <iostream>

using namespace std;

void print_sum(int x1, int x2) {
  cout << x1 << " + " << x2 << " = " << x1 + x2 << endl;
}

int main() {
  int calculated_value = 10;

  auto print_plus_calculated_val = [&calculated_value](int x) {
    print_sum(calculated_value, x);
  };

  for (int i = 0; i < 5; i++) {
    print_plus_calculated_val(i);
  }
}
```

[Example](https://ideone.com/i5U6hC)


## STL and Functors

-   Many STL algorithms accept functors and lambdas as their last argument
-   Types of Functors:
    -   Generator: a Functor that takes no arguments
    -   Unary Function: a Functor that can be called with one argument
    -   Binary Function: a Functor that can be called with two arguments


## Predicates

-   A Predicate is a function or functor that returns a boolean value
-   A common use for predicates is to allow an STL algorithm to be applied only to the members of a container that meet the requirement of the Predicate


## `count_if`

-   `count_if` is a STL function that counts the number of elements in a container that match the condition of the unary predicate it is given
-   `count_if(InputIterator first, InputIterator last, UnaryPredicate pred)`


## `count_if` example

```c++
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

int main() {
	vector<int> x{ 1, 2, 3, 4, 5, 6, 7, 8 };

	int numOdd = count_if(x.begin(), x.end(), [] (int in) {
		return in%2;
	});

	std::cout<< "there were " << numOdd << " odd numbers" << std::endl;

	return 0;
}
```

[Example](https://ideone.com/6wBPvF)


## transform

-   transform is a STL function that iterates through a given range applying the given Unary function and storing it in a second range
    -   Useful hint: The output range can be the same as the input range causing transform to just overwrite each value
-   transform(InputIterator first, InputIterator last, OutputIterator start, UnaryOperation op)


## transform example

```c++
#include <algorithm>
#include <iostream>
#include <string>

int main()
{
    std::string s("hello world");

    std::transform(s.begin(), s.end(), s.begin(),
	[](char c) { return std::toupper(c); });

    std::cout << "s is: " << s << std::endl;

    return 0;
}
```

[Example](https://ideone.com/RTQd7i)


# Exercise: Color Binning

1.  Drive the light sensor across the series of strips recording all of the values seen
2.  Use a STL function to replace each reading with the value of:
    -   (the reading) / 256
    -   This will sort the values into 16 categories ("bins")
3.  Use a for loop and another STL function to print the number of readings that fell into each category