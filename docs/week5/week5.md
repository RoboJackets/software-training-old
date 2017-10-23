# What are we doing today?

-   Classes in C++
-   Writing a differential drive class


# What are classes?

-   Definitions for objects, i.e. what data and methods they have
    -   In other words, a blueprint for how to create an object
-   An object is called an **instance** of a class
    -   An object has all the functions and data structures defined in its class
    -   Data schema is the same across all instances of a class, but values differ
    -   A method can treat all instances of an object the same
        -   Think of it like a complex, user defined variable type


## Structure

<div class="NOTES">
talk about the structure of a class

</div>

-   Typically declaration in a .h file with implementation in a cpp file
    -   This is not a strict rule

```c++
class NAME {
    // decleration of variables
    // method prototypes
    //   can be declared in this file or a seperate cpp file
};
```


# Why bother?

-   Software projects are easier to understand when related data and functions are grouped together
-   Abstracting data behind getters/setters allows you to validate inputs from other parts of your application
-   Abstracting complex tasks into class methods enables simpler, easier to read high-level code
-   Classes can be extended to add functionality with minimal extra code using inheritance


# Scope

<div class="NOTES">
Make sure to explain the different reason for not giving everything public scope

</div>

-   Data and methods can have their scope set in order to have limited availability
    -   Prevents unverified values from being inserted in class variables
        -   Can force using getters and setters to interact with data
    -   Hides the internals of your classes


## public

-   Accessible by everyone

```c++
class example {
public:
    int var;
    void do_something() {std::cout << "doing something" << std::endl;}
};
```


## private

<div class="NOTES">
try to use the private method and then show them the compiler error. Show an exaple of using the var in the class

</div>

-   Data and methods are only accessible from within the class itself
    -   Forces using setters and getters as an abstraction layer
-   implied if no scope is initially set in a class

```c++
class example {
private:
    int var;
    void do_something() {std::cout << "doing something" << std::endl;}
};
```

```c++
class example {
    int var;
    void do_something() {std::cout << "doing something" << std::endl;}
};
```


## review

| Scope Type | Accessible inside the class | Accessible outside the class directly |
|---------- |--------------------------- |------------------------------------- |
| private    | Yes                         | No                                    |
| public     | Yes                         | Yes                                   |


# A simple example

<div class="NOTES">
A class can have some data and some methods, which can either be public or private. Drop to a terminal and use/modify this class.

</div>

```c++
class example {
    private:         // private is also implied if you don't specify visibility
	int a;
    public:
	int getA() { return a; }
};
```


# Constructors and Destructors

<div class="NOTES">
DO NOT go into why some constructors might be private.

</div>

-   Often there are things that are required to create or destroy an instance of a class
-   We put the construction code in constructors
-   We put the destruction code in destructors
-   Usually public, scope still applies to these methods


## Constructors

<div class="NOTES">
use this version of the class

</div>

-   Called whenever an object is created
-   Can have multiple constructors as long as they have different argument lists
-   Contains all of the code to create and initialize all of an object's members

```c++
class example {
    private:
	int a;
	// lots of important variables that live on the heap
    public:
	int getA() { return a; }
	example() {
	    std::cout << "Creating an example" << std::endl;
	    /* allocating all the memory */
	};
	example(int a_local) {
	    a = a_local;
	    /* allocating all the memory */
	}
};
```


## Destructors

<div class="NOTES">
use this version of the class. Make sure to destruct the object

</div>

-   This is always a no argument method
-   Contains all of the code to destory and deallocate all an objects members
-   Can only have one destructor

```c++
class example {
    private:
	int a;
	// lots of important variables from the heap
    public:
	int getA() { return a; }
	// ... constructors
	~example() {/* lots of deletes */};
};
```


# Static members

<div class="NOTES">
Drop to a terminal and use this class.

</div>

-   For when you want data or functions to be part of your class, but they don't need to be "attached" to an instance of the class
-   Static data is shared between all instances of a class
-   Remember the `static` keyword means something else outside of class definitions!
-   Static variables must be initialized before use
    -   accessed by the **::** operator

```c++
class static_example {
    private:
	static int a;
    public:
	static_example() {a++;}
	static int getA() { return a; }
};
// init in implementing class
int static_example::a = 0;
```


# Inheritance

<div class="NOTES">
pull up the two classes we have written so far and use the methods of the subclass under different circumstances

</div>

-   Classes can inherit data and methods from other classes

```c++
class child: public example {
    // we get `a` and `getA()` from example
    private:
	int b;
    public:
	int getB() { return b; }
	int getAplusB() { return getA() + b; } // we can't use `a` directly since it's private
};
```


## Example code

<div class="NOTES">
mention that you can go up the class structure but not down

</div>

![img](https://i.imgur.com/9cF7NTq.png)


# Scope and inheritence

<div class="NOTES">
make sure to explain every bullet

</div>

-   Scope applies across subclasses

| scope     | subclasses | externally | internally |
|--------- |---------- |---------- |---------- |
| private   | no         | no         | yes        |
| protected | yes        | no         | yes        |
| public    | yes        | yes        | yes        |


# Polymorphism

<div class="NOTES">
explain how dynamic cast is used to change what an object is

</div>

-   Now we can create `child` objects with all the properties of an `example` object
-   This means we can safely cast a `child` object to an `example` object
    
    ```c++
    child c;
    example& e = dynamic_cast<example&>(c);
    ```
-   But not the other way around
    
    ```c++
    example e;
    child& c = dynamic_cast<child&>(e);
    ```
    
    ```
    example.cpp: In function ‘int main()’:
    example.cpp:20:38: error: cannot dynamic_cast ‘e’ (of type ‘class example’) to type
    ‘class child&’ (source type is not polymorphic)
         child& c = dynamic_cast<child&>(e);
    				     ^
    ```


# Virtual functions

<div class="NOTES">
Show an example using the classes defined in the example code and explained earlier

</div>

-   A parent class can specify that certain functions are **virtual**
-   Child classes can then implement their own versions of the function
-   The child implementation will be called even from a reference of the type of the parent
    -   If the function isn't marked virtual, which implementation is called depends on the type of the reference


# Questions?

-   Ask here or on Piazza!