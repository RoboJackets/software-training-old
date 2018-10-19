# What are we doing today?

-   Classes in C++
-   Inheritance and polymorphism


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
class Example {
    // declaration of variables
    // method prototypes
    //   can be declared in this file or a seperate cpp file
    void DoThing(int);
};
Example::DoThing(int x) {
    // ... implementation of the function
}
```

[Example](https://ideone.com/jaRQsF)


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
class Example {
public:
    int var;
    void do_something() {std::cout << "doing something" << std::endl;}
};
```

[Run It](https://ideone.com/2pacF2)


## private

<div class="NOTES">
try to use the private method and then show them the compiler error. Show an exaple of using the var in the class

</div>

-   Data and methods are only accessible from within the class itself
    -   Forces using setters and getters as an abstraction layer
-   implied if no scope is initially set in a class

```c++
class Example {
private:
    int var;
    void do_something() {std::cout << "doing something" << std::endl;}
};
```

```c++
class Example {
    int var;
    void do_something() {std::cout << "doing something" << std::endl;}
};
```

[Run It](https://ideone.com/huMvVg)


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
class Example {
    private:         // private is also implied if you don't specify visibility
	int a;
    public:
	int getA() { return a; }
};
```

[Run It](https://ideone.com/eUhSw6)


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
-   `default` constructors initialize all data members to default values (default constructor is used if no custom constructor is given)

```c++
class Example {
    private:
	int a;
	// lots of important variables that live on the heap
    public:
	int getA() { return a; }
	Example() = default;
	Example(int a_local) {
	    a = a_local;
	    std::cout << "custom constructor" << std::endl;
	    /* allocating memory, etc. ... */
	}
};
```

[Run It](https://ideone.com/5y5iAk)


## Destructors

<div class="NOTES">
use this version of the class. Make sure to destruct the object

</div>

-   This is always a no argument method
-   Contains all of the code to destory and deallocate all an objects members
-   Can only have one destructor

```c++
class Example {
    private:
	int a;
	// lots of important variables from the heap
    public:
	int getA() { return a; }
	// ... constructors
	~Example() {/* lots of deletes */};
};
```

[Run It](https://ideone.com/NBki2a)


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
class StaticExample {
    private:
	static int a;
    public:
	void modify_static_var() { a++; }
	static int getA() { return a; }
};
// init in implementing class
int static_example::a = 0;
```

[live demo of static](https://ideone.com/LiGymG)


# Inheritance

<div class="NOTES">
pull up the two classes we have written so far and use the methods of the subclass under different circumstances

</div>

-   Classes can inherit data and methods from other classes
-   Inheritance is an "is" relationship

![img](https://i.imgur.com/9cF7NTq.png)


## Inheritance Example

```c++
class Child: public Example {
    // we get `a` and `getA()` from example
    private:
	int b;
    public:
	int getB() { return b; }
	int getAplusB() { return getA() + b; } // we can't use `a` directly since it's private
};
```


# Scope and inheritence

<div class="NOTES">
make sure to explain every bullet

</div>

-   Scope applies across subclasses

| scope     | internally | subclasses | externally |
|--------- |---------- |---------- |---------- |
| private   | yes        | no         | no         |
| protected | yes        | yes        | no         |
| public    | yes        | yes        | yes        |


## Inheritance and constructor chaining

-   The constructor of the parent class is called before the constructor of the child class
-   Passing parameters to the parent constructor
    -   Place `: Parent(args)` before the opening brace

```c++
class Example {
public:
    int a;
    Example(int a2) { a = a2; }
};
class Child: public Example {
public:
    int b;
    Child(int a2, int b2) : Example(a2) {
	b = b2;
    }
};
```


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
    example.cpp:20:38: error: cannot dynamic_cast ‘e’ (of type ‘class Example’) to type
    ‘class child&’ (source type is not polymorphic)
         child& c = dynamic_cast<child&>(e);
    				     ^
    ```

[Polymorphism Example](https://ideone.com/ETJ0uh)


# Virtual functions

<div class="NOTES">
Show an example using the classes defined in the example code and explained earlier

</div>

-   A parent class can specify that certain functions are **virtual**
-   Child classes can then implement their own versions of the function
-   The child implementation will be called even from a reference of the type of the parent
    -   If the function isn't marked virtual, which implementation is called depends on the type of the reference

[Run It](https://ideone.com/iFqp7I)


## Exercise: RJ Robot Extended to drive in square

-   Starter code in `hardware_applications/inheritance` folder
-   File `inheritance.cpp` is already complete
-   Your `RJRobotExtended` class should have the following:
    -   Be a subclass of RJRobot
    -   Have a private data member `duration` for an amount of time to drive on the edge of the square
        -   This is a `std::chrono::milliseconds` type
        -   You should have a constructor argument for this
    -   private methods
        -   `DriveEdge()`: drive forward for `duration` milliseconds
        -   `DriveCorner()`: turn 90 degrees right
    -   public methods:
        -   `DriveInSquare()`: drive in a square, calling your private