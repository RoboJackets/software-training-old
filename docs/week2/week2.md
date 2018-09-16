# What are we doing today?

-   Introduction to C++
    -   Functions
    -   Headers and includes
    -   Namespaces
    -   Working with strings
-   Using the STSL Robot API
    -   Drive in a square
    -   Line following


# What is C++?

-   Compiled (fast)
-   Can be written in many different ways (flexible)
-   NOT "C with classes" (much more powerful)


# Starting notes

-   We assume some knowledge of the absolute basics of C/C++ syntax (if, for, while, etc)
    -   There is another "Week 0" training session which will teach these things


# hello-world.cpp

```C++
1  #include <iostream>
2  
3  int main() {
4    std::cout << "Hello, world!" << std::endl;
5  }
```

<http://cpp.sh/9yupt>


# Functions in C++

-   Review: A function is a chunk of code with inputs and outputs
-   C++ functions have 0 or more parameters (inputs) and always have 1 output
-   Each parameter has a type and there is a return type
-   The special function `main()` with return type `int` gets run automatically
-   Example
    -   Want to add two integers, x and y, and print their result


# Functions Example

1.  Informative function name
2.  Input types
3.  Return type

```C++
1  void addAndPrint(int x, int y) {
2  
3  }
```


# Functions Example

-   Perform the addition

```C++
1  void addAndPrint(int x, int y) {
2      int sum = x + y;
3  }
```


# Functions Example

-   Print the result

```C++
1  void addAndPrint(int x, int y) {
2      int sum = x + y;
3      std::cout << sum << std::endl;
4  }
```


# Functions Example

-   We can split up this code to show you real return types
-   `return` statement must be used since the return type is not `void`

```C++
 1  void addAndPrint(int x, int y) {
 2      int sum = x + y;
 3      std::cout << sum << std::endl;
 4  }
 5  
 6  int add(int x, int y) {
 7      return x + y;
 8  }
 9  
10  void print(int value) {
11      std::cout << value << std::endl;
12  }
```

`print(add(3, 4))` gives the same result as `addAndPrint(3, 4)` Try it yourself <http://cpp.sh/6lvcn>


# Forward Declaration

-   Declaration = defining the name, parameters, and return type
-   Definition = declaration + filling in the function
-   Functions have to be declared higher up in the file than when they are used
-   <http://cpp.sh/4mbgg>


# Includes

-   Why do we need `#include <iostream>`?
    -   <http://cpp.sh/7jb5t>
-   Includes let you bring other code into a file
-   Use this for
    -   Standard library functions and data types (anything std::)
    -   Other installed libraries (e.g. ROS, OpenCV, Qt)
    -   Splitting up a large program into multiple files
-   Without including anything, C++ is very limited


# Include syntax

-   `<>` gets code from standard library or apt-installed libraries
-   `""` gets code from a nearby folder
-   If `""` does not find the file, it reverts to `<>` behavior
-   What's in the `<>` or `""` is a file name
    -   Traditionally the file name has extension ".h" or ".hpp"
-   Examples:

```C++
1  #include <string>  // standard library
2  #include <QWidget>  // file from Qt installed through apt
3  #include <ros/ros.h>  // file from ros installed through apt
4  #include "include/my_interface.hpp"  // another file in the same project,
5  				     // in a folder called "include"
```


# Namespaces

-   What happens when two things are assigned the same name?
    -   <http://cpp.sh/24v43>
-   Solution: protect your variables and function names with a unique namespace
    -   <http://cpp.sh/66mrj>
    -   Use keywords from a namespace using the `::` operator
        -   types: `std::string`, `my_library::MyClass`
        -   functions: `std::min_element`
        -   static variables: `std::string::npos`
-   Everything that is included should be in a namespace


# "using namespace" keyword

-   <http://cpp.sh/4d5gz>
-   Handy (used in RJ) but also defeats the purpose of namespaces
-   Use with caution
    -   Must be clear, without the namespace, where the function comes from
    -   In general, use only one external namespace in a file (usually std)


# Strings

-   C string (still valid in C++):

```C++
1  char[] s = "this is a string";
```

-   C++ string:

```C++
1  #include <string>
2  std::string s1("this is a string");  // constructor
3  std::string s2 = "this is a string";  // same effect as constructor
```

-   Unlike in C, C++ strings are a class instances and have methods


# String Methods

-   What methods can I use for a string? (there are lots)
    -   Google! <https://www.google.com/search?q=c%2B%2B+std%3A%3Astring>
    -   cppreference.com is comprehensive and up-to-date
-   Adding to end: + and += operators, `append(string)`, `push_back(char)`
    -   <http://cpp.sh/84u76>
-   Reading user input: `std::cin >>`
    -   <http://cpp.sh/64mc>
-   Access characters like an array
    -   <http://cpp.sh/9gb4x>
-   `size()` and `length()` each get number of characters


# Excercise

-   Find a buddy (or work alone if you want)
-   Write a function `make_palindrome`
    -   Input: string
    -   Output: string with reversed copy attached
    -   `make_palindrome("apple")` returns `"appleelppa"`
-   Starter code: <http://cpp.sh/844tx>


# Solution

-   Using what we've learned so far
    -   <http://cpp.sh/92y54>
-   Preview for next week
    -   <http://cpp.sh/7jmjg>


# Our Training Robots

-   We have robots for you to use!
-   <TODO insert details on robot capabilities>


# STSL: Robot Control Basics

-   <TODO content>
    -   STSL functions for writing to motors
    -   compiling from terminal


# Exercise: Drive in Square


# Exercise: Line Following