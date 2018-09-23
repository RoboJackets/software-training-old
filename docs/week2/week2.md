# What are we doing today?

-   Introduction to C++
    -   Functions
    -   Headers and includes
    -   Namespaces
    -   `auto` keyword
    -   Working with strings
-   Using the RJRobot API
    -   Drive in a square
    -   Line following


# What is C++?

-   Compiled (fast)
-   Can be written in many different ways (flexible)
-   Not merely "C with classes"


# Starting notes

-   We assume some knowledge of basic of C/C++ syntax (if, for, while, datatypes, etc)
    -   If you are lost today, there is a catch-up training session which will teach these things. Next Saturday 2-4


# hello-world.cpp

-   A simple C++ program, we will be explaining what's going on here and more
-   We will be using cpp.sh for live demos

```C++
1  #include <iostream>
2  
3  int main() {
4    std::cout << "Hello, world!" << std::endl;
5  }
```

[Hello World live example](http://cpp.sh/9yupt)


# Compiling and Running

-   Compiling turns C++ code into an executable program
-   g++ is a common C++ compiler

```
g++ hello-world.cpp -o hello-world.out
./hello-world.out
```


# Functions in C++

-   Review: A function is a chunk of code with inputs and outputs
-   C++ functions have 0 or more parameters (inputs) and have 0 or 1 output
-   Each parameter has a type and there is a return type
-   The special function `main()` with return type `int` gets run automatically
-   Example
    -   Want to add two integers, x and y, and print their result


# Making a Function

1.  Informative function name
2.  Input types
3.  Return type (`void` means no return/output)

```C++
1  void addAndPrint(int x, int y) {
2  
3  }
```


# Making a Function

-   Use the parameters to do something

```C++
1  void addAndPrint(int x, int y) {
2      int sum = x + y;
3      std::cout << sum << std::endl;
4  }
```


# Functions Example

-   We can split up this code to show you real return types
-   `return` statement must be used since the return type is not `void` anymore

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

-   `print(add(3, 4))` gives the same result as `addAndPrint(3, 4)`
-   [Verify that this indeed works](http://cpp.sh/8skfr)


# Forward Declaration

-   Declaration = defining the name, parameters, and return type
    -   `int foo(int x);` OR
    -   `int foo(int);`
-   Definition = declaration + filling in the function
    -   `int foo(int x) { return x+1; }`
-   If you forward-declare a function, you must do it higher up in the file than the definition
-   [Forward Declaration live example](http://cpp.sh/92su4m) <- try removing the declaration


# Overloading

-   Function signature = name + list of parameter types
-   Each function must have a unique signature. Names can be the same as long as the parameters are different

```C++
1  // these all have different signatures
2  int add(int, int);
3  int add(int, int, int);
4  double add(double, double);
5  double add(double, double, double);
```


# Includes

-   Why do we need `#include <iostream>`?
    -   [Many things like std::cout don't exist unless you import them](http://cpp.sh/7jb5t)
-   Includes let you bring other code into a file
-   Use this for
    -   Standard library functions and data types (anything std::)
    -   Other installed libraries (e.g. ROS, OpenCV, Qt)
    -   Splitting up a large program into multiple files
-   Without including anything, C++ is very limited


# Include syntax

-   `<>` gets code from the standard library or installed libraries
-   `""` gets code from a nearby folder, or does the same thing as `<>` if it can't find anything
-   What's in the `<>` or `""` is a file name
    -   Traditionally the file name has extension ".h" or ".hpp"
-   Examples:

```C++
1  #include <string>  // standard library
2  #include <QWidget>  // file installed with Qt
3  #include <ros/ros.h>  // file installed with ROS
4  #include "include/my_interface.hpp"  // another file in the same project,
5  				     // in a folder called "include"
```


# Namespaces

-   What happens when two things are assigned the same name (or the same function signature)?
    -   [You get an error](http://cpp.sh/24v43)
-   Solution: protect your variables and function names with a unique namespace
    -   [Same program, but with namespaces](http://cpp.sh/66mrj)
    -   Use keywords from a namespace using the `::` operator
        -   types: `std::string`, `my_library::MyClass`
        -   functions: `std::min_element`
        -   static variables: `std::string::npos`
-   Best practice: everything you write that is included should be in a namespace


# "using namespace" keyword

-   [Hello world, but using std namespace](http://cpp.sh/4d5gz)
-   Handy but also defeats the purpose of namespaces
-   Use with caution
    -   Must be clear, without the namespace, where the function comes from
    -   In general, use only one external namespace in a file (usually std)


# "Auto" Keyword

-   C++ can figure out for you what type something should be
-   Function `MakeObject()` returns some data of a particular type

```C++
namespace::MyVeryLongDataTypeName data = MakeObject();
```

OR

```C++
auto data = MakeObject();
```

This can make your code easier or harder to read/maintain, depending on whether you name your variables well


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

-   C++ strings are classes and have methods (like Java, unlike C)


# String Methods

-   What methods can I use for a string? (there are lots)
    -   [Online documentation!](https://en.cppreference.com/w/cpp/string/basic_string)
    -   cppreference.com is a very good resource
-   Adding to end: + and += operators, `append(string)`, `push_back(char)`
    -   [Adding to string example](http://cpp.sh/84u76)
-   Reading user input: `std::cin >>`
    -   [Reading user input example](http://cpp.sh/64mc)
-   Access characters like an array
    -   [Access chars example](http://cpp.sh/9gb4x)
-   `size()` and `length()` each get number of characters


# Exercise 1

-   Find a buddy (or work alone if you want)
-   Write a function `make_palindrome`
    -   Input: string
    -   Output: string with reversed copy attached
    -   `make_palindrome("apple")` returns `"appleelppa"`
-   Starter code: <http://cpp.sh/844tx>


# Solution

-   Using what we've learned so far
    -   <http://cpp.sh/92y54>
-   Using Standard Template Library (next week's topic)
    -   <http://cpp.sh/7jmjg>


# Our Training Robots

-   We have robots for you to use!
-   Your code runs on your laptop, sending commands via wifi to the robot
-   Sensors
    -   2 line
    -   1 color
    -   1 ultrasonic
    -   hand proximity / gesture


# RJRobot API

```C++
1  RJRobot robot(REAL);  // Make a new robot. Simulation may come later
2  robot.SetDriveMotors(200, -200);  //-255 to 255 range on motors
3  robot.SetLiftMotor(127);
4  robot.Wait(1000ms);
5  robot.StopMotors();
6  int line_brightness = robot.GetLineValue(LightSensor::CENTER);  // downwards line sensor
7  double clearance = robot.GetUltrasonicDistance();  // forwards ultrasonic sensor
8  Color ball_color = robot.GetColor();  // forwards color sensor. RED, BLUE, or UNKNOWN
```

Full interface defined in [RJRobot.h](https://github.com/RoboJackets/stsl/blob/master/include/STSL/RJRobot.h). Use code completion in CLion as well.


# Connecting to the Robot

-   Make sure you have the software-training repo cloned
-   Open CLion
-   Open the existing project `software-training/hardware_applications`
-   In the Build Configuration menu in the top-right of CLion, select the program you want to run (e.g. `spin_in_place`)
-   Hit the build button (hammer), make sure everything compiles
-   Connect to your robot's wifi network
-   Hit the build and run button (green arrow)
-   Hit the red square button to stop the robot


# Today's robot challenges

-   Drive in a square
    -   modify code in `drive_in_square` folder
    -   Use a combination of SetDriveMotors, Wait, and StopMotors to drive in a square
    -   You should use a for loop
-   Line Following
    -   Basic algorithm: if black, turn forward-left, else turn forward-right
    -   Implement a better way if you want


# Supplemental Challenge 1: "FizzBuzz"

-   Print numbers 1 to n, except:
    -   If the number is a multiple of 3, print "fizz" instead of the number
    -   If the number is a multiple of 5, print "buzz" instead of the number
    -   If the number is a multiple of both 3 and 5, print "fizzbuzz" instead
-   Example output (n = 8): `1 2 fizz 4 buzz fizz 7 8`
-   Starter code: <http://cpp.sh/8xuu>


# Supplemental Challenge 2: Merge Strings

-   Given two strings s1 and s2, return the string that has elements of s1 and s2 interweaved
-   "abc", "12345" -> "a1b2c345"
-   Hint: `<algorithm>` header contains `min` and `max` functions
    -   `max(4,7) -> 7`
-   Starter code: <http://cpp.sh/4z5as>


# Supplemental Challenge Solutions

-   FizzBuzz <http://cpp.sh/7zrsc>
-   Merge Strings <http://cpp.sh/2h4eg>
-   There are many ways to solve each of these