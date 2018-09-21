# What are we doing today?

-   Introduction to C++
    -   Functions
    -   Headers and includes
    -   Namespaces
    -   `auto` keyword
    -   Working with strings
-   Using the STSL Robot API
    -   Drive in a square
    -   Line following


# What is C++?

-   Compiled (fast)
-   Can be written in many different ways (flexible)
-   Not merely "C with classes"


# Starting notes

-   We assume some knowledge of basic of C/C++ syntax (if, for, while, etc)
    -   If you are lost today, there is another "Week 0" training session which will teach these things


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

<http://cpp.sh/9yupt>


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
-   Try it yourself <http://cpp.sh/6lvcn>


# Forward Declaration

-   Declaration = defining the name, parameters, and return type
    -   `int foo(int x);` OR
    -   `int foo(int);`
-   Definition = declaration + filling in the function
    -   `int foo(int x) { return x+1; }`
-   If you forward-declare a function, you must do it higher up in the file than the definition
-   <http://cpp.sh/4mbgg>


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
    -   Many things like std::cout don't exist unless you import them
    -   <http://cpp.sh/7jb5t>
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
    -   <http://cpp.sh/24v43>
-   Solution: protect your variables and function names with a unique namespace
    -   <http://cpp.sh/66mrj>
    -   Use keywords from a namespace using the `::` operator
        -   types: `std::string`, `my_library::MyClass`
        -   functions: `std::min_element`
        -   static variables: `std::string::npos`
-   Best practice: everything you write that is included should be in a namespace


# "using namespace" keyword

-   <http://cpp.sh/4d5gz>
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


# STSL: RJRobot API

```C++
1  RJRobot robot(REAL);  // Make a new robot. Simulation may come later
2  robot.SetMotor(Motor::LEFT, -255);  //-255 to 255 range on motors
3  robot.SetMotor(Motor::RIGHT, 255);
4  robot.Wait(1000ms);
5  robot.StopMotors();
6  int line_brightness = robot.GetLineValue(LightSensor::CENTER);  // downwards line sensor
7  double clearance = robot.GetUltrasonicDistance();  // forwards ultrasonic sensor
8  Color ball_color = robot.GetColor();  // forwards color sensor. RED, BLUE, or UNKNOWN
```

Full details in [STSL/RJRobot.h](https://github.com/RoboJackets/stsl/blob/master/include/STSL/RJRobot.h)


# Connecting to the Robot

-   Make sure you have the software-training repo cloned
-   Open CLion
-   Open the existing project software-training/hardware<sub>applications</sub>
-   Connect to your robot's wifi network
-   In the Build Configuration menu in the top-right of CLion, select spin<sub>in</sub><sub>place</sub>
-   Hit the run button


# Today's coding exercises

-   Drive in a square
    -   modify code in drive<sub>in</sub><sub>square</sub> folder
    -   Use a combination of SetMotor, Wait, and StopMotors to drive in a square
    -   You should use a for loop
-   Line Following
    -   Basic algorithm: if black, turn forward-left, else turn forward-right
    -   Implement a better way if you want