# What are we doing today?

-   Introduction to C++
    -   What it is
    -   Functions
    -   Includes and headers
    -   Working with strings
-   How to find and read documentation
-   Sending and recieving changes from remote Git repositories


# What is C++?

<div class="NOTES">
C++ is a more general programming langauge than C. It can have higher levels of abstraction. C is minimalist. C++ also has classes, it is not C with classes.

</div>

-   A general-purpose programming language
-   Designed with performance, efficiency, and flexibility in mind
-   Great for robots!


# Functions

<div class="NOTES">
After introducing the concept, write a simple example. Don't go into prototypes yet, we'll cover them in a bit.

Open the example code on your display and try to stick to that as much as you can.

</div>

-   A function is a chunk of code with defined inputs and outputs
    -   Minimizes copy-and-paste
    -   Less buggy
    -   Easier to test than a whole program
-   These can be chained together for more complex functionality
-   You want each function to have a single high-level purpose


# Anatomy of a function declaration

-   C++ lets you have multiple functions with the same name, provided their parameter signatures are different
    -   A function's parameter signature is just the ordered list of its parameter types.
    -   `foo(int a, char b)` and `bar(int x, char z)` have the same signature
    -   `fizz(int a, char b)` and `buzz(char x, int z)` don't
-   The compiler will determine which function to actually call depending on the parameters
-   (This will come in handy later!)


# Header files

<div class="NOTES">
Now you can talk about prototypes!

</div>

-   Only contain *declarations*, also known as prototypes, not definitions
    -   Function name
    -   Arguments
    -   Return type
    -   No functionality
    -   Example: `int add(int a, int b);`
-   This enables a lot of different files to "know" about a function but only have one definition for it


# Note about header files

-   C++ has a lot of flexibility; this is a blessing and a curse
-   Your code will have little structure unless you organize it well
-   Break up your code logically and purposefully
    -   Readability above all


# Includes

<div class="NOTES">
After introducing the concept, demo some basic examples of including a file within another file. Draw a picture on the board of a.h included in b.h and then b.h included in another file

</div>

-   Allows you to use code from other files
-   Will literally insert anything in name.h into the file

```C++
#include <name.h>
```


# Header guards C style

<div class="NOTES">
using the previous example include a.h in the final file as well. Show how the same code is copied in twice.

</div>

-   header guards prevent the compiler from including the same file multiple times
-   will work in all cases
    -   uses C style macros

```C++
1  #ifndef FILENAME_H
2  #define FILENAME_H
3      ...
4  #endif  // FILENAME_H
```


# #pragma once

-   a different method to do header guards
    -   cleaner
-   most compilers support this

```C++
1  #pragma once
```


# Working with strings

<div class="NOTES">
Write and walk through some example code.

</div>

-   A C-style string is stored as an array of characters ending with a NULL (0x0)
-   The C++ standard library contains a string class with several useful functions, in `<string>`
    -   (We'll get to classes later)
    -   Use this instead of C-style strings unless you have a specific reason not to
-   Use the addition oprator (+) to concatenate strings
-   Use `to_string` to convert other types to strings
-   Use `sto_` functions to parse strings to other types
    -   Ex). `stoi` to convert string to integer
    -   Ex). `stod` to convert string to double


# String methods

| Name     | Description                                     |
|-------- |----------------------------------------------- |
| `length` | Returns the length of a string                  |
| `substr` | Returns a portion (substring) of the string     |
| `find`   | Returns the position of a substring, if present |
| `empty`  | Returns true if length is zero, otherwise false |


# How to find and read documentation

<div class="NOTES">
Show how to get to the string documentation

</div>

-   Most of the last slide was sourced from [cppreference.com](http://en.cppreference.com/w/)
-   Documents standard library functionality
-   Great reference for all things C++


# Example Code

<div class="NOTES">
draw the tree structure of the example code files. week2.cpp includes week2.h which includes operator.h. operator.cpp includes operator.cpp.

</div>


# Git

<div class="NOTES">
Time to switch to the Git/GitHub presentation.

</div>

-   [Click here for this week's Git presentation](git.md)