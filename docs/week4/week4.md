# What are we doing today?

-   How a program's memory is structured and maintained
-   References and pointers
-   Pass by reference/value
-   How to convert objects between different types


# What is the output?

<div class="NOTES">
Ask them the output of this code. Then talk about how var and a are not the same place in memory.

</div>

```c++
#include <iostream>
void increment(int a) {
  a = a + 5;
}
int main() {
  int var = 5;
  increment(var);
  std::cout << var << std::endl;
}
```


# Memory structure

<div class="NOTES">
Make sure to emphasize how important memory allocation is for any real program. This is all fundamentally the same RAM, just arranged and managed in a certain way. Static memory is used for static/global variables. A new block is allocated when a function is called, and released when the function exits - this is where your parameters and local variables are stored. Mention memory leaks.

</div>

![img](https://i.imgur.com/5dJ912z.png)

-   All the memory in your computer is a huge, indexable space divided into bytes
-   Your program has three kinds of memory
    -   **Static memory:** calculated at compile time
    -   **The stack:** managed automatically by the language
    -   **The heap:** your space to do whatever you want
        -   You can ask for heap memory from the OS
        -   You have to release/free what you've allocated to avoid memory leaks


## What is a byte

<div class="NOTES">
They may never have seen binary before. 00101010 = 26, 00010001 = 17 do not talk about how binary works

</div>

-   8 bits (0,1)
    -   00101010
    -   00010001
-   smallest indexable space on your computer


## Object lifetimes stack

<div class="NOTES">
Picture on next slide. talk about how in the below example the int is allocated on the stack. mention what stack overflow is

</div>

-   All variables have different lifetimes depending on where they are allocated in memory
-   Every method call has its own copy of the variables on the stack

```c++
#include <iostream>
void increment(int a) {
  int c = 5;
  a = a + 5;
}
int main() {
  int d = 6;
  int var = 5;
  increment(var);
  std::cout << var << std::endl;
}
```


### Stack example

```c++
#include <iostream>
void increment(int a) {
  int c = 5;
  a = a + 5;
}
int main() {
  int d = 6;
  int var = 5;
  increment(var);
  std::cout << var << std::endl;
}
```

![img](https://imgur.com/b6KnwB2.png)


# References

<div class="NOTES">
Parameters are copied into the new block so the function can't modify anything outside of its scope. Explicitly using a reference allows a function to do that. Performance can also improve, since you're only copying a memory address instead of a (potentially) larger object. Run through some basic examples here.

</div>

-   A reference is a variable that points to the same content as another variable
-   Why would you want that?
    -   Share data across functions
    -   Improve performance
-   You can't change where a reference points after initialization
-   You can mostly just treat references exactly like normal variables once they're initialized


## Passing by reference

<div class="NOTES">
ask them what the output of the code is and then walk them through it

</div>

-   use the **&** next to a variable to make it pass by reference

```c++
#include <iostream>
void increment(int &a) {
  a = a + 5;
}
int main() {
  int var = 5;
  increment(var);
  std::cout << var << std::endl;
}
```


# Pointers

<div class="NOTES">
Pointers are great for passing around heap objects, but how do you keep track of what objects are still in use?

</div>

-   A pointer is a special type of variable that "points to" another variable
    -   the pointer variable stores the address of a variable
-   How are these different from references?
    -   You have to explicitly get the value a pointer is pointing to
    -   You can change where a pointer is pointing


## How to get an address

-   The **&** operator is used to get an address of a variable
-   Use the **\*** operator to denote a pointer variable

```c++
int main() {
  int var = 5;
  // TYPE* name;
  int* var_ptr = &var;
}
```


### Example

```c++
int main() {
  int var = 5;
  // TYPE* name;
  int* var_ptr = &var;
}
```

![img](https://imgur.com/HvxBD32.png)


## Dereferencing pointers

-   gets the value pointed to
    -   here is an address, what is there?
-   use the **\*** operator in front of a pointer to dereference it
-   use the `->` operator behind a pointer to deference and do something else


## Example

<div class="NOTES">
Now run through some basic examples

</div>

```c++
#include <iostream>
int main() {
  int var = 5;
  // TYPE* name;
  int* var_ptr = &var;
  std::cout << *var_ptr << std::endl;
}
```

![img](https://imgur.com/7alG4QH.png)


## Another example

```c++
#include <iostream>
#include <string>

int main() {
  int var = 5;
  std::string s = "Pointers are fun";

  // TYPE* name;
  int* var_ptr = &var;

  std::string* str_ptr = &s;
  str_ptr->push_back('!');

  std::cout << "var_ptr is " << *var_ptr << std::endl;
  std::cout << "str_ptr has size " << str_ptr->size() << std::endl;
  std::cout << "Now s is: " << s << std::endl;
}

```


# Pointer arithmatic

-   In c++ different datatypes have different sizes
-   A pointer will increment differently based on its datatype's size in bytes

| `type` | `# bytes` |
|------ |--------- |
| char   | 1         |
| int    | 4         |
| double | 8         |


## Example int

```c++
#include <iostream>
int main() {
  int var = 6;
  int* var_ptr = &var;
  var_ptr += 1;
}
```

![img](https://imgur.com/74Lb2KQ.png)


## Example char

```c++
#include <iostream>
int main() {
  char rj = 'c';
  char* char_ptr = &rj;
  char_ptr += 2;
}
```

![img](https://imgur.com/vc7sR0R.png)


# New / Delete

<div class="NOTES">
do example when you get to the delete slide

</div>

-   To allocate memory on the heap you can use **new**
    -   be careful how much you allocate
    -   returns a pointer to the memory
-   To deallocate the memory you must use **delete**
    -   anytime memory is allocated you must deallocate it
    -   you pass the pointer to the memory you got from new
    -   not deallocating is the cause of memory leaks
        -   stay tuned for a demo


## New

```c++
int main() {
  double *a = new double(10.0);
}
```

![img](https://imgur.com/z3xGol7.png)


## Delete

<div class="NOTES">
use the stuct defined in week4.h, prints when constructed and destructed.

</div>

```c++
int main() {
  double *a = new double(10.0);
  // ... legit code
  delete a;
}
```

![img](https://imgur.com/A6a4bsz.png)


### Segfault

<div class="NOTES">
tell them what a segfault is

</div>

```c++
int main() {
  double *a = new double(10.0);
  // ... legit code
  delete a;
  // .. more code
  *a = 20.0
  // .. more code
}
```


# Smart pointers

<div class="NOTES">
Each type has an initialization function. Run through examples in 4 slides

</div>

-   A smart pointer object manages a pointer
-   When the smart pointer is done (e.g. end of function), calls `delete` on its data
-   You should generally use one of these two types instead of a normal pointer:
    -   `shared_ptr`: multiple `shared_ptr` objects can own a single pointer
    -   `unique_ptr`: only one `unique_ptr` can own an pointer


## `unique_ptr`

<div class="NOTES">
object is a struct defined in week4.h. make sure to use it as an example

</div>

```c++
int main() {
  unique_ptr<object> u_ptr = make_unique<object>();
}
```

![img](https://i.imgur.com/nK51rsP.png)


## `shared_ptr`

<div class="NOTES">
object is a struct defined in week4.h. make sure to use it as an example

</div>

```c++
int main() {
  shared_ptr<object> s_ptr1 = make_shared<object>();
  // <--- HERE
  {
    shared_ptr<object> s_ptr2 = s_ptr1;
  }
  shared_ptr<object> s_ptr3 =  s_ptr1;
}
```

![img](https://i.imgur.com/Sy6CY5q.png)


### `shared_ptr`

<div class="NOTES">
object is a struct defined in week4.h. make sure to use it as an example

</div>

```c++
int main() {
  shared_ptr<object> s_ptr1 = make_shared<object>();
  {
    shared_ptr<object> s_ptr2 =  s_ptr1;
    // <--- HERE
  }
  shared_ptr<object> s_ptr3 =  s_ptr1;
}
```

![img](https://i.imgur.com/y3ITWBn.png)


### `shared_ptr`

<div class="NOTES">
object is a struct defined in week4.h. make sure to use it as an example

</div>

```c++
int main() {
  shared_ptr<object> s_ptr1 = make_shared<object>();
  {
    shared_ptr<object> s_ptr2 =  s_ptr1;
  }
  shared_ptr<object> s_ptr3 =  s_ptr1;
  // <--- HERE
}
```

![img](https://i.imgur.com/LV3kigZ.png)


# Ownership

-   be careful about returning a shared pointer from a method
    -   who owns what?


# Casting, the wrong way

-   Sometimes you want to convert a variable to a different type: this is called casting
-   In C, you can simply change the type of a variable like so:

```c++
B* old_var = new B(); // some object
A* new_var = (A*)old_var;
```

-   The compiler will now treat the bytes of object `old_var` as though it was of type `A`
-   This is bad for type safety!


# Type Safety

```c++
char c = 10;                        // this is one byte in memory

int *p = (int*) &c;                 // this is a 4-byte pointer pointing to one byte of
				    // memory - it compiles but leads to corrupted memory
				    // if you try to write to what p points to

int *q = static_cast<int*>(&c);     // throws an exception at compile time
```

-   Our second cast was type-safe, the first one was not
-   Type safety is a language feature that ensures that every variable you handle is actually the type you think it is
    -   The compiler will check each time you assign a variable to ensure the types are compatible


# Casting, the right way

<div class="NOTES">
Really talk about why type safety is good. otherwise you can arbitrarily cast any type to any other. You might not make the mistake but someone else will

</div>

-   C++ gives us functions that are type-safe, including compile-time checks
    -   `static_cast` is mostly used for basic type conversions, e.g. between different types of numbers
    -   `dynamic_cast` is mostly used for conversions between object types for polymorphism
    -   `reinterpret_cast` works like a C-style cast and generally shouldn't be used unless you know exactly what you're doing


# Exercise: line following with a twist

-   Modify `line_follow.cpp`
-   You should make 2 functions `turn_right` and `turn_left`
-   We can't use the function signature `turn_left(RJRobot r)`
    -   This copies the RJRobot object, opening a second wifi link to the robot (which doesn't work)


# Exercise cont'd

-   You can solve this problem several ways by changing the function signature. For practice, try these three:
    1.  Pass a reference to the RJRobot
    2.  Pass a pointer to the RJRobot (a non-smart pointer is fine)
    3.  Pass no parameters
        -   Declare a global (outside of any function) `unique_ptr` to an RJRobot
        -   Don't forget to `make_unique` in `main`
        -   You can use this `unique_ptr` in any function

-   Line follow algorithm:
    -   `repeat forever: if (sensor < threshold) then turn_left, else turn_right`