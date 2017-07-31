# What are we doing today?

-   How a program's memory is structured at a high level
-   References and pointers
-   How to convert objects between different types
-   No `git` slides this week!


# Memory structure

<div class="NOTES">
This is all fundamentally the same RAM, just arranged and managed in a certain way. Static memory is used for static/global variables. A new stack frame is allocated when a function is called, and released when the function exits - this is where your parameters and local variables are stored. Mention memory leaks.

</div>

-   At a high level, your program has three kinds of memory
    -   **Static memory:** allocated at compile time
    -   **The stack:** managed automatically by the language
    -   **The heap:** your space to do whatever you want
        -   You can ask for memory from the OS and return it at runtime
        -   You have to keep track of what you've allocated to avoid memory leaks
-   At an even higher level, all the memory in your computer is a huge, indexable, iterable space, divided into bytes


# References

<div class="NOTES">
Parameters are copied into the new stack frame so the function can't modify anything outside of its scope. Explicitly using a reference allows a function to do that. Performance can also improve, since you're only copying a memory address instead of a (potentially) larger object. Run through some basic examples here.

</div>

-   A reference is a variable that points to the same content as another variable
-   Why would you want that?
    -   Share data across stack frames
    -   Improve performance
-   You can't change where a reference points after initialization
-   You can mostly just treat references exactly like normal variables once they're initialized


# Pointers

<div class="NOTES">
Run through some basic examples here. Pointers are great for passing around heap objects, but how do you keep track of what objects are still in use?

</div>

-   A pointer is a special type of variable that "points to" another variable
-   How are these different from references?
    -   You have to explicitly dereference a pointer to access its target
    -   You can change where a pointer is pointing


# Smart pointers

<div class="NOTES">
Each type has an initialization function. Run through some basic examples here.

</div>

-   Wrap a normal pointer to make it easier to manage the lifecycle of objects
-   You should generally use one of these two types instead of a normal pointer:
    -   `shared_ptr`: multiple `shared_ptr` s can own a single object
    -   `unique_ptr`: only one `unique_ptr` can own an object


# Casting, the wrong way

-   Sometimes you want to convert a variable to a different type: this is called casting
-   In C, you can simply change the type of a variable like so:

```c
B* old_var = ...; // some object
A* new_var = (A*)old_var;
```

-   The compiler will now treat the bytes of object `old_var` as though it was of type `A`
-   This is bad for type safety!


# Type Safety

```c
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

-   C++ gives us functions that are type-safe, including compile-time checks
    -   `static_cast` is mostly used for basic type conversions, e.g. between different types of numbers
    -   `dynamic_cast` is mostly used for conversions between object types for polymorphism
    -   `reinterpret_cast` works like a C-style cast and generally shouldn't be used unless you know exactly what you're doing


# Questions?

-   Ask here or on Piazza!