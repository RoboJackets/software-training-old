# Exercise 3.1
Welcome to the first exercise of week 3! This week we covered pointers, the canonical
hardest topic in C++ (so ask questions if you get lost). We are going to be doing a simple
worksheet today and some code that goes along with it.

# Project Objective
If you open the .csv file in this folder you will see a couple columns (use libreoffice).
The left most if the column that describes what happens at each step, then you have a
line for the code (each step is only one line except the last line). At the far end you see
some locations for what is at the different locations.

If you see a,b,pa,pb,temp. These are five variables. The first couple lines are filled
in for you in the right side. If it is a primitive  (a,b,temp), you should put
what the value of that variables is, if it is a pointer (pa, pb) you should put what variable
it is pointing to.

# Code
Finally what you need to do is code up a simple example using the main.cpp file located in this
folder. Just write the code that would correspond to the different actions described in the text.

A couple important points to remember about syntax

```c++
int a = 21;
int* pa = &a;
```

The second line is creating a pointer ```int*``` the ```*``` indicates it is a pointer
to an int. ```&a``` is the address of operator. What this line does in words is to
assign the pointer pa the address of a. Now that means we can do the following and
change the value of a by using pa

```c++
*pa = 10;
```

this assigns the value of 10 to what pa is pointing at, in this case a. The ```*```
here means that we are dereferencing the pointer.

Again students tend to struggle at pointers initially, so please post questions on piazza.
The way most of us learned is just by doing these simple tasks with pointers until we
get a feel for what we are looking at.




