# What are we doing today?

-   Instructor introductions
-   What is programming?
-   Hello World!
-   Variables
-   Control flow
-   Conditionals


# Meet Jason

![img](https://i.imgur.com/izC5WWA.jpg)

-   Jason Gibson
    -   Junior, Computer Science (Threads: Devices, Intelligence)
    -   Inside RoboJackets: Training Chair, IGVC Software Lead
    -   Outside RoboJackets: Avid lover of dad jokes
    -   Master of cringe-worthy pictures
-   How to contact me
    -   Slack: [@jasongibson](https://robojackets.slack.com/messages/@jasongibson/)
    -   Email: [jgibson37@gatech.edu](mailto:jgibson37@gatech.edu)


# What is programming?

<div class="NOTES">
Talk for a minute or two about everything programming can do - Fully understanding a problem, analyzing that problem, determining recurrent actions, and then optimizing those actions through logical constructs.

</div>


# What programming is not

<div class="NOTES">
This is meant to emphasize that they will not become a God programmer overnight and to dispell the misconception that they are going to be coding amazing things immediately.

</div>

-   Impossible
-   Easy


# Programming failures

<div class="NOTES">
1.  Talk for a minute or two about everything programming can do. 2. Orbiter was pounds instead of newtons
2.  Ariane software testing did not acccount for increased speed and tried too large a number.

</div>

-   Mars Climate Orbiter (1998)
    -   The code returned the wrong units causing a 655 million dollar explosion on Mars
-   Ariane 5 flight 501
    -   Faster speed resulted in a 64 bit number being stored as 16 bits, ~500 million dollar failure


# CodePad

-   Today we will be using CodePad to write and compile code
    -   This is a remote interview tool
-   Go to <https://codepad.remoteinterview.io>


# Change to C++ - the superior language

![img](https://i.imgur.com/S1PecNy.png)


# Hello World!

-   Add **World!** to your code

```C++
1  #include <iostream>
2  using namespace std;
3  
4  int main() {
5    cout << "Hello World!";
6    return 0;
7  }
```

    Hello World!


# Click Run

![img](https://i.imgur.com/caYg2AY.png)


# {}

-   Curly braces define scope of a function in the code
-   Right now, that means that your code goes in the main function
-   All compound or control flow statements have curly braces
    -   inside the **{}** defines the main method

```C++
1  int main() {
2    // <==== HERE =====
3  }
```


# ;

<div class="NOTES">
Make sure to note that they will see this error a lot and that it will go away with time.

</div>

-   A semicolon defines the end of a statement of code
-   Think of it like a period in a sentence
-   All expression statements end in a semicolon
    -   Declarations, assignments, function calls, etc
    -   A missing semicolon results in an error message that can point to the wrong line

```C++
1  #include <iostream>
2  using namespace std;
3  int main() {
4    cout << "Hello" // <=====
5    cout << "World!";
6    return 0;
7  }
```

    /temp/file.cpp: In function ‘int main()’:
    /temp/file.cpp:5:5: error: expected ‘;’ before ‘cout’
         cout << "World!";
         ^
    
    Compilation Failed


# Looking back at Hello World!

-   Our code is in `{}`
-   `cout` tells the computer to write the letters to the screen
-   `<<` tells the computer what to write out
-   Line 1: includes a header file (covered later)
-   Line 4: defines the main function
-   Line 6: tells the computer that the program ran successfully

```C++
1  #include <iostream>
2  using namespace std;
3  
4  int main() {
5    cout << "Hello World!";
6    return 0;
7  }
```

    Hello World!


# Most useful program ever

<div class="NOTES">
This should naturally lead into why variables are useful. Without them all programs would be deterministic.

</div>

```C++
1  int main() {
2    cout << 2 + 2;
3    return 0;
4  }
```

    4


# Variables

-   Alias for a value
-   The value can be changed without changing the alias


# Primitive Variable Types

| Name     | Description                         |
|-------- |----------------------------------- |
| `bool`   | Can be either **true** or **false** |
| `char`   | a character like 'c' or '+'         |
| `int`    | A whole number \*bounded            |
| `double` | a decimal number \*bounded          |


# What makes a variable primitive?


# Primitive variables?

![img](https://i.imgur.com/Wo0ovX5.jpg)

-   But actually&#x2026;it means that it cannot be divided into smaller parts
-   A number is just a number


# Important symbols

<div class="NOTES">
No lengthy explanation of << and stdout

</div>

| Symbol | Meaning                                                              |
|------ |-------------------------------------------------------------------- |
| `//`   | Signifies a comment - the computer will not try to compile that line |
| `=`    | Assignment operator - value of right is assigned to variable on left |
| `<<`   | Sends something to the OS which displays it in the terminal          |


# Math operators

<div class="NOTES">
explain that the table is in the order of operations

</div>

| Symbol | Meaning                     |
|------ |--------------------------- |
| `()`   | Groups a statement          |
| `*`    | Multiplies two numbers      |
| `/`    | Divides a number by another |
| `+`    | Adds two numbers together   |
| `-`    | Subtracts two numbers       |


# Variable syntax

```C++
1  bool var = true;
2  char character = 'c';
3  int intVar = 2;
4  double double_var = 2.2647;
```


# Variables

-   `my_var` now represents the value 2

```C++
1  int main() {
2    int my_var = 2;
3    cout << my_var + my_var;
4    return 0;
5  }
```

    4


# Declaration vs Definition

```C++
1  // creates the variable num of type int and assigns it the value 1
2  int num = 1;
3  
4  // the type is already known from above so this line just assigns it the value 4
5  num = 4;
```


# Variables

-   Variables can have their assigned value changed on the fly

```C++
1  int main() {
2    int my_var = 2;
3    cout << my_var + my_var;
4    my_var = 4;
5    cout << my_var + my_var;
6    return 0;
7  }
```

    48


# Questions?

<div class="NOTES">
Write a couple examples of setting up a primitive already described and then changing it using only the operators above. like printing numbers 1 - 10 using a var. DO NOT use strings, ++, &#x2013;, or anything else not covered already

</div>

-   Still more to come&#x2026;
-   Let's write some code


# If statements

-   What **IF** I only want to execute code sometimes?


# If statement syntax

```C++
1  int main() {
2    if (true) {
3        // <==== This executes =====
4    }
5    if (false) {
6        // <==== This does *NOT* execute =====
7    }
8  }
```


# Equivalence operators

| Symbol | Condition                                                    | True Example | False Example |
|------ |------------------------------------------------------------ |------------ |------------- |
| `!`    | opposite of current value                                    | `!false`     | `!true`       |
| `==`   | "equal"                                                      | `2 == 2`     | `4 == 2`      |
| `!=`   | **NOT** "equal"                                              | `2 != 4`     | `2 != 2`      |
| `>`    | the number of the left is larger and not equal to the right  | `4 > 2`      | `2 > 4`       |
| `<`    | the number of the left is smaller and not equal to the right | `2 < 4`      | `4 < 2`       |


# If else syntax

```C++
 1  int main() {
 2    if (true) {
 3        // <==== This executes =====
 4    } else {
 5        // <==== This does *NOT* execute =====
 6    }
 7    if (false) {
 8        // <==== This does *NOT* execute =====
 9    } else {
10        // <==== This executes =====
11    }
12  }
```


# Conditional practice 1 question

<div class="NOTES">
1

</div>

-   1 or 2?

```C++
1  if (2 == 2) {
2      // 1
3  } else {
4      // 2
5  }
```


# Conditional practice 2 question

<div class="NOTES">
2

</div>

-   1 or 2?

```C++
1  if (2 != 2) {
2      // 1
3  } else {
4      // 2
5  }
```


# Conditional practice 3 question

<div class="NOTES">
2

</div>

-   1 or 2?

```C++
1  if (2 < 2) {
2      // 1
3  } else {
4      // 2
5  }
```


# Chaining conditionals

<div class="NOTES">
Talk about how this is difficult to write at deeper levels, Think 5-6 Conditionals

</div>

```C++
 1  int main() {
 2    if (true) {
 3        // <==== This executes =====
 4        if (true) {
 5  	  // <==== This executes =====
 6        }
 7        // <==== This executes =====
 8    }
 9    // <==== This executes =====
10  }
```


# Logical operators - AND

| a     | b     | a && b |
|----- |----- |------ |
| True  | True  | True   |
| True  | False | False  |
| False | True  | False  |
| False | False | False  |


# Logical operators - OR

| a     | b     | a &vert;&vert; b |
|----- |----- |---------------- |
| True  | True  | True             |
| True  | False | True             |
| False | True  | True             |
| False | False | False            |


# Logical operators order

-   Statements will be evaluated from left to right
-   No limit to the number you can have
-   Statements can be grouped using `()`
    -   Just like with math, operators `()` are executed first
    -   `1 && (2 || 3)`
        -   `2 || 3` is done first and the result is ANDed with 1


# Conditional practice 4 question

<div class="NOTES">
2

</div>

-   1 or 2?

```C++
1  if (2 < 2 && 2 == 2) {
2      // 1
3  } else {
4      // 2
5  }
```


# Conditional practice 4 question

<div class="NOTES">
1

</div>

-   1 or 2?

```C++
1  if (2 != 2 || 2 == 2) {
2      // 1
3  } else {
4      // 2
5  }
```


# Loops!

-   What if I want to do something multiple times?


# While loops syntax

```C++
1  while (condition == true) {
2      // do something
3  }
```


# For loop syntax

```C++
1  for (initializer; condition; change condition) {
2      // do something
3  }
```


# Math operators continued

```C++
1  var = var + 1;
2  // equivalent to
3  var++;
4  
5  var = var - 1;
6  // equivalent to
7  var--;
```


# Equivalent while and for loop

```C++
1  int while_counter = 10;
2  while (while_counter > 0) {
3      // do something
4      while_counter = while_counter - 1;
5  }
6  for (int for_counter = 10; for_counter > 0; for_counter++) {
7      // do something
8  }
```


# Let's write some code!

<div class="NOTES">
Write a couple examples of code using what has already been covered. Multiplication Table Converting Time s->m->h etc Find length of a number 99 bottles

</div>


# Questions?

-   A go-to place to ask questions ([Piazza!](https://piazza.com/gatech/fall2017/rjsw/home))
-   Monitored by highly experienced RoboJackets software members