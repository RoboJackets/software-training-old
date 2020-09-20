# Exercise 2.3
In this exercise we will be working on converting between different representations
of numbers. Likely you have heard of binary and hexidecimal. Our goal is to write
a class that will allow you to convert between the different types of numbers.

# Background
There are a lot of different representations of numbers. The standrd way we represent
numbers are in base 10, the digits 0-9 are used in sequence to represent larger numbers.
You could write out a large number like 25 as the following

```
10^0 * 5 + 10^1*2 = 5 + 20 = 25
```

We will be using the same approach but with different base numbers in this exercise.

## Binary

Binary is when you represent the number as only {1,0} this is a base two representation.
If we take the number

```
11001
```

that can bet written out as

```
1 * 2^0 + 0*2^1 + 0*2^2 + 1*2^3 + 1*2^4 = 1 + 0 + 0 + 8 + 16 = 25
```

## Hex

Hex has a representation of numbers in base 16, now we only have 10 digits to represent numbers
with, so we will need to include some letters to represent the actual number.
Here we use the letters {a,b,c,d,e,f} to represent {10,11,12,13,14,15,16}.
All hex numbers are prefeced with **0x** like below1

```
0x19 = 9 * 16^0 + 1*16^1 = 9 + 16 = 25
```

# Implementation

Today we will be writing a class from scratch in order to convert between the
different formats of the numbers. Furthermore, we will be keeping track of
how many conversions we do between each type inside out class. During this implementation
we will be keeping it simple so you will not have to worry about the 0x or b prepend to show
what kind of number you have.

## Starter Code
There are two methods that have been implemented for you, they are below.
They use the actual ASCII numbers of the different characters to convert
hex to digits and digits to hex characters. Don't worry about how this works.
Here we name out class NumberConverter, you can name it whatever you want.

```c++
/**
 * returns a number 0-15 based off of a hex char
 * This uses ascii to figure out number it corresponds to
 * @param input
 * @return
 */
int NumberConvereter::hexCharToNumber(char input) {
  // this means it is [A,B,C,D,E,F]
  if(input > 57) {
    return input - (65 - 10);
  } else {
    // this is a digit in ascii
    return input - 48;
  }
}

/**
 * returns the character that is the given hex number
 * only valid on 0-15
 * @param input
 * @return
 */
char NumberConvereter::numberToHexChar(int input) {
  if(input < 10) {
    // it is a digit
    return input + 48;
  } else {
    return (input - 10) + 65;
  }
}
```

There are multiple online resources to convert between the three types of numbers.
Here are some known conversions to check your code

## Part 1 To Decimal
Converting from binary and hex to decimal is straightforward. Implement two methods
that take in strings and output an integer that is the decimal representation of
the input. Remember you should be using the hexCharToNumber helper function to
convert each digit. Think about how we were writing out the numbers earlier to help
you along in your implementation.

## Part 2 To Hex
Now we will be writing two methods that convert from binary and decimal to hex.
binary to decimal should take in a string and return a string, decimal to hex should
take in an int and return a string.

### Decimal To Hex Tips

There is an elegant exploit to convert from decimal to hex. Let us say we have the number
```
0x2A7 = 7 * 16^0 + 10 * 16^1 + 2*16^2 = 7 + 160 + 512 = 679
```

What do we get if we use the modulus operator (%)
```
679 % 16 = 7
```

Now that is interesting, that is the left most digit of our hex representation.
Now let us see if we can get the next digit as well, so we divide out the first digit
by taking the entire number and dividing by 16 (remember we have integers so we will
not have any decimals).

```
679 / 16 = 42 // remember integer division lops off the decimals
```

Now we take out mod again

```
42 % 16 = 10 = A
```

Now we can see out algorithm begin to take shape. The summarized steps are

```
679 % 16 = 7
679 / 16 = 42
42 % 16 = 10 = A
42 / 16 = 2
2 % 16 = 2
```
So if we combine out mod operators we end up with 7A2, now the C++ algorithms header
gives us a function to reverse a string that can be called like this (make sure to have the header file.

```c++
#include <algorithm>
// lotta cool code
std::string str = "str";
std::reverse(str.begin(), str.end());
// str = "rts" now
```

### Binary To Hex Tips
Now there is an interesting relationship between binary and hex. Let's use the same
number as before.

```
0x2A7 = 7 * 16^0 + 10 * 16^1 + 2*16^2 = 7 + 160 + 512 = 679
```

Now lets keep in mind that 2^4 = 16 and see if we can use that fact. Let's just look
at the max value hex can represent with a single digit (15), what about 0xF0?

```
0xF = 15 = b1111
0xF0 = 15*16^1 = b11110000
```

in fact all hex digits can be represented in binary using only 4 values. We can
use this fact to go from binary to hex. Let us rewrite each digit of hex in binary and see how this
fits in, we will make sure each digit gets represented with 4 binary digits, we pad with zeros

```
0x7 = 7 = b0111 = 1*2^0 + 1*2^1 + 1*2^2+ 0*2^3 = 1 + 2 + 4 + 0
0xA = 10 = b1010 = 0*2^0 + 1*2^1 + 0*2^2 + 1*2^3 = 0 + 2 + 0 + 8
0x2 = 2 = b0010 = 0*2^0 + 1*2^1 + 0*2^2 + 0*2^3 = 0 + 2 + 0 + 0
```

Now if we take these and put them in sequence we get the binary representation of
the original hex number in binary.

```
0x2A7 = 0010 1010 0111
```

## Part 3 to Binary
Now we will be writing two methods that convert from hex and decimal to binary.
Write two methods with the correct inputs and return type (you should see the pattern by now).

use the previous information to correctly convert from hex and decimal to binary.

## Part 4
use cin to take in a character that represents what type of number to print out. Using
the following convention

```
b = binary
d = decimal
h = hex
```

you will then need to figure out what type of number input you have based on the conventions

```
0x = hex
b = binary
otherwise decimal
```

the input and output for an example run should match what you see below.
You can check your output by running the different example.txt files using the command,
Your output should match

```bash
cat example4_1.txt | ./exercise_2_3
input: 2A7
output type: d
679
decimal_inputs: 0
binary_inputs: 0
hex_inputs: 1
decimal_outputs: 1
binary_outputs: 0
hex_outputs: 0
```
```bash
cat example4_2.txt | ./exercise_2_3
input: A7
output type: b
7
10
10100111
decimal_inputs: 0
binary_inputs: 0
hex_inputs: 1
decimal_outputs: 0
binary_outputs: 1
hex_outputs: 0

```
```bash
cat example4_3.txt | ./exercise_2_3
input: 1011101
output type: d
93
decimal_inputs: 0
binary_inputs: 1
hex_inputs: 0
decimal_outputs: 1
binary_outputs: 0
hex_outputs: 0
```

## Part 5 Keeping Track
Now we want to add some private variables to keep track of the number of different
conversions we have done. You want to know how many of each type have been input and output.


Now you need to implement a constructor that can be used to set the 6 member variables.
Also implement a destructor that prints out the different values. Use the method provided
in the exercise_2_3.cpp in order to parse the first line. Your input will be

```
1,2,3,4,5,6
d
0x2A7
```

your output should be

```
input: 2A7
output type: d
result: 679
decimal_inputs: 1
binary_inputs: 2
hex_inputs: 4
decimal_outputs: 5
binary_outputs: 5
hex_outputs: 6
```

this is for example5_1.txt

Remember to implement your destructor and constructor in the .cpp file of your class

Finally you will need to check that your code works by running the final config files

```bash
cat example5_2.txt | ./exercise_2_3
```

```
input: 2A7
output type: b
result: 7
10
2
001010100111
decimal_inputs: 1
binary_inputs: 3
hex_inputs: 4
decimal_outputs: 77
binary_outputs: 54
hex_outputs: 12
```

```
cat example5_3.txt | ./exercise_2_3
```

```
input: 42152
output type: h
result: A4A8
decimal_inputs: 87
binary_inputs: 75
hex_inputs: 3
decimal_outputs: 49
binary_outputs: 5
hex_outputs: 7
```




