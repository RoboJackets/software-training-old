# Project 1.3
Welcome to the first project for software training. Projects are more involved than
exercises. This is taking what you learn in the videos and exercises and applying it
to a nontrivial task. At this point you should have finished all the videos and exercises
for the first section.

# Project Objective
During this project you will code your very own 1D convolution on a vector.
1D convolutions are used for smoothing and signal processing in robotics.
2D convolutions are used frequently in image filtering, but you will just be
coding up a 1D filter for today.

## Project Background
The components of a convolution are two things, the array you will be
convolving over and the kernel that will be used. At each step we will calculate
 a new spot in the resulting vector using the kernel and a subsection of
the array you are convolving over.

```
Vector {1,2,3,4,5,6,7}
Kernel {2,5,6}
```

In this example the array we are convolving over is x, the kernel is w and the output vector is y.
There should be an immediate question, what do we do when we are calculating the value at y 0 or 6.
In that case you should use 0 for the missing data. For example calculating y at 0 would be

```
Y[0] = x[-1]w[0] + x[0]w[1] + x[1]w[2] = 0x2 + 1x5 + 2x6 = 17
Y[1] = x[0]w[0] + x[1]w[1] + x[2]w[2] = 1x2 + 2x5 + 3x6 = 30
Y[2] = x[1]w[0] + x[2]w[1] + x[3]w[2] = 2x2 + 3x5 + 4x6 = 43
```

Here are some example solutions to the above convolution example

```
Y[3] = 56
```
```
Y[6] = 47
```

<img src="https://miro.medium.com/max/2340/1*Fw-ehcNBR9byHtho-Rxbtw.gif" width="1170" height="849" />

# Starter Code
You are given the following as starter code. You are given some basic code to read from cin.

```c++
#include <iostream>
#include <vector>

std::vector<double> readInVector(std::string s) {
  int prev_location = 0;
  int next_location = 0;
  std::vector<double> result;
  while(s.find(',', prev_location) != std::string::npos) {
    next_location = s.find(',', prev_location);
    result.push_back(std::stod(s.substr(prev_location, next_location - prev_location)));
    next_location++;
    prev_location = next_location;
  }
  result.push_back(std::stod(s.substr(prev_location, std::string::npos)));
  return result;
}

int main() {
  std::vector<double> x;
  std::vector<double> w;
  std::vector<double> y;
  bool pack_with_zeros = true;

  std::string s;
  std::cin >> s;
  if(s == "false") {
    pack_with_zeros = false;
  }
  std::cin >> s;
  x = readInVector(s);
  std::cin >> s;
  w = readInVector(s);

  return 0;

}

```

# Implementation
Now take the main.cpp file and create your own version of applying this kernel.
Print out the result Y vector in the format `{Y[0], Y[1], Y[2], ..., Y[Y.size()-2], Y[Y.size()-1]}`

Let's print out the input and output vectors in the format we are looking for.
You need to run the command
```bash
cat ../example1.txt | ./project_1_3
```

You should write to see the output below

```bash
x: {1,2,3,4,5,6,7}
w: {2,5,6}
y: {}
```

Try to write your code so that the size of the vectors can be changed and
your code will continue to work. If you are really clever you will not have to
change the code printing y even as begin to populate the vector.

## Example 1

Your output for this section should be

```
{17, 30, 43, 56, 69, 82, 47}
```

### How To Run Example 1
make sure you are in your build folder
```bash
cat ../assignments/week_1/project_1/example1.txt | ./assignments/week_1/project_1/project_1
```

### Tips:
Think about writing your code in a way that will work on any size input array or any size kernel

If you are seeing an error on access into y, think about what the size of the vector is.
Do we have the memory allocated in order to access that location yet?

## Example 2
```
Vector {1,2,3,4,5,6,7,8,9}
Kernel {2,5,6,-1,-2}
```

Print out your result using the same format as before, you should see

```
{-2, 6, 16, 26, 36, 46, 56, 86, 108}
```

### How To Run Example 2
```bash
cat ../assignments/week_1/project_1/example2.txt | ./assignments/week_1/project_1/project_1
```

## Implementation Part 2
Now what about is we change how we are packing the convolutional when we are missing
data on the array we are convolving over. Try using the closest array index, therefore using our
example from earlier we would have

```
Vector {1,2,3,4,5,6,7}
Kernel {2,5,6}
```

```
Y[0] = x[0]w[0] + x[0]w[1] + x[1]w[2] = 1x2 + 1x5 + 2x6 = 19
Y[1] = x[0]w[0] + x[1]w[1] + x[2]w[2] = 1x2 + 1x5 + 2x6 = 30
Y[6] = x[5]w[0] + x[6]w[1] + x[6]w[2] = 6x2 + 7x5 + 7x6 = 89
```

## Example 3

Print out your result using the same format as before, you should see

```
{19, 30, 43, 56, 69, 82, 89}
```

### Tip
Try to change your code so that you can use either method of packing based off of some variable.
We have provided you with a boolean in the starter code, use that.


### How To Run Example 3
```bash
cat ../assignments/week_1/project_1/example3.txt | ./assignments/week_1/project_1/project_1
```
