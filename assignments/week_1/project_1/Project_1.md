# Project 1
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

#TODO more example of convolutions in the real world

```
Vector {1,2,3,4,5,6,7}
Kernel {2,5,6}
```

In this example the array we are convolving over is x, the kernel is w and the output vector is y.
There should be an immediate question, what do we do when we are calculating the value at y 0 or 6.
In that case you should use 0 for the missing data. For example calculating y at 0 would be

```
Y[0] = x[-1]w[0] + x[0]w[1] + x[1]w[2] = 0x2 + 1x5 + 2x6 = 17
```

Here are some example solutions to the above convolution example

```
Y[3] = 56
```
```
Y[6] = 47
```

# Start Code
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
    //std::cout << "prev_location: " << prev_location << std::endl;
    //std::cout << "next_location: " << next_location << std::endl;
    // substr [,]
    //std::cout << s.substr(prev_location, next_location - prev_location) << std::endl;
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


  std::cout << "x: {" << x[0];
  for(int i = 1; i < x.size(); i++) {
    std::cout << ", " << x[i];
  }
  std::cout << "}" << std::endl;

  std::cout << "w: {" << w[0];
  for(int i = 1; i < w.size(); i++) {
    std::cout << ", " << w[i];
  }
  std::cout << "}" << std::endl;

  return 0;

}

```

# Implementation
Now take the main.cpp file and create your own version of applying this kernel.
Print out the result Y vector in the format `{Y[0], Y[1], Y[2], Y[3], Y[4], Y[5], Y[6]}`

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
Y[0] = x[-1]w[0] + x[0]w[1] + x[1]w[2] = 1x2 + 1x5 + 2x6 = 19
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
