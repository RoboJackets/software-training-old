# Exercise 2.1
Welcome to the first exercise of week 2!

# Project Objective
Today we will be making a quick change to the code that we wrote for Project 1.3
last week. We want to convert the code you wrote into a function so that we can call
if multiple times without copy pasting code.

Below is the solution the instructors wrote for this week. Feel free to work off of this
directly or use your own code.

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

  int packing_size = (w.size()-1)/2;

  for(int i = 0; i < x.size(); i++) {
    double accumulator = 0;
    for(int j = 0; j < w.size(); j++) {

      // Since kernel is filled in with 0's outside does not matter
      if(i - packing_size + j >= 0 && i - packing_size + j < x.size()) {
        accumulator += x[i - packing_size + j]*w[j];
      } else if(!pack_with_zeros && i - packing_size + j < 0) {
        // Handles the case where where we underflow indexes
        accumulator += x[0] * w[j];
      } else if(!pack_with_zeros && i - packing_size + j >= x.size()) {
        // Handles the case where we overflow indexes
        accumulator += x[x.size()-1] * w[j];
      }
    }
    y.push_back(accumulator);
  }

  std::cout << "{" << y[0];
  for(int i = 1; i < y.size(); i++) {
    std::cout << ", " << y[i];
  }
  std::cout << "}" << std::endl;

  return 0;
}

```

# Writing a method
First we will convert it to a method in the same file we already have. You should copy
the above code or your own version into the exercise_2_1.cpp file.

Your method signature should be something similar to

```c++
std::vector<double> applyConvolution(std::vector<double> x, std::vector<double> w, bool pack_with_zeros);
```

Now your main method should just simply call the method.

## Use Methods
Now you should print out the results of calling the convolution with both possible values of
pack_with_zeros. You should use the configured result first then the opposite.

Running example one your output should be

```bash
{17, 30, 43, 56, 69, 82, 47}
{19, 30, 43, 56, 69, 82, 89}
```

# Writing a method in another file
Now we will move this method outside of the current code so that we can use it elsewhere.

## Create a header file

Use this command to create 2 files. All touch does is generate an empty file with the
given name

```bash
touch convolution.cpp
touch convolution.h
```

Now move the method prototype into the convolution.h file, you should see something like this

```c++
std::vector<double> applyConvolution(std::vector<double> x, std::vector<double> w, bool pack_with_zeros);
```

Make sure to not forget the vector header file since you need that in convolution.h

Now move your method definition into the convolution.cpp file.

## Getting it to compile

Now we need to make changes to the cmake file in order to compile the convolution.cpp file
and include convolution.h in the exercise_2_1.cpp.

Use a standard include statement to include convolution.h in exercise_2_1.cpp

<details>
  <summary>Answer</summary>

  ```c++
    #include "convolution.h"
  ```

</details>

Also don't forget your header guards in convolution.h

<details>
  <summary>Answer</summary>

  ```c++
  #pragma once
  ```

</details>

Add convolution.cpp to the compilation TODO based off of what the build style is.

Finally compile and run the code yourself. Your output should be the same as before

```bash
{17, 30, 43, 56, 69, 82, 47}
{19, 30, 43, 56, 69, 82, 89}
```

# Writing another function
We are reusing code to print out the vector at the end. You should write a method
that takes in a std::vector<double> and prints out the vector in the same format.

Your prototype should be something like

```c++
void printVector(std::vector<double> vec);
```

Now use that method to refactor your code. You should revel in the amount of code (and therefore bugs)
you have averted by using methods effectively. If you ever see yourself copy pasting
code, stop and see if you can find a way to reuse the first version.
