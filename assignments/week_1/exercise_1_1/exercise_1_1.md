# Exercise 1.1
Welcome to your first exercise of RoboJackets software training! Exercises are shorter and simpler assignments where you will be walked through how to write certain code. This exercise will be testing your knowledge of basic C++ syntax as well as basic programming concepts. For this exercise we expect you to have watched or know the content of the Codecademy C++ course up through vectors.

# Exercise Objective
In this exercise you will be controlling the movements of a car like robot with 4 wheels via the command line. The robot looks like a simple car with 4 wheels, each numbered as shown below. All 4 wheel speeds are controlled independently of each other and can move either forward (positive speed) or backward (negative speed). Since the wheels do not pivot, to turn left the left wheels must be moving slower than the right wheels. Similarly to turn right the right wheels must be going slower than the left wheels. The goal is to create a system that will let the robot move forward, backward, and turn left or right while moving forward. Every time the direction of the robot is inputted by the user, the robot should print out the speed each wheel is going. The robot should also continuously check for user input until it is told to stop by the user. 

![alt text](exercise_1.1_robot.PNG)

# Starting Code
We have already given you a main() function to write your code in and have included the proper libraries for you to use. We have also set up most of the while loop where you will write most of your code; the keep_going boolean will be used to trigger then end of the while loop, so you will need to change its value at some point. We have also set up the user input which is stored in the char variable, direction; this is the variable that you need to check for what was inputted.

```c++
//Do not edit the next 4 lines
#include <iostream>
#include <vector>

int main() {
    std::cout << "Use the wasd keys and enter to input a direction, or the E key to stop" << std::endl;

    // Write your code for step 1 here

    // Write your code for step 2 here
   
    // Do not edit the next 5 lines
    bool keep_going = true;
    char direction = 0;

    while (keep_going) {
        std::cin >> direction; //At the begining of every iteration look for an input from the user
        // Write rest of code here (within while loop)
    }

}
```
1. Create a vector of 4 floats called wheel_speeds
<details>
  <summary>Answer</summary>
  
  ```c++
  std::vector<float> wheel_speeds(4);
  ```
  
</details>

2. Set each float in wheel_speeds to 0 using square brackets ```[]``` to access each index
<details>
  <summary>Answer</summary>
  
  ```c++
  wheel_speeds[0] = 0;
  wheel_speeds[1] = 0;
  wheel_speeds[2] = 0;
  wheel_speeds[3] = 0;
  ```
  
</details>

3. In the while loop create a set else if statements for each case. Remember to include one to check if direction is ‘w’,’a’,’s’, ’d’,‘e’ or none of those things. You can just use ```==``` to check if direction is equal to a certain character.
<details>
  <summary>Answer</summary>
  
  Note that the order of what you check does not matter, as long as the if statement is frist, the else statement is last and the else ifs are in between.
  
  ```c++
  if (direction == ‘w’) {}
  else if (direction == ‘a’) {}
  else if (direction == ‘s’) {}
  else if (direction == ‘d’) {}
  else if (direction == ‘e’) {}
  else {}
  ```
  
</details>

4. Within the else if statement for ‘w’, create a for loop which sets each float in wheel_speeds to 1. The for loop should count from 0 to 3 and use the counter with square brackets as the index for wheel_speeds.
<details>
  <summary>Answer</summary>
  
  ```c++
  for (int i = 0; i < 4; i++) {
    wheel_speeds[i] = 1; 
  }
  ```
  
</details>

5. Within the else if statement for ‘a’, set the floats at indices 0 and 2 to 0.5 and set the floats at indices 1 and 3 to 1 in the wheel_speeds vector. Do this by individually accessing each index using square brackets ```[]```.
<details>
  <summary>Answer</summary>

  ```c++
  wheel_speeds[0] = 0.5;
  wheel_speeds[2] = 0.5;
  wheel_speeds[1] = 1;
  wheel_speeds[3] = 1;
  ```
  
</details>

6. Within the else if statement for ‘s’, create a for loop which sets each float in wheel_speeds to -1.
<details>
  <summary>Answer</summary>

  ```c++
  for (int i = 0; i < 4; i++) {
    wheel_speeds[i] = -1;
  }
  ```
  
</details>

7. Within the if else statement for ‘d’, set the floats at indices 1 and 3 to 0.5 and set the floats at indices 0 and 2 to 0.5 in the wheel_speeds vector.
<details>
  <summary>Answer</summary>

  ```c++
  wheel_speeds[0] = 1;
  wheel_speeds[2] = 1;
  wheel_speeds[1] = 0.5;
  wheel_speeds[3] = 0.5;
  ```
  
</details>

8. Within the if else statement for ‘e’, create a for loop which sets all wheel speeds to 0, then create a print statement that prints “Shutting Down” and set the the keep_going boolean to false to stop the while loop from continuing.
<details>
  <summary>Answer</summary>

  ```c++
  for (int i = 0; i < 4; i++) {
    wheel_speeds[i] = 0;
  }
  std::cout << "Shutting down" << std::endl;
  keep_going = false;
  ```
  
</details>

9. Within the else statement create a print statement which prints “Sorry, I didn’t get that. Try again.” on a line by itself.
<details>
  <summary>Answer</summary>

  ```c++
  std::cout << "Sorry, I didn't get that.  Try again." << std::endl;
  ```
  
</details>

10. After all else statement but still within the while loop create a for loop that print all of the wheel speeds on a separate line in the following format: “Wheel at index [index] has speed [speed at that index]”
<details>
  <summary>Answer</summary>

  ```c++
  for (int i = 0; i < 4; i++) {
    std::cout << "wheel at index " << i << " has speed: " << wheel_speeds[i] << std::endl;
  }
  ```
  
</details>

# Expected Behavior
When you run the program you should be able to type in a ‘w’,’a’,’s’, or ‘d’ into the command line and hit enter, which should prompt all the wheels to change speeds accordingly and print out the correct new wheel speeds. Typing in an ‘e’ should stop the program, and any other single character input should output the error message described previously.
