# Exercise 1.2
Welcome to your second exercise of RoboJackets software training! This exercise will be testing your knowledge of basic C++ syntax and vector manipulation. For this exercise we expect you to have watched or know the content of the Codecademy C++ course up through vectors.

# Exercise Objective
In this exercise, you will be reversing an vector and checking if a string is a palindrome. The purpose of this exercise is to get you familar with working with vectors and strings. This exercise should help solidify your knowledge of loops, variables, conditionals, and vectors. _We aren't going to be using any new C++ standard library functions in the instructions, but feel free to use them if you want._

# Starting Code
We have already given you a main() function to write your code in and have included the proper libraries for you to use. We have also set up most of variable initilization and commented where you will write your code.


### Part 1: Vector Reversal
```c++
#include <iostream>
#include <vector>

int main()
{
    // Part 1: Vector Reversal

    // Input vector from command-line
    std::vector<int> list;
    int input;
    std::cout << "Input List Of Numbers (end list with non-integer):" << std::endl;
    while (std::cin >> input)
        list.push_back(input);

    // Print original vector
    std::cout << "Original Vector: ";
    for (int v : list)
        std::cout << v << " ";
    std::cout << std::endl;

    // Reverse Vector
    // WRITE YOUR VECTOR REVERSAL HERE

    // Print the Reversed vector
    std::cout << "Reversed Vector: ";
    for (int v : list)
        std::cout << v << " ";
    std::cout << std::endl;

    return 0;
}
```

1. Create variables for the vector's `start` and `end` indices
<details>
  <summary>Answer</summary>

  ```c++
    int start = 0;
    int end = list.size() - 1;
  ```

</details>

2. Create a loop which continues as long as `start` is less than `end`
<details>
  <summary>Answer</summary>

  ```c++
    while (start < end)
    {
        // MORE CODE
    }
  ```

</details>

3. In the loop, increment `start` and decrement `end` so that `start` will iterate forwards over the first half and `end` will iterate backwards over the second half
<details>
  <summary>Answer</summary>

  ```c++
    while (start < end)
    {
        // MORE CODE

        start++;
        end--;
    }
  ```

</details>

4. In the loop, swap the elements at `start` and `end` with each other so `list[start]` -> `list[end]` and `list[end]` -> `list[start]`
<details>
  <summary>Answer</summary>

  ```c++
    while (start < end)
    {
        int temp = list[start];
        list[start] = list[end];
        list[end] = temp;
        start++;
        end--;
    }
  ```

</details>

Now try to run the program and test that the vector gets reversed!


### Part 2: Palindrome Check
```c++
#include <iostream>
#include <vector>

int main()
#include <iostream>

int main()
{
    // Part 2: Palindrome Check

    // Input string from command-line
    std::string str;
    std::cout << "Ender String:" << std::endl;
    std::cin >> str;

    // Palindrome Check
    // WRITE YOUR PALINDROME CHECK HERE

    return 0;
}
```

1. Create variables for the string's leftmost and rightmost indices (`left` and `right`)
<details>
  <summary>Answer</summary>

  ```c++
    int left = 0;
    int right = str.length() - 1;
  ```

</details>

2. Create a variable (`isPalindrome`) that will store the result of the palindrome check
<details>
  <summary>Answer</summary>

  ```c++
    bool isPalindrome = true;
  ```

</details>

3. Create a loop which continues as long as `left` is less than `right`
<details>
  <summary>Answer</summary>

  ```c++
    while (left < right)
    {
        // MORE CODE
    }
  ```

</details>

4. In the loop, check if the elements at `left` and `right` in the string are different
<details>
  <summary>Answer</summary>

  ```c++
    while (left < right)
    {
        if (str[left] != str[right])
        {
            // MORE CODE
        }
    }
  ```

</details>

5. If the elements are different, then the string should be marked are not a palindrome and you should leave the loop
<details>
  <summary>Answer</summary>

  ```c++
    while (h > l)
    {
        if (str[l] != str[h])
        {
            isPalindrome = false;
            break;
        }
    }
  ```

</details>

6. In the loop, increment `left` and decrement `right` so that `left` will iterate forwards over the first half and `right` will iterate backwards over the second half
<details>
  <summary>Answer</summary>

  ```c++
    while (left < right)
    {
        if (str[left] != str[right])
        {
            isPalindrome = false;
            break;
        }
        left++;
        right--;
    }
  ```

</details>

7. Print out the results (ex. <str\> is (NOT) a palindrome)
<details>
  <summary>Answer</summary>

  ```c++
    if (isPalindrome)
        std::cout << str << " is a palindrome" << std::endl;
    else
        std::cout << str << " is NOT a palindrome" << std::endl;
  ```

</details>

Now run the program and test multiple strings to check your program works!

# Review
Great Job! You successfully manipulated a vector and string in this exercise. Although, these intructions explains one method to do vector reversal and palindrome
checking, there exists much easier ways to do these operations using C++ standard library functions. For example, `std::reverse(list.begin(), list.end());` would
reversal the vector in just one line without any hassle! Also, `str == string(str.rbegin(), str.rend())` would check if `str` is a palindrome in one line as well.
The standard library is powerful tool for writing C++ code and as you progress through future weeks' exercises you become more and more familiar with it!