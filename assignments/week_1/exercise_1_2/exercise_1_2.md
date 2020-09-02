# Exercise 1.2
Welcome to your second exercise of RoboJackets software training! This exercise will be testing your knowledge of basic C++ syntax and array manipulation. For this exercise we expect you to have watched or know the content of the Codecademy C++ course up through vectors, as well as our own video about types.

# Exercise Objective
In this exercise, you will be reversing an array and checking if a string is a palindrome. The purpose of this exercise is to get you familar with working with arrays. This exercise should help solidify your knowledge of loops, variables, conditionals, and arrays.

# Starting Code
We have already given you a main() function to write your code in and have included the proper libraries for you to use. We have also set up most of variable initilization and commented where you will write your code.

```c++
// Iterative C++ program to reverse an array and check if string is palindrome
#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace std;

int main()
{
    // Part 1: Array Reversal
    int arr[] = {1, 2, 3, 4, 5, 6};
    int n = sizeof(arr) / sizeof(arr[0]);

    // Print original array
    cout << "Original Array: ";
    for (int i = 0; i < n; i++)
        cout << arr[i] << " ";
    cout << endl;

    // Reverse Array
    // WRITE YOUR STRING REVERSAL HERE

    // Print the Reversed array
    cout << "Reversed Array: ";
    for (int i = 0; i < n; i++)
        cout << arr[i] << " ";
    cout << endl;


    // Part 2: Palindrome Check
    char str[] = "abbcdcbba";

    // Palindrome Check
    // WRITE YOUR PALINDROME CHECK HERE

    return 0;
}
```

#### Array Reversal
1. Create variables for the array's `start` and `end` indices
<details>
  <summary>Answer</summary>

  ```c++
    int start = 0;
    int end = n - 1;
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

4. In the loop, swap the elements at `start` and `end` with each other so `arr[start]` -> `arr[end]` and `arr[end]` -> `arr[start]`
<details>
  <summary>Answer</summary>

  ```c++
    while (start < end)
    {
        int temp = arr[start];
        arr[start] = arr[end];
        arr[end] = temp;
        start++;
        end--;
    }
  ```

</details>

Now try to run the program and test that the array gets reversed!


#### Palindrome Check
1. Create variables for the array's leftmost and rightmost indices (`left` and `right`)
<details>
  <summary>Answer</summary>

  ```c++
    int left = 0;
    int right = strlen(str) - 1;
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
        cout << str << " is a palindrome" << endl;
    else
        cout << str << " is NOT a palindrome" << endl;
  ```

</details>

Now run the program and test multiple strings to check your program works!
