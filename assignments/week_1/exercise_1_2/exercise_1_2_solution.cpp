// Iterative C++ program to reverse an array and check if string is palindrome
#include <stdio.h>
#include <string.h>
#include <iostream>

int main()
{
    // Part 1: Array Reversal
    int arr[] = {1, 2, 3, 4, 5, 6};
    int n = sizeof(arr) / sizeof(arr[0]);

    // Print original array
    std::cout << "Original Array: ";
    for (int i = 0; i < n; i++)
        std::cout << arr[i] << " ";
    std::cout << std::endl;

    // Reverse Array
    // WRITE YOUR STRING REVERSAL HERE
    int start = 0;
    int end = n - 1;

    while (start < end)
    {
        int temp = arr[start];
        arr[start] = arr[end];
        arr[end] = temp;
        start++;
        end--;
    }

    // Print the Reversed array
    std::cout << "Reversed Array: ";
    for (int i = 0; i < n; i++)
        std::cout << arr[i] << " ";
    std::cout << std::endl;

    // Part 2: Palindrome Check
    char str[] = "abbcdcbba";

    // Palindrome Check
    // WRITE YOUR PALINDROME CHECK HERE
    int left = 0;
    int right = strlen(str) - 1;

    bool isPalindrome = true;
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

    if (isPalindrome)
        std::cout << str << " is a palindrome" << std::endl;
    else
        std::cout << str << " is NOT a palindrome" << std::endl;

    return 0;
}
