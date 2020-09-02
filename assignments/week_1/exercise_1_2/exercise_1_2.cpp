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
