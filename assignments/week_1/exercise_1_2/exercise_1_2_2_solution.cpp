#include <iostream>

int main()
{
    // Part 2: Palindrome Check

    // Input string from command-line
    std::string str;
    std::cout << "Enter String:" << std::endl;
    std::cin >> str;

    // Palindrome Check
    // WRITE YOUR PALINDROME CHECK HERE
    int left = 0;
    int right = str.length() - 1;

    bool is_palindrome = true;
    while (left < right)
    {
        if (str[left] != str[right])
        {
            is_palindrome = false;
            break;
        }
        left++;
        right--;
    }

    if (is_palindrome)
    {
        std::cout << str << " is a palindrome" << std::endl;
    }
    else
    {
        std::cout << str << " is NOT a palindrome" << std::endl;
    }
    return 0;
}
