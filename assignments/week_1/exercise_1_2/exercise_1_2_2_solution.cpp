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
    int left = 0;
    int right = str.length() - 1;

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
