#include <iostream>  // Gives us access to std::cin and std::cout
#include <string>  // Gives us access to std::string and std::getline()

// TODO Define your light controller state machine class here

int main()
{
    // TODO Initialize your state machine class here
    while(true)
    {
        std::string input;
        std::getline(std::cin, input);
        if(input.size() != 2)
            break;

        // TODO Parse the input digits

        // TODO Update your state machine based on input

        // TODO Output your state machine's light controls to std::cout
    }
    return 0;
}