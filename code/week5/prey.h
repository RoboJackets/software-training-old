#pragma once

#include <iostream>

#include "animal.h"

// subclass of Animal
class Prey : public Animal {
public:
    // constructor. Calls the animal constructor with name
    Prey(std::string name) : Animal(name) {std::cout <<
    "constructor of Prey called" << std::endl;};

    // destructor
    ~Prey() {std::cout << "Prey, ";}

    // this will be called instead of the say_hello method from animal
    void say_hello();

    // overrides the do_something method from animal and will be called
    // even if a Prey is referenced as a Animal
    // override annotation has the compiler check that the method in overriden
    // correctly i.e. this prototype matches one in Animal of the same name.
    void do_something() override;
};
