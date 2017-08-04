#pragma once

#include <iostream>

#include "animal.h"

// subclass of Animal
class Predator : public Animal {
public:
	// constructor 
    Predator(std::string name) : Animal(name) {};
	
	// overwrites the say_hello method from animal
	void say_hello();

	// overwrites the do_something method from animal and will be called
	// even if a Predator is referenced as a Animal
	// override annotation has the compiler check that the method in overriden
	// correctly i.e. this prototype matches one in Animal of the same name.
	virtual void do_something() override;
};
