#pragma once

#include <iostream>

#include "animal.h"

class Prey : public Animal {
public:
    Prey(std::string name) : Animal(name) {};
	void say_hello();
	virtual void do_something();
};
