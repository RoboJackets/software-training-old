#pragma once

#include <iostream>

#include "animal.h"

class Predator : public Animal {
 public:
 Predator(std::string name) : Animal(name) {};
	void say_hello();
	virtual void do_something();
};
