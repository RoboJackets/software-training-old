#pragma once

#include <iostream>

#include "animal.h"

class Mammal : public Animal {
 public:
	void say_hello();
	virtual void do_something();
};
