#pragma once

#include <iostream>

class Animal {
	std::string name;
public:
	Animal(std::string name);
	void say_hello();
	virtual void do_something();
	void say_your_name();
	std::string get_name();
	void set_name(std::string name);
};
