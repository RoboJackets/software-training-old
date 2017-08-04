#pragma once

#include <iostream>

class Animal {
	std::string name;
public:
	// prototype for constructor. Called when the object is constructed.
	Animal(std::string name);

	// virtual method
	virtual void do_something();

	//  two generic class methods
	void say_hello();
	void say_your_name();

	// getter and setter for name
	std::string get_name();
	void set_name(std::string name);
};
