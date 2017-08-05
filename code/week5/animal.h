#pragma once

#include <iostream>

class Animal {
private:
    std::string name;
public:
    // prototype for constructor. Called when the object is constructed.
    Animal(std::string name);

    // virtual destructor the ensure that it will be called by subclasses
    virtual ~Animal() {std::cout << "Animal destructed" << std::endl;};

    // virtual method that can be overriden by subclasses
    virtual void do_something();

    // basic class methods
    void say_hello();
    void say_your_name();

    // getter and setter for name
    std::string get_name();
    void set_name(std::string name);
};
