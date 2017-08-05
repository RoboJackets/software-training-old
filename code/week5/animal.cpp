#include "animal.h"

// constructor
Animal::Animal(std::string new_name) {
    name = new_name;
    std:: cout << new_name << "\nconstructor of Animal called" << std::endl;
}

void Animal::say_hello() {
    std::cout << "Hello, I am an Animal" << std::endl;
}

void Animal::do_something() {
    std::cout << "generic animals cannot do anything!" << std::endl;
}

void Animal::say_your_name() {
    std:: cout << "My Name is: " << get_name() << std::endl;
}

std::string Animal::get_name() {
    return name;
}

void Animal::set_name(std::string new_name) {
    name = new_name;
}
