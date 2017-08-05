#pragma once

#include <iostream>

#include "prey.h"

// subclass of Prey through that a subclass of Animal
class Bunny : public Prey {
private:
    static int meter_maid_number;
    int required_tickets;
    int ticket_num;
public:
    // constructor
    Bunny(std::string name, int required_tickets);

    // destructor
    ~Bunny() {std::cout << "object with name " << get_name() << " Bunnny, ";}

    // overrides the do_something method from animal and will be called
    // even if a Bunny is referenced as a Animal
    // override annotation has the compiler check that the method in overriden
    // correctly i.e. this prototype matches one in Animal of the same name.
    void do_something() override;

    //class methods
    int get_ticket_num();
    void print_ticket_numbers();

    // static class method
    static int get_meter_maid_number();
};
