#pragma once

#include <iostream>

#include "prey.h"

class Bunny : public Prey {
	int required_tickets;
	int ticket_num;
public:
	Bunny(std::string name, int required_tickets);
	virtual void do_something();
	int get_ticket_num();
	void print_ticket_numbers();
};
