#include "bunny.h"

Bunny::Bunny(std::string name, int required_tickets) : Prey(name) {
	this->required_tickets = required_tickets;
}

void Bunny::do_something() {
	this->ticket_num++;
	std::cout << "parking ticket issued" << std::endl;
}

int Bunny::get_ticket_num() {
	return ticket_num;
}

void Bunny::print_ticket_numbers() {
	std::cout << "I have given out " << ticket_num << " tickets" << std::endl;
	if(ticket_num > required_tickets) {
		std::cout << "I have all the ticket I need" << std::endl;
	} else {
		std::cout << "I need " << required_tickets - ticket_num << " more tickets" << std::endl;
	}
}
