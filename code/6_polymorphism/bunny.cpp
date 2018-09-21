#include "bunny.h"

// init static varibale in cpp class so it can be found
int Bunny::meter_maid_number = 0;

// calls the Prey constructor with : Prey(name)
Bunny::Bunny(std::string name, int required_tickets_num) : Prey(name) {
    required_tickets = required_tickets_num;
    meter_maid_number++;
    std::cout << "constructor of Bunny called" << std::endl;
}

void Bunny::do_something() {
    ticket_num++;
    std::cout << "parking ticket issued" << std::endl;
}

int Bunny::get_ticket_num() {
    return ticket_num;
}

void Bunny::print_ticket_numbers() {
    std::cout << "I have given out " << get_ticket_num() << " tickets" << std::endl;
    if(get_ticket_num() > required_tickets) {
        std::cout << "I have all the ticket I need" << std::endl;
    } else {
        std::cout << "I need " << required_tickets - get_ticket_num() << " more tickets" << std::endl;
    }
}

int Bunny::get_meter_maid_number() {
    return meter_maid_number;
}
