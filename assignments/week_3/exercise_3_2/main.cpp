#include "exercise_3_2.hpp"
#include <string>
#include <iostream>

int main() {
    LinkedList trainers = LinkedList();
    trainers.remove_from_back();
    trainers.add_to_front("Kyle");
    trainers.print_names();
    trainers.add_at_index("Daniel", 0);
    trainers.print_names();
    trainers.add_to_front("Hussain");
    trainers.print_names();
    trainers.remove_at_index(1);
    trainers.print_names();
    trainers.add_to_back("Oswin");
    trainers.print_names();
    trainers.add_at_index("Jason", 2);
    trainers.print_names();
    trainers.remove_from_back();
    trainers.print_names();
    trainers.add_to_back("Woodward");
    trainers.print_names();
    trainers.remove_at_index(0);
    trainers.print_names();


    return 1;
}
