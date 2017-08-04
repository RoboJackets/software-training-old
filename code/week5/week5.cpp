
#include "week5.h"

int main() {
	using namespace std;

	/************* STATIC METHODS *************/
	cout << "1 + 2 = " << Calculator::add(1, 2) << endl;
	cout << "5 * 6 = " << Calculator::multiply(5, 6) << endl;
	cout << "5 / 6 = " << Calculator::divide(5, 6) << endl;
	cout << "5.0 / 6.0 = " << Calculator::divide(5.0, 6.0) << endl;

	cout << "\n\n";
	/************* ANIMAL *************/
	unique_ptr<Animal> animal_object = make_unique<Animal>("Bob");
	animal_object->set_name("Bob");
	animal_object->say_hello();
	animal_object->do_something();
	animal_object->say_your_name();
	cout << "\n";

	/************* PREDATOR *************/
	unique_ptr<Predator> predator_object = make_unique<Predator>("Sally");
	predator_object->set_name("Sally");
	predator_object->say_hello();
	predator_object->do_something();
	predator_object->say_your_name();
	cout << "\n";

	/************* PREY *************/
	unique_ptr<Prey> prey_object = make_unique<Prey>("Emma");
	prey_object->say_your_name();
	prey_object->set_name("Not Emma");
	prey_object->say_your_name();
	prey_object->say_hello();
	prey_object->do_something();
	cout << "\n";

	/************* BUNNY *************/
	unique_ptr<Bunny> judy_bunny_object = make_unique<Bunny>("Judy", 10);
	judy_bunny_object->say_your_name();
	judy_bunny_object->say_hello();
	judy_bunny_object->print_ticket_numbers();
	judy_bunny_object->do_something();
	judy_bunny_object->do_something();
	judy_bunny_object->print_ticket_numbers();
	cout << "there are " << Bunny::get_meter_maid_number() << " meter maids" << endl;
	unique_ptr<Bunny> another_bunny = make_unique<Bunny>("Judy", 10);
	cout << "there are " << Bunny::get_meter_maid_number() << " meter maids" << endl;
	cout << "\n";

	/************* DYNAMIC CASTING *************/
	Animal* casted_predator = dynamic_cast<Animal*>(make_unique<Predator>("John").get());
	casted_predator->set_name("John");
	casted_predator->say_hello();
	casted_predator->do_something(); 
	casted_predator->say_your_name();
	cout << "\n";

	
}
