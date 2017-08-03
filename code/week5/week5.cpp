#include "week5.h"

int main() {
	using namespace std;

	/************* STATIC METHODS *************/
	cout << "1 + 2 = " << Calculator::add(1, 2) << endl;
	cout << "5 * 6 = " << Calculator::multiply(5, 6) << endl;
	cout << "5 / 6 = " << Calculator::divide(5, 6) << endl;
	cout << "5.0 / 6.0 = " << Calculator::divide(5.0, 6.0) << endl;

	cout << "\n\n";
	unique_ptr<Animal> animal_object = make_unique<Animal>();
	animal_object->say_hello();
	animal_object->do_something();

	unique_ptr<Mammal> mammal_object = make_unique<Mammal>();
	mammal_object->say_hello();
	mammal_object->do_something();

	Animal* casted_mammal = dynamic_cast<Animal*>(make_unique<Mammal>().get());
	casted_mammal->say_hello();
	casted_mammal->do_something(); 
}
