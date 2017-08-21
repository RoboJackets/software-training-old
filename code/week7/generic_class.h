#include <iostream>

template <class primitive_type> class Example {
private:
    primitive_type var;
public:
    Example(primitive_type var) : var(var) {
        std::cout << "creating object with var = " << var << std::endl;
    };
    ~Example() {};

    primitive_type getVar() const {
        return var;
    }

};
