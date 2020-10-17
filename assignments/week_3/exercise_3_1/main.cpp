//
// Created by jason on 9/20/20.
//

#include <iostream>

int main() {
  int variable = 10;
  int* variable_ptr = &variable;
  std::cout << "var: " << variable << std::endl;
  std::cout << "var_ptr: " << variable_ptr << std::endl; // this points out the memory address that it is pointing at
  return 0;
}
