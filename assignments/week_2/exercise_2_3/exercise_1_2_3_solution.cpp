#include <iostream>
#include "NumberConverter.h"

int main() {
  char input_type;
  std::cout << "Type what type of number you are going to input" << std::endl;

  int test = 123;
  NumberConvereter conv;
  std::cout << "10 == " << conv.decimalToBinary(test) << std::endl;
  std::cout << "10 == " << conv.decimalToHex(test) << std::endl;

  std::cout << "b11111 == " << conv.binaryToDecimal("110101011") << std::endl;
  std::cout << "b11111 == " << conv.binaryToHex("110101011") << std::endl;

  std::cout << "0xA == " << conv.hexToDecimal("511AFE") << std::endl;
  std::cout << "0xA == " << conv.hexToBinary("511AFE") << std::endl;
}
