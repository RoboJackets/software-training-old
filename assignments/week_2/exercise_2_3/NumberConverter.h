#pragma once

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>


class NumberConvereter {
  int number_conversions_ = 0;
  int num_binary_inputs_ = 0;
  int num_hex_inputs_ = 0;
  int num_decimal_inputs = 0;
  int num_binary_outputs_ = 0;
  int num_hex_outputs_ = 0;
  int num_decimal_outputs_ = 0;

  int hexCharToNumber(char input);
  char numberToHexChar(int input);
public:
  NumberConvereter() = default;
  ~NumberConvereter() {
    std::cout << "TODO do we discuss this topic?" << std::endl;
  }

  int binaryToDecimal(std::string input);
  std::string binaryToHex(std::string input);
  std::string decimalToBinary(int input);
  std::string decimalToHex(int input);
  int hexToDecimal(std::string);
  std::string hexToBinary(std::string);
};
