#pragma once

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>


class NumberConverter {
  int number_conversions_ = 0;
  int num_binary_inputs_ = 0;
  int num_hex_inputs_ = 0;
  int num_decimal_inputs_ = 0;
  int num_binary_outputs_ = 0;
  int num_hex_outputs_ = 0;
  int num_decimal_outputs_ = 0;

  int hexCharToNumber(char input);
  char numberToHexChar(int input);
public:
  NumberConverter(std::vector<int> input);
  ~NumberConverter();

  int binaryToDecimal(std::string input, bool inc = true);
  std::string binaryToHex(std::string input, bool inc = true);
  std::string decimalToBinary(int input, bool inc = true);
  std::string decimalToHex(int input, bool inc = true);
  int hexToDecimal(std::string, bool inc = true);
  std::string hexToBinary(std::string, bool inc = true);
};
