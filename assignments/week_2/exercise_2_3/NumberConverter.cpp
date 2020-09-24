#include "NumberConverter.h"


NumberConverter::NumberConverter(std::vector<int> previous_counts) {
  num_decimal_inputs_ = previous_counts[0];
  num_binary_inputs_ = previous_counts[1];
  num_hex_inputs_ = previous_counts[2];
  num_decimal_outputs_ = previous_counts[3];
  num_binary_outputs_ = previous_counts[4];
  num_hex_outputs_ = previous_counts[5];
}

NumberConverter::~NumberConverter() {

  std::cout << "decimal_inputs: " << num_decimal_inputs_ << std::endl;
  std::cout << "binary_inputs: " << num_binary_inputs_ << std::endl;
  std::cout << "hex_inputs: " << num_hex_inputs_ << std::endl;
  std::cout << "decimal_outputs: " << num_decimal_outputs_ << std::endl;
  std::cout << "binary_outputs: " << num_binary_outputs_ << std::endl;
  std::cout << "hex_outputs: " << num_hex_outputs_ << std::endl;
}


/**
 * returns a number 0-15 based off of a hex char
 * This uses ascii to figure out number it corresponds to
 * @param input
 * @return
 */
int NumberConverter::hexCharToNumber(char input) {
  // this means it is [A,B,C,D,E,F]
  if(input > 57) {
    return input - (65 - 10);
  } else {
    // this is a digit in ascii
    return input - 48;
  }
}

/**
 * returns the character that is the given hex number
 * only valid on 0-15
 * @param input
 * @return
 */
char NumberConverter::numberToHexChar(int input) {
  if(input < 10) {
    // it is a digit
    return input + 48;
  } else {
    return (input - 10) + 65;
  }
}

int NumberConverter::binaryToDecimal(std::string input, bool inc) {
  if(inc) {
    num_binary_inputs_++;
    num_decimal_outputs_++;
  }
  int result = 0;
  // iterate through the string
  for(int i = 0; i < input.size(); i++) {
    // if there is a one we want to increase our result
    if(input[i] == '1') {
      result += std::pow(2, i);
    }
  }
  return result;
}

int NumberConverter::hexToDecimal(std::string input, bool inc) {
  if(inc) {
    num_hex_inputs_++;
    num_decimal_outputs_++;
  }
  int result = 0;
  // iterate through the string
  std::reverse(input.begin(), input.end());
  for(int i = 0; i < input.size(); i++) {
    result += hexCharToNumber(input[i])*std::pow(16, i);
  }
  return result;
}

std::string NumberConverter::decimalToHex(int input, bool inc) {
  if(inc) {
    num_decimal_inputs_++;
    num_hex_outputs_++;
  }
  int temp = input;
  std::string result = "";
  while(temp != 0) {
    int mod = temp % 16;
    temp /= 16;
    result.push_back(numberToHexChar(mod));
  }
  std::reverse(result.begin(), result.end());
  return result;
}

std::string NumberConverter::binaryToHex(std::string input, bool inc) {
  if(inc) {
    num_binary_inputs_++;
    num_hex_outputs_++;
  }
  std::string result;
  int accum = 0;
  for(int i = 0; i < input.size(); i++) {
    if(i % 4 == 3) {
      if(input[i] == '1') {
        accum += pow(2, i%4);
      }
      result.append(decimalToHex(accum, false));
      accum = 0;
    } else {
      if(input[i] == '1') {
        accum += pow(2, i%4);
      }
    }
  }
  result.append(decimalToHex(accum, false));
  std::reverse(result.begin(), result.end());
  return result;
}

std::string NumberConverter::decimalToBinary(int input, bool inc) {
  if(inc) {
    num_decimal_inputs_++;
    num_binary_outputs_++;
  }
  int temp = input;
  std::string result;
  while(temp != 0) {
    int mod = temp % 2;
    result.append(std::to_string(mod));
    temp /= 2;
  }
  std::reverse(result.begin(), result.end());
  return result;
}

std::string NumberConverter::hexToBinary(std::string input, bool inc) {
  if(inc) {
    num_hex_inputs_++;
    num_binary_outputs_++;
  }
  std::string result;
  std::reverse(input.begin(), input.end());
  for(int i = 0; i < input.size(); i++) {
    int number = hexCharToNumber(input[i]);
    std::cout << number << std::endl;
    std::string bin = decimalToBinary(number, false);
    while(bin.size() < 4) {
      bin.insert(0, "0");
    }

    result.insert(0, bin);
  }
  return result;
}
