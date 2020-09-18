#include "NumberConverter.h"

/**
 * returns a number 0-15 based off of a hex char
 * This uses ascii to figure out number it corresponds to
 * @param input
 * @return
 */
int NumberConvereter::hexCharToNumber(char input) {
  // this means it is [A,B,C,D,E,F]
  if(input > 57) {
    return input - (65 - 10);
  } else {
    // this is a digit in ascii
    return input - 48;
  }
}

char NumberConvereter::numberToHexChar(int input) {
  if(input < 10) {
    // it is a digit
    return input + 48;
  } else {
    return (input - 10) + 65;
  }
}

int NumberConvereter::binaryToDecimal(std::string input) {
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

std::string NumberConvereter::binaryToHex(std::string input) {
  std::string result;
  int accum = 0;
  for(int i = 0; i < input.size(); i++) {
    if(i % 4 == 3) {
      if(input[i] == '1') {
        accum += pow(2, i%4);
      }
      result.append(decimalToHex(accum));
      accum = 0;
    } else {
      if(input[i] == '1') {
        accum += pow(2, i%4);
      }
    }
  }
  result.append(decimalToHex(accum));
  std::reverse(result.begin(), result.end());
  return result;
}

std::string NumberConvereter::decimalToBinary(int input) {
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

std::string NumberConvereter::decimalToHex(int input) {
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

int NumberConvereter::hexToDecimal(std::string input) {
  int result = 0;
  // iterate through the string
  std::reverse(input.begin(), input.end());
  for(int i = 0; i < input.size(); i++) {
    result += hexCharToNumber(input[i])*std::pow(16, i);
  }
  return result;
}

std::string NumberConvereter::hexToBinary(std::string input) {
  std::string result;
  std::reverse(input.begin(), input.end());
  for(int i = 0; i < input.size(); i++) {
    int number = hexCharToNumber(input[i]);
    std::cout << number << std::endl;
    std::string bin = decimalToBinary(number);
    while(bin.size() < 4) {
      bin.insert(0, "0");
    }

    result.insert(0, bin);
  }
  return result;
}
