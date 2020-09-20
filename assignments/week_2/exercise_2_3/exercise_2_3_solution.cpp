#include <iostream>
#include "NumberConverter.h"

std::vector<int> readInVector(std::string s) {
  int prev_location = 0;
  int next_location = 0;
  std::vector<int> result;
  while(s.find(',', prev_location) != std::string::npos) {
    next_location = s.find(',', prev_location);
    // substr [,]
    result.push_back(std::stoi(s.substr(prev_location, next_location - prev_location)));
    next_location++;
    prev_location = next_location;
  }
  result.push_back(std::stoi(s.substr(prev_location, std::string::npos)));
  return result;
}

int main() {
  char output_type;

  std::string num_input;

  std::string input_vec_str;
  std::cin >> input_vec_str;
  std::vector<int> input = readInVector(input_vec_str);
  std::cin >> output_type;
  std::cin >> num_input;

  NumberConvereter conv(input);

  bool hex = num_input.size() > 2 && num_input[1] == 'x';
  bool binary = num_input.size() > 1 && num_input[0] == 'b';
  bool decimal = !hex && !binary;

  if(hex) {
    num_input = num_input.substr(2, std::string::npos);
  }

  if(binary) {
    num_input = num_input.substr(1, std::string::npos);
  }

  std::cout << "input: " << num_input << std::endl;
  std::cout << "output type: " << output_type << std::endl;

  std::cout << "result: ";
  if(output_type == 'b' && hex) {
    std::cout << conv.hexToBinary(num_input) << std::endl;
  } else if(output_type == 'b' && decimal) {
    std::cout << conv.decimalToBinary(std::stoi(num_input)) << std::endl;
  } else if(output_type == 'h' && binary) {
    std::cout << conv.binaryToHex(num_input) << std::endl;
  } else if(output_type == 'h' && decimal) {
    std::cout << conv.decimalToHex(std::stoi(num_input)) << std::endl;
  } else if(output_type == 'd' && binary) {
    std::cout << conv.binaryToDecimal(num_input) << std::endl;
  } else if(output_type == 'd' && hex) {
    std::cout << conv.hexToDecimal(num_input) << std::endl;
  }

}
