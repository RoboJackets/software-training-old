#include <iostream>
#include <vector>

std::vector<double> readInVector(std::string s) {
  int prev_location = 0;
  int next_location = 0;
  std::vector<double> result;
  while(s.find(',', prev_location) != std::string::npos) {
    next_location = s.find(',', prev_location);
    //std::cout << "prev_location: " << prev_location << std::endl;
    //std::cout << "next_location: " << next_location << std::endl;
    // substr [,]
    //std::cout << s.substr(prev_location, next_location - prev_location) << std::endl;
    result.push_back(std::stod(s.substr(prev_location, next_location - prev_location)));
    next_location++;
    prev_location = next_location;
  }
  result.push_back(std::stod(s.substr(prev_location, std::string::npos)));
  return result;
}

int main() {
  std::vector<double> x;
  std::vector<double> w;
  std::vector<double> y;
  bool pack_with_zeros = true;

  std::string s;
  std::cin >> s;
  if(s == "false") {
    pack_with_zeros = false;
  }
  std::cin >> s;
  x = readInVector(s);
  std::cin >> s;
  w = readInVector(s);


  std::cout << "x: {" << x[0];
  for(int i = 1; i < x.size(); i++) {
    std::cout << ", " << x[i];
  }
  std::cout << "}" << std::endl;

  std::cout << "w: {" << w[0];
  for(int i = 1; i < w.size(); i++) {
    std::cout << ", " << w[i];
  }
  std::cout << "}" << std::endl;

  int packing_size = (w.size()-1)/2;

  for(int i = 0; i < x.size(); i++) {
    double accumulator = 0;
    for(int j = 0; j < w.size(); j++) {

      // Since kernel is filled in with 0's outside does not matter
      if(i - packing_size + j >= 0 && i - packing_size + j < x.size()) {
        accumulator += x[i - packing_size + j]*w[j];
      } else if(!pack_with_zeros && i - packing_size + j < 0) {
        // Handles the case where where we underflow indexes
        accumulator += x[0] * w[j];
      } else if(!pack_with_zeros && i - packing_size + j >= x.size()) {
        // Handles the case where we overflow indexes
        accumulator += x[x.size()-1] * w[j];
      }
    }
    y.push_back(accumulator);
  }

  std::cout << "{" << y[0];
  for(int i = 1; i < y.size(); i++) {
    std::cout << ", " << y[i];
  }
  std::cout << "}" << std::endl;

  return 0;
}

