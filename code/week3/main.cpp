#include <iostream>
#include <stdlib.h>
#include <vector>
#include <array>
#include <algorithm>
#include <iterator>
#include <numeric>

int main() {
    using namespace std;

    vector<int> int_vec = {1,2,3,4,5,6,7,8,9,1,1,1};
    cout << int_vec.front() << "..." << int_vec.back() << endl;

    vector<int>::iterator it = int_vec.begin();
    for(; it != int_vec.end(); it++) {
        cout << *it << endl;
    }

    cout << count(int_vec.begin(), int_vec.end()-1, 1) << endl;

}
