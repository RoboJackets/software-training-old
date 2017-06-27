# This is a test file


## Nested

```C++
using namespace std;
// C++ swapping function
int swap(int& a, int& b) {
     int tmp = a;
     a = b;
     b = tmp;
}
int main() {
     cout << "Swapping test:" << endl;
     int a = 1;
     int b = 2;
     cout << "a: " << a << " b: " << b << endl;
     swap(a, b);
     cout << "a: " << a << " b: " << b << endl;
     swap(a, b);
     cout << "a: " << a << " b: " << b << endl;
}
```

    Swapping test:
    a: 1 b: 2
    a: 2 b: 1
    a: 1 b: 2