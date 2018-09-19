#include "operations.h"

// add
int add(int a, int b) {
    std::cout << "adding (int) " << a << " + (int) " << b << std::endl;
    return a + b;
}

int add(int a, int b, int c) {
    std::cout << "adding (int) " << a << " + (int) " << b << " + (int) " << c << std::endl;
    return add(a,  add(b, c));
}

double add(double a, double b) {
    std::cout << "adding (double) " << a << " + (double) " << b << std::endl;
    return a + b;
}

// subtract
int subtract(int a, int b) {
    std::cout << "subtracting (int) " << a << " - (int) " << b << std::endl;
    return add(a, -b);
}

double subtract(double a, double b) {
    std::cout << "subtracting (double) " << a << " - (double) " << b << std::endl;
    return add(a, -b);
}

// multiply
int multiply(int a, int b) {
    std::cout << "multiplying (int) " << a << " * (int) " << b << std::endl;
    int accum = 0;
    bool neg = (a < 0 && b > 0) || (b < 0 && a > 0);
    if(a < 0) {
        a = -a;
    }
    if(b < 0) {
        b = -b;
    }
    for(int i = 0; i < a; i++) {
        accum = add(accum, b);
    }
    if (neg) {
        return -accum;
    } else {
        return accum;
    }
}

// divide
int divide(int a, int b) {
    std::cout << "dividing (int) " << a << " / (int) " << b << std::endl;
    int result = 0;
    bool neg = (a < 0 && b > 0) || (b < 0 && a > 0);
    if(a < 0) {
        a = -a;
    }
    if(b < 0) {
        b = -b;
    }
    while(a > 0) {
        a = subtract(a, b);
        result++;
    }
    if(neg) {
        return -result;
    } else {
        return result;
    }
}


