int add(int a, int b) {
    return a + b;
}

int subtract(int a, int b) {
    return add(a, -b);
}

int multiply(int a, int b) {
    int accum = 0;
    for(int i = 0; i < a; i++) {
      accum = add(accum, b);
    }
    return accum;
}

int divide(int a, int b) {
    int result = 0;
    while(a > 0) {
      a = subtract(a, b);
      result++;
    }
    return result;
}

