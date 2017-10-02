#pragma once
// This line has the complier check that this header file is not included more than once in a single file

// iostream contains the functions cout, cin, cerr, and clog all fucntions taht deal with IO (input and output)
#include <iostream>

/**
 * adds two integers together
 */
int add(int a, int b);

/**
 * adds three integers together
 */
int add(int a, int b, int c);

/**
 * adds two doubles together
 */
double add(double a, double b);

/**
 * subtracts (int) b from (int) a
 */
int subtract(int a, int b);

/**
 * subtracts (double) b from (double) a
 */
double subtract(double a, double b);

/**
 * multiplies two integers together
 */
int multiply(int a, int b);

/**
 * divies (int) a by (int) b
 * divide by zero not suuported
 */
int divide(int a, int b);
