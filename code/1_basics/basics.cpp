// make sure to compile with g++
// to compile do
//      g++ -o basics.out basics.cpp
// to run do
//      ./basics.out

// This is including a header file for the standard library
#include <stdlib.h>
#include <string.h>
#include <iostream>


// This is a comment!
/*
This is a block comment ...
C++ has more complicated rules involving global varibales
We will cover this in depth when we talk about pointers
and memory strucutre
*/

//////////Primitive Data Types//////////
int yearsInRoboJackets = 3;
double myBirthday = 8.11;
bool isRoboJacketsCool = true;

//////////Arrays//////////
//You can do this with any primitive data type
double gpas[] = {1.0, 2.6, 4.0, 3.6};
int membersInRoboJackets[200];

int main() {
    using namespace std;
    //Ignore the above line for now we will cover this later

    //This is the main function that runs any logic!
    //////////Local Variables/////////
    int num = 0;
    int length = 0;
    int count = 0;
    string str = "";

    cout << ((3 + 3) == 9) << endl;
    cout << ((2 + 2) != 5) << endl;
    cout << true << endl;
    cout << false << endl;
    cout << (!false) << endl;
    cout << (true || false) << endl;
    cout << (true && false) << endl;

    //If-ElseIf-Else
    if (num ==  0) {
        num = 3;
        cout << "I understand Loops!" << endl;
    } else if (num > 4) {
        num += 3;
        cout << "Like John Cena, you can't see me" << endl;
    } else {
        num--;
        cout << "Like George P. Burdell, I don't ever show up" << endl;
    }

    //////////Loops/////////
    //For
    for (int n = 0; n < sizeof(gpas)/sizeof(gpas[0]); n++) {
        cout << "GPA: " << gpas[n] << endl;
    }

    //While
    while (length < sizeof(gpas)/sizeof(gpas[0])) {
        cout << "Adjusted GPA: " << (gpas[length] + 2) << endl;
        length++;
    }

    //Do-While
    do {
        cout << "Count is: " << count << endl;
        count++;
    } while (count < 11);

    // this is a status code that will be returned to whatever ran this executable
    // 0 typically means no errors
    return 0;
}
