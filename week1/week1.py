#!/usr/bin/env python

# To run this file do
#		python3 week1.py

###########Primitive Data Types##########
# python is a typed language like C++ and Java
# BUT... it is dynamically typed instead of statically typed like the other two
# this means that at compile time the C++ and Java compliler knows what type variable is
# but in python they do not
# This is a comment!

'''
This is a block comment!
You can write more than
one line here.
'''
myName = "George Burdell" #string
middleInitial = 'P' #char
yearsInRoboJackets = 3 #int
myBirthday = 8.11 #float
isRoboJacketsCool = True #boolean

###########Lists##########
gpas = [1.0, 2.6, 4.0, 3.6]
membersInRoboJackets = []

def main():
    #########Local Variables##########
    number = 0
    length = 0
    count = 0
    string = ""

    ##########Operators#########
    print (3 + 3 == 9)
    print (2 + 2 != 5)
    print (not True)
    print (True or False)
    print (True and False)

    #########Loops#########
    ##If-ElseIf-Else
    if (number == 0):
        number = 3
        print ("I understand Loops!")
    elif (number > 4):
        number += 3
        print ("Like John Cena, you can't see me")
    else:
        number -= 1
        print ("Like George P. Burdell, I don't ever show up")

    ##For
    for n in gpas:
        print ("GPA:", n)

    ##While
    length = 0
    while (length < len(gpas)):
        print ("Adjusted GPA:", (gpas[length] + 2))
        length += 1

main()
