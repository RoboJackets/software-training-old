##########Global Variables##########

###########Primitive Data Types##########
#python isn't a typed language like C++ or Java
#This is a comment!

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

def exampleFunction():
	#########Local Variables##########
	number = 0
	length = 0
	count = 0
	string = ""

	##########Operators#########
	print (3 + 3 == 9)
	print (2 + 2 != 5)
	print not True
	print True or False
	print True and False

	#########Loops#########
	##If-ElseIf-Else
	if (number == 0):
		number = 3
		print "I understand Loops!"
	elif (number > 4):
		number += 3
		print "Like John Cena, you can't see me"
	else:
		number -= 1
		print "Like George P. Burdell, I don't ever show up"

	##For
	for n in range(len(gpas)):
		print "GPA:", gpas[n]

	##While
	length = 0
	while (length < len(gpas)):
		print "Adjusted GPA:", (gpas[length] + 2)
		length += 1

exampleFunction()