# Exercise 3.2

Welcome to exercise 3.2. This exercise will test your ability to create and use pointers. This exercise will expect you to know all the information in the Codecademy C++ course through the References and Pointers section. Our own videos playlist about smart pointers is also needed.

# Exercise Statement

In this exercise you will finish writing the code needed to create a linked list. A linked list is a data structure that holds data, similar to a vector. A linked list is made up of nodes in a line each which contains two pieces of information:
1. Whatever data you want to store in the linked list (in this case a string)
2. A pointer to the next node
However, the linked list itself does not keep track of the pointer of each piece of data, but instead only keeps track of the first, or head, node. From the head pointer it is possible to iterate through and access the entire list. You will be creating some functions that help use a linked list, this includes functions that remove data and add new data at specific locations and also one to print the whole list out. Here is a graphical representation of a linked list below.

![](https://www.geeksforgeeks.org/wp-content/uploads/gq/2013/03/Linkedlist.png)

# Starting Off

We have given you a linked_list.hpp header file which contains the class definitions of both the Node class, which stores a name as a string and the pointer to the next node, and the LinkedList class. LinkedList will have one private variable call head, which is a pointer to a Node. Its public members are all methods and are add_to_front, add_to_back, add_at_index, remove_from_front, remove_from_back, remove_from_index, and print_names. It will be your task to define these functions in linked_list.cpp. There is also a constructor which has been done for you. In the linked_list.cpp file you will see the constructors for both Node and LinkedList have been defined for you, but the rest of the functions have empty definitions. 

Something else you will have to consider is the use of the .get() method of std::unique_ptr. The .get() returns the stored pointer of the unique pointer which is a pointer which points to the same object as the unique pointer. However, this stored pointer does not have the ownership contraints of a unique_ptr, but also does not make the unique_ptr give up its ownership. This makes .get() useful for iterating through the linked list as we can change the stored pointer easily.

Header File:
```c++
#include <string>
#include <vector>
#include <memory>

class Node {
public:
    std::string name;
    std::unique_ptr<Node> next;
};

class LinkedList {
private:
    std::unique_ptr<Node> head;
public:

    LinkedList();

    void add_to_front(std::string name);

    void add_to_back(std::string name);

    void add_at_index(std::string name, int index);

    void remove_from_front();

    void remove_from_back();

    void remove_at_index(int index);

    void print_names();

};
```

Starter Code:
```c++
#include "exercise_3_2.hpp"
#include <iostream>

LinkedList::LinkedList() {
    head = nullptr;
}

void LinkedList::add_to_front(std::string name) {
    return;
}

void LinkedList::add_to_back(std::string name){
    return;
}

void LinkedList::add_at_index(std::string name, int index) {
    return;
}

void LinkedList::remove_from_front() {
    return;
}

void LinkedList::remove_from_back() {
    return;
}

void LinkedList::print_names() {
    return;
}
};
```

# Steps
The algorthms for most of the functions below can be found here: https://visualgo.net/en/list.
This should also give you a good visualization about what is happening in each of these methods.
It should also be noted that the code the visualization shows is just pseudo code and should NOT be used for this exercise, this tool is just to help you understnad the add and remove algormthms for linked lists.
### add_to_front:
1. Create a new variable that is a unique pointer to a Node using std::make_unique<>()
2. Set the name of the new node to the name parameter passed into the function by changing the string variable called name to the passed in value by using -> to access the name member of the Node
3. Set the new node’s next variable to the head pointer of the linked list by using std::move() on the head of the linked list
4. Set the head of the linked list to the new node pointer again using std::move() except on the new node pointer
### add_to_back:
1. Create a new variable that is a unique pointer to a Node
2. Set the name of new node pointer to be the name parameter and set its next pointer to nullptr
3. Create an if else statement that checks if the linked list is empty by checking if the head is a nullptr
4. Within the if statement create a variable, called curr, that is a pointer to a Node and set it to the head pointer using .get() on the head pointer
5. Within the if statement create a while loop that sets curr to its next pointer while the next pointer of curr is not nullptr by using .get() on curr's next pointer
6. Outside of the loop and inside the if statement set curr’s next pointer to be the new node pointer by using std::move()
7. In the else statement set head to be the new node pointer again by using the std::move() function
### add_at_index:
1. Create a new variable that is a unique pointer to a Node
2. Set the name of new node pointer to be the name parameter
3. Create an if else statement that checks if the index is not 0
4. In the if statement create a variable that is a pointer to a Node and define set it to the head pointer called curr using .get()
5. In the if statement create a for loop that iterates index-1 number of times that sets curr to its next pointer using .get()
6. Outside of the loop in the if statement set the new node’s next pointer to curr’s next pointer using std::move()
7. In the if statement set curr’s next pointer to the new node pointer using std::move()
8. In the else statement use the same technique in add to front to make the new node pointer the head
### remove_from_front:
10. Create an if statement that checks that the linked list is not empty
11. Within the if statement set head to head’s next pointer using the proper method
### remove_from_back:
1. Create an if statement that checks if the linked list is empty
2. Within the if statement write an if else statement that checks if a second node exists
3. Within this if statement create a unique node pointer called curr that points to head
4. Create a while loop which sets curr to its next pointer properly while the node after curr’s next pointer is not the nullptr
5. Outside of this loop set curr’s next pointer to nullptr
6. In the else statement set head to nullptr
### remove_at_index: 
1. Create an if statement to check if the linked list is empty
2. Create an if else statement to check that index is not 0
3. In the if statement define a pointer called curr and use it with a loop iterate through the linked list to the index-1 position
4. Set curr’s next pointer to curr’s next node’s next pointer
5. In the else statement set head to be head’s next pointer
### print_names:
1. Using a newly defined pointer to the head iterate through the entire linked list with a while loop
2. Within the while loop create a print statement which prints the name of the current node with a tab after it (“\t” will create a tab)
3. After all the names get printed, print a new line so the list is on its own line

# Expected Results:
After compiling and running main.cpp (remember to link the other files) the following should be printed onto the terminal:
```
Kyle    
Daniel  Kyle    
Hussain Daniel  Kyle    
Hussain Kyle    
Hussain Kyle    Oswin   
Hussain Kyle    Jason   Oswin   
Hussain Kyle    Jason   
Hussain Kyle    Jason   Woodward    
Kyle    Jason   Woodward    
```

# More on Linked Lists:
Linked lists are very basic data structures and if you ever end up taking a class about data structures, will likely be one of the first data structures you will learn about. As you may have noticed with how they are strucutured, linked lists are actually not that useful generally. You can see through your implementation that to change or access data somewhere you must iterate through the entire linked list to get there. While this might not seem so bad for smaller linked lists like the example here, for large data bases, the inefficencey is blatant. Imagine something like a data base of all of a bank's customers, it would have millions of entries, if it were strucutred as a linked list it would be very inneficient for the bank to access information on someone becauase they would have to iterate through large portions of the enitre data base. There are some optimizations you can make to a linked list however, which while not really making them that much more useable do make them more efficient. For example, here we programmed a singly linked list, but you could have made a doubly linked list where each node contains a next and previous pointer so you can move back and forward through the list, which makes the method of adding and removing nodes a bit simpler. You can also add a tail pointer which is like the head pointer except it points to the back of the list, however you would also need a size variable to make it useful, since you would need to see if the index you are accessing is closer to begining or end. There are some other optimizations as well as types of linked lists which you may want to look into if you are interested or even other types of data structures like arrays, hash maps, or trees.
