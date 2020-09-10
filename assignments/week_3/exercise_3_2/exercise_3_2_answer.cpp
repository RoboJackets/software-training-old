#include "exercise_3_2.hpp"
#include <iostream>

LinkedList::LinkedList() {
    head = NULL;
}

void LinkedList::add_to_front(std::string name) {
    Node *new_node = new Node();
    new_node->name = name;
    new_node->next = head;
    head = new_node;
    return;
}

void LinkedList::add_to_back(std::string name){
    Node *new_node = new Node();
    new_node->name = name;
    new_node->next = NULL;
    if (head != NULL) {
        Node *curr = head;
        while (curr->next != NULL) {
            curr = curr->next;
        }
        curr->next = new_node;
    } else {
        head = new_node;
    }
    return;
}

void LinkedList::add_at_index(std::string name, int index) {
    Node *new_node = new Node();
    new_node->name = name;
    if (index != 0) {
        Node *curr = head;
        for (int i = 0; i < index-1; i++) {
            curr = curr->next;
        }
        new_node->next = curr->next;
        curr->next = new_node;
    } else {
        new_node->next = head;
        head = new_node;
    }
    return;
}

void LinkedList::remove_from_front() {
    if (head != NULL) {
        head=head->next;
    }
    return;
}

void LinkedList::remove_from_back() {
    if (head != NULL) {
        if (head->next != NULL) {
            Node *curr = head;
            while (curr->next->next != NULL) {
                curr = curr->next;
            }
            curr->next = NULL;
        } else {
            head = NULL;
        }
    return;
    }
}

void LinkedList::remove_at_index(int index) {
    if (head != NULL) {
        if (index != 0) {
            Node *curr = head;
            for (int i = 0; i < index - 1; i++) {
                curr = curr->next;
            }
            curr->next = curr->next->next;
        } else {
            head = head->next;
        }
    }
}   

void LinkedList::print_names() {
    Node *curr = new Node();
    curr = head;
    while (curr != NULL) {
        std::cout << curr->name << "\t";
        curr = curr->next;
    }
    std::cout << std::endl;
}
