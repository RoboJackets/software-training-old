#include "exercise_3_2.hpp"
#include <iostream>

LinkedList::LinkedList() {
    head = nullptr;
}

void LinkedList::add_to_front(std::string name) {
    std::unique_ptr<Node> new_node = std::make_unique<Node>();
    new_node->name = name;
    new_node->next = std::move(head);
    head = std::move(new_node);
    return;
}

void LinkedList::add_to_back(std::string name){
    std::unique_ptr<Node> new_node = std::make_unique<Node>();
    new_node->name = name;
    new_node->next = nullptr;
    if (head != nullptr) {
        Node* curr = head.get();
        while (curr->next != nullptr) {
            curr = curr->next.get();
        }
        curr->next = std::move(new_node);
    } else {
        head = std::move(new_node);
    }
    return;
}

void LinkedList::add_at_index(std::string name, int index) {
    std::unique_ptr<Node> new_node = std::make_unique<Node>();
    new_node->name = name;
    if (index != 0) {
        Node* curr = head.get();
        for (int i = 0; i < index-1; i++) {
            curr = curr->next.get();
        }
        new_node->next = std::move(curr->next);
        curr->next = std::move(new_node);
    } else {
        new_node->next = std::move(head);
        head = std::move(new_node);
    }
    return;
}

void LinkedList::remove_from_front() {
    if (head != nullptr) {
        head = std::move(head->next);
    }
    return;
}

void LinkedList::remove_from_back() {
    if (head != nullptr) {
        if (head->next != nullptr) {
            Node* curr = head.get();
            while (curr->next->next != nullptr) {
                curr = curr->next.get();
            }
            curr->next = nullptr;
        } else {
            head = nullptr;
        }
    return;
    }
}

void LinkedList::remove_at_index(int index) {
    if (head != nullptr) {
        if (index != 0) {
            Node* curr = head.get();
            for (int i = 0; i < index - 1; i++) {
                curr = curr->next.get();
            }
            curr->next = std::move(curr->next->next);
        } else {
            head = std::move(head->next);
        }
    }
}   

void LinkedList::print_names() {
    Node* curr = head.get();
    while (curr != nullptr) {
        std::cout << curr->name << "\t";
        curr = curr->next.get();
    }
    std::cout << std::endl;
}
