#include <string>
#include <vector>

class Node {
public:
	std::string name;
	Node *next;
};

class LinkedList {
private:
	Node *head;
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