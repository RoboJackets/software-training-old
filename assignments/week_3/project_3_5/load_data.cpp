// C++ program to print DFS traversal from
// a given vertex in a given graph
#include <string>
#include <iostream>
#include <sstream>
#include <utility>
#include <iterator>
#include <numeric>
#include <algorithm>
#include <vector>

#include "graph.hpp"

using namespace std;

template <class T>
std::vector<T> getInputLine()
{
    string line;
    std::getline(std::cin, line);
    std::istringstream this_line(line);
    return std::vector<T>(std::istream_iterator<T>(this_line), std::istream_iterator<T>());
}

std::vector<std::vector<std::string>> loadGrid() {
    int lines = getInputLine<int>()[0];
    std::vector<std::vector<std::string>> grid;
    for (int i = 0; i < lines; i++)
    {
        grid.push_back(getInputLine<std::string>());
    }
    return grid;
}

int main(int argc, char **argv)
{
    // Ensure command-line args are correct
    if (argc != 2) {
        std::cout << "Incorrect Arguement Length (ex. ./graph_search.out True)" << std::endl;
        return 1;
    }

    std::string verbose_type = argv[1];
    if (verbose_type != "True" && verbose_type != "False")
    {
        std::cout << "Verbose Type [" << argv[1] << "] Not Reecognized (True or False)" << std::endl;
        return 1;
    }
    bool verbose = verbose_type == "True";

    std::cout << "Please Input Grid Map:" << std::endl;
    GridGraph graph(loadGrid());
    std::vector<Node *> instructions =  graph.A_star(verbose);
    if (!instructions.empty())
        for (Node *node : instructions)
            std::cout << node->name << std::endl;
    else
        std::cout << "No Path Found" << std::endl;

    return 0;
}
