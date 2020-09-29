#include <string>
#include <iostream>
#include <sstream>
#include <utility>
#include <iterator>
#include <numeric>
#include <algorithm>
#include <vector>

#include "graph_search.hpp"

template <class T>
std::vector<T> getInputLine()
{
    std::string line;
    std::getline(std::cin, line);
    std::istringstream this_line(line);
    return std::vector<T>(std::istream_iterator<T>(this_line), std::istream_iterator<T>());
}

std::vector<std::vector<std::string>> loadGrid()
{
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
    // Ensure command-line args are correct (ex. ./dungeon.out DFA True)
    if (argc != 3)
    {
        std::cout << "Incorrect Arguement Length (ex. ./dungeon DFA True)" << std::endl;
        return 1;
    }

    std::string search_type = argv[1];
    std::string verbose_type = argv[2];
    if (!(search_type == "DFS" || search_type == "BFS"))
    {
        std::cout << "Search Type [" << argv[1] << "] Not Recognized (DFS or BFS)" << std::endl;
        return 1;
    }
    else if (verbose_type != "True" && verbose_type != "False")
    {
        std::cout << "Verbose Type [" << argv[2] << "] Not Recognized (True or False)" << std::endl;
        return 1;
    }
    bool verbose = verbose_type == "True";

    // Load Dungeon Map
    std::cout << "Please Input Dungeon Map: <Copy/Paste Text from test_case/*.txt>" << std::endl;
    GridGraph graph(loadGrid());

    // Run Graph Search
    std::vector<Node *> instructions = (search_type == "BFS") ? graph.BFS(verbose) : graph.DFS(verbose);
    if (!instructions.empty())
        for (Node *node : instructions)
            std::cout << node->name << std::endl;
    else
        std::cout << "No Path Found" << std::endl;

    return 0;
}
