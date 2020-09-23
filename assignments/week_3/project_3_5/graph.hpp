#include <string>
#include <iostream>
#include <sstream>
#include <utility>
#include <iterator>
#include <numeric>
#include <algorithm>
#include <vector>
#include <tuple>
#include <map>
#include <set>
#include <queue>
#include <unistd.h>

struct Node
{
    std::string name;
    int r;
    int c;
    int cost;
    std::vector<Node*> neighbors;
};

template <typename T, typename priority_t>
struct PriorityQueue
{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
                        std::greater<PQElement>>
        elements;

    inline bool empty() const
    {
        return elements.empty();
    }

    inline void put(T item, priority_t priority)
    {
        elements.emplace(priority, item);
    }

    T get()
    {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

class GridGraph {
    std::vector<Node> adjecency_list_;
    std::vector<std::vector<std::string>> grid_;
    Node* start_node_;
    Node* end_node_;
    const int TIME_DELAY = 100000; // micro sec

private:
    int getIndex(int r, int c)
    {
        return r * grid_[0].size() + c;
    }

    void addIfValid(std::vector<Node *> &neighbors, int r, int c)
    {
        if (0 <= r && 0 <= c && r < grid_.size() && c < grid_[0].size() && grid_[r][c] != "#")
        {
            neighbors.push_back(&adjecency_list_[getIndex(r, c)]);
        }
    }

    void displayGrid()
    {
        for (const std::vector<std::string> &row : grid_)
        {
            for (std::string v : row)
            {
                std::string color;
                if (v == "#")
                    color = "\033[1;31m"; // Red
                else if (v == "*")
                    color = "\033[1;32m"; // Green
                else if (v == "+")
                    color = "\033[1;37m"; // White
                else
                    color = "\033[1;34m"; // Blue

                std::cout << color << v << "\033[0m" << ' ';
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }


public:
    GridGraph(const std::vector<std::vector<std::string>> grid)
    {
        grid_ = grid;
        adjecency_list_.resize(grid_.size() * grid_[0].size());

        // Populate Node adjacency list
        for (int r = 0; r < grid_.size(); r++)
        {
            for (int c = 0; c < grid_[0].size(); c++)
            {
                std::string cell = grid_[r][c];
                if (cell != "#") {
                    std::string name = std::to_string(r) + ", " + std::to_string(c);
                    int cost = (cell == "S" || cell == "E") ? 0 : std::stoi(grid_[r][c]);
                    std::vector<Node*> neighbors;
                    addIfValid(neighbors, r - 1, c);
                    addIfValid(neighbors, r + 1, c);
                    addIfValid(neighbors, r, c - 1);
                    addIfValid(neighbors, r, c + 1);
                    adjecency_list_[getIndex(r, c)] = Node{name, r, c, cost, neighbors};
                }

                if (cell == "S")
                    start_node_ = &adjecency_list_[getIndex(r, c)];
                else if (cell == "E")
                    end_node_ = &adjecency_list_[getIndex(r, c)];
            }
        }
    }

    int heuristic(Node *node, Node *goal){
        return 10 * (std::abs(node->r - goal->r) + std::abs(node->c - goal->c));
        }

    std::vector<Node *> A_star(bool verbose)
    {
        PriorityQueue<Node*, int> queue;
        std::map<Node *, int> pathCost;
        std::map<Node *, Node *> parent;

        // Mark the current node as visited and enqueue it
        queue.put(start_node_, 0);
        pathCost[start_node_] = 0;
        parent[start_node_] = start_node_;

        while (!queue.empty())
        {
            Node *currNode = queue.get();

            if (currNode == end_node_)
            {
                std::vector<Node *> path = {currNode};
                Node * parentNode = currNode;
                while (parentNode != start_node_){
                    parentNode = parent[parentNode];
                    path.insert(path.begin(), parentNode);
                }
                if (verbose)
                {
                    for (Node *n : path)
                        grid_[n->r][n->c] = "+";
                    displayGrid();
                }
                return path;
            }

            for (Node *neighbor : currNode->neighbors)
            {
                int newPathCost = pathCost[currNode] + neighbor->cost;
                if (pathCost.find(neighbor) == pathCost.end() || newPathCost < pathCost[neighbor])
                {
                    pathCost[neighbor] = newPathCost;
                    int priority = newPathCost + heuristic(neighbor, end_node_);
                    queue.put(neighbor, priority);
                    parent[neighbor] = currNode;
                    grid_[neighbor->r][neighbor->c] = "*";
                }
            }
            if (verbose)
            {
                displayGrid();
                usleep(TIME_DELAY); // micro sec
            }
        }
        return std::vector<Node *>{};
    }
};