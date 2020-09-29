#include <string>
#include <iostream>
#include <vector>
#include <set>
#include <queue>
#include <stack>
#include <unistd.h>

struct Node
{
    std::string name;
    int r;
    int c;
    std::vector<Node *> neighbors;
};

class GridGraph
{
    std::vector<Node> adjecencyList_;
    std::vector<std::vector<std::string>> grid_;
    Node *startNode_;
    Node *exitNode_;
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
            neighbors.push_back(&adjecencyList_[getIndex(r, c)]);
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
        adjecencyList_.resize(grid_.size() * grid_[0].size());

        // Populate Node adjacency list
        for (int r = 0; r < grid_.size(); r++)
        {
            for (int c = 0; c < grid_[0].size(); c++)
            {
                if (grid_[r][c] != "#")
                {
                    std::string name = std::to_string(r) + ", " + std::to_string(c);
                    std::vector<Node *> neighbors;
                    addIfValid(neighbors, r - 1, c);
                    addIfValid(neighbors, r + 1, c);
                    addIfValid(neighbors, r, c - 1);
                    addIfValid(neighbors, r, c + 1);
                    adjecencyList_[getIndex(r, c)] = Node{name, r, c, neighbors};
                }

                if (grid_[r][c] == "S")
                    startNode_ = &adjecencyList_[getIndex(r, c)];
                else if (grid_[r][c] == "E")
                    exitNode_ = &adjecencyList_[getIndex(r, c)];
            }
        }
    }

    std::vector<Node *> BFS(bool verbose)
    {
        // visited set will be visited nodes
        // queue will be queue of paths
        std::set<Node *> visited;
        std::queue<std::vector<Node *>> queue;

        // Mark the start node as visited and enqueue it
        visited.insert(startNode_);
        queue.push(std::vector<Node *>{startNode_});

        while (!queue.empty())
        {
            // IMPLEMENT CODE HERE!

            // Dequeued front path from queue (get and delete from queue)
            // Get path's last node (current node)

            // Leave if current node is exit node
            // DEBUGGING: Iterate though nodes in solution path and set path grid cells (grid_) to "+" and print grid (displayGrid)

            // Get all adjacent nodes of the current node (use currNode->neighbors)
            // If an adjacent has not been visited (not in visited set),
            // mark it as visited and enqueue its path (copy current path. add the adjacent node, add to queue)
            // DEBUGGING: Set visited cell in grid to "*" so they are different color when displaying

            // Print grid for debugging
            if (verbose)
            {
                displayGrid();
                usleep(TIME_DELAY);
            }
        }
        return std::vector<Node *>{};
    }

    std::vector<Node *> DFS(bool verbose)
    {
        // visited set will be visited nodes
        // stack will be stack of paths
        std::set<Node *> visited;
        std::stack<std::vector<Node *>> stack;

        // Mark the start node as visited and enqueue it
        visited.insert(startNode_);
        stack.push(std::vector<Node *>{startNode_});

        while (!stack.empty())
        {
            // IMPLEMENT CODE HERE!

            // Dequeued front path from stack (get and delete from stack)
            // Get path's last node (current node)
            // Mark current node as visited
            // DEBUGGING: Set visited cell in grid to "*" so they are different color when displaying

            // Leave if current node is exit node
            // DEBUGGING: Iterate though nodes in solution path and set path grid cells (grid_) to "+" and print grid (displayGrid)

            // Get all adjacent nodes of the current node (use currNode->neighbors)
            // If an adjacent has not been visited (not in visited set),
            // enqueue its path (copy current path. add the adjacent node, add to stack)

            // Print grid for debugging
            if (verbose)
            {
                displayGrid();
                usleep(TIME_DELAY);
            }
        }
        return std::vector<Node *>{};
    }
};