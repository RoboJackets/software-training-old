#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <unistd.h>

struct Node
{
    std::string name;
    int r;
    int c;
    int cost;
    std::vector<Node *> neighbors;
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
                std::string cell = grid_[r][c];
                if (cell != "#")
                {
                    std::string name = std::to_string(r) + ", " + std::to_string(c);
                    int cost = (cell == "S" || cell == "E") ? 0 : std::stoi(grid_[r][c]);
                    std::vector<Node *> neighbors;
                    addIfValid(neighbors, r - 1, c);
                    addIfValid(neighbors, r + 1, c);
                    addIfValid(neighbors, r, c - 1);
                    addIfValid(neighbors, r, c + 1);
                    adjecencyList_[getIndex(r, c)] = Node{name, r, c, cost, neighbors};
                }

                if (cell == "S")
                    startNode_ = &adjecencyList_[getIndex(r, c)];
                else if (cell == "E")
                    exitNode_ = &adjecencyList_[getIndex(r, c)];
            }
        }
    }

    int heuristic(Node *node, Node *goal)
    {
        // Make the heuristic just be the Manhattan distance between the node and goal
        return std::abs(node->r - goal->r) + std::abs(node->c - goal->c);
    }

    std::vector<Node *> reconstruct_path(std::map<Node *, Node *> cameFrom, Node *currNode)
    {
        // Construct path from the current node to the top most parent (startNode_)
        std::vector<Node *> path = {currNode};
        Node *parent = currNode;
        while (parent != startNode_)
        {
            parent = cameFrom[parent];
            path.insert(path.begin(), parent);
        }
        return path;
    }

    std::vector<Node *> A_star(bool verbose)
    {
        // openSet will be min-heap priority queue
        // gScore[n] will be the cheapest path cost to n currently known
        // cameFrom[n] will be n's parent for cheapest path currently known
        PriorityQueue<Node *, int> openSet;
        std::map<Node *, int> gScore;
        std::map<Node *, Node *> cameFrom;

        // Queue start node, add it to path cost map, and mark its cameFrom
        openSet.put(startNode_, 0);
        gScore[startNode_] = 0;
        cameFrom[startNode_] = startNode_;

        while (!openSet.empty())
        {
            // Dequeued front path from queue
            // DEBUGGING: Set visited cell in grid to "*" so they are different color when displaying
            Node *currNode = openSet.get();
            grid_[currNode->r][currNode->c] = "*";

            // Leave if current node is exit node
            // Find path from start to exit by iterating over parents of nodes
            // DEBUGGING: Iterate though nodes in solution path and set path grid cells (grid_) to "+" and print grid (displayGrid)
            if (currNode == exitNode_)
            {
                std::vector<Node *> path = reconstruct_path(cameFrom, currNode);
                if (verbose)
                {
                    for (Node *n : path)
                        grid_[n->r][n->c] = "+";
                    displayGrid();
                }
                return path;
            }

            // Get all adjacent nodes of the current node (use currNode->neighbors)
            // Calculate tentative_gScore := gScore[current] + cost(neighbor)
            // If tentative_gScore < gScore[neighbor], then
            // update cameFrom[neighbor] and gScore[neighbor], then
            // with fScore := tentative_gScore + heuristic(neighbor, goal)
            // update openSet
            for (Node *neighbor : currNode->neighbors)
            {
                int tentative_gScore = gScore[currNode] + neighbor->cost;
                if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = currNode;
                    gScore[neighbor] = tentative_gScore;
                    int fScore = gScore[neighbor] + heuristic(neighbor, exitNode_);
                    openSet.put(neighbor, fScore);
                }
            }

            if (verbose)
            {
                displayGrid();
                usleep(TIME_DELAY);
            }
        }
        return std::vector<Node *>{};
    }
};