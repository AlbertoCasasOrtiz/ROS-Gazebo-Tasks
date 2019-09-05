//
// Created by alberto on 5/09/19.
//

#ifndef SRC_ASTAR_H
#define SRC_ASTAR_H

#include "Node.h"
#include "Point.h"
#include "vector"
#include "AdjacencyGraph.h"
#include <queue>

class AStar {
public:
    struct Compare
    {
        bool operator()(const Node<Point>* lhs, const Node<Point> *rhs) const
        {
            return lhs->cost > rhs->cost;
        }
    };

    AdjacencyGraph graph;

    float heuristic(Node<Point> *origin, Node<Point> *goal);

    std::vector<Node<Point> *>findPath(Node<Point> *origin, Node<Point> *goal);

    static bool contains(std::vector<Node<Point>*> vector, Node<Point> *node);
    static void remove(std::vector<Node<Point>*>* vector, Node<Point> *node);

    std::vector<Node<Point>*> getPath(Node<Point>* node);

    void printPath(std::vector<Node<Point> *> path);

    AStar();
};


#endif //SRC_ASTAR_H
