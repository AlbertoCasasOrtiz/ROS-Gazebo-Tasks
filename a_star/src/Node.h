//
// Created by alberto on 5/09/19.
//

#ifndef SRC_NODE_H
#define SRC_NODE_H

#include <limits>
#include "Point.h"
#include <string>

template <class T>
class Node {
public:
    //Constructor and destructor
    /// Constructor of the node.
    Node() {
        cost = std::numeric_limits<float>::max();
        path_cost = std::numeric_limits<float>::max();
        parent = nullptr;
    }

    // Helpers
    /// Convert node intro string.
    /// \return string representation of a node.
    std::string toString() {
        std::string point = "Point: (" + std::to_string(Node<T>::point.x) + " " + std::to_string(Node<T>::point.y) + ")\n";
        std::string cost = "Cost: " + std::to_string(Node<T>::cost) + "\n";
        std::string path_cost = "Cost Path: " + std::to_string(Node<T>::path_cost) + "\n";
        return point + cost + path_cost;
    }

    /// Determine priority (in the priority queue)
    bool operator<(Node<T> node)
    {
        return this->cost < node.cost;
    }


    /// Point contained in the node.
    Point point;
    /// Cost of the node.
    float cost;
    /// Cost from origin to node.
    float path_cost;
    /// Node parent of the node,
    Node<T> *parent;

};

#endif //SRC_NODE_H
