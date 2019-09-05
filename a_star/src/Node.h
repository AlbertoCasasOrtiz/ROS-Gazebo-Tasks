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
    Point point;
    float cost;
    float path_cost;
    Node<T> *parent;

    // Determine priority (in the priority queue)
    bool operator<(Node<T> node)
    {
        return this->cost < node.cost;
    }

    Node() {
        cost = std::numeric_limits<float>::max();
        path_cost = std::numeric_limits<float>::max();
        parent = nullptr;
    }

    std::string toString() {
        std::string point = "Point: (" + std::to_string(Node<T>::point.x) + " " + std::to_string(Node<T>::point.y) + ")\n";
        std::string cost = "Cost: " + std::to_string(Node<T>::cost) + "\n";
        std::string path_cost = "Cost Path: " + std::to_string(Node<T>::path_cost) + "\n";
        return point + cost + path_cost;
    }

};

#endif //SRC_NODE_H
