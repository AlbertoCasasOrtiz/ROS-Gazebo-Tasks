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
    //Enumerates and structs.
    /// Struct for comparing nodes.
    struct Compare
    {
        bool operator()(const Node<Point>* lhs, const Node<Point> *rhs) const
        {
            return lhs->cost > rhs->cost;
        }
    };

    //Class functions
    /// Heuristic function of A*
    /// \param origin Origin point.
    /// \param goal Goal point.
    /// \return Estimated distance to goal from a node.
    float heuristic(Node<Point> *origin, Node<Point> *goal);
    /// Find a path using A*.
    /// \param origin Origin point.
    /// \param goal Goal point.
    /// \return Vector containing nodes of the path ordered from origin to goal.
    std::vector<Node<Point> *>findPath(Node<Point> *origin, Node<Point> *goal);
    /// Get path to origin from a node.
    /// \param node Given node.
    /// \return Full path in a vector.
    std::vector<Node<Point>*> getPath(Node<Point>* node);

    // IO functions
    /// Write path into a file.
    /// \param path Path to be written.
    void printPath(std::vector<Node<Point> *> path);

    // Helper functions
    /// Return if a given vector contains a node.
    /// \param vector Given vector.
    /// \param node Node.
    /// \return True if a given vector contains a node.
    static bool contains(std::vector<Node<Point>*> vector, Node<Point> *node);
    /// Remove a node from a vector.
    /// \param vector Given vector.
    /// \param node Node to be removed.
    static void remove(std::vector<Node<Point>*>* vector, Node<Point> *node);

    //Constructor and Destructor
    /// Constructor of A*.
    AStar();

    /// Graph of adjacency.
    AdjacencyGraph graph;

};


#endif //SRC_ASTAR_H
