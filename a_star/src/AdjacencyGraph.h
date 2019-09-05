//
// Created by alberto on 5/09/19.
//

#ifndef SRC_ADJACENCYGRAPH_H
#define SRC_ADJACENCYGRAPH_H

#include <vector>
#include "Point.h"
#include "Node.h"
#include <string>

class AdjacencyGraph {
public:
    // Class methods-
    /// Get adjacent nodes to a given node.
    /// \param node Given node.
    /// \return Adjacents to given node.
    std::vector<Node<Point>*> getAdjacents(Node<Point> *node);

    // IO functions
    /// Write nodes into a file.
    void printNodes();
    /// Write adjacencies into a file.
    void printAdjacencies();
    /// Read nodes from a file.
    void readNodes();
    /// Read adjacents from a file.
    void readAdjacencies();

    // Helper functions
    /// Split string with a delimiter.
    /// \param str String to be splitted.
    /// \param del Delimiter of the split.
    /// \return Vector with parts of the string.
    std::vector<std::string> split(std::string str, const std::string& del);

    //Constructor and Destructor
    /// Destructor of adjacency graph.
    ~AdjacencyGraph();

    /// Graph containing nodes.
    std::vector<Node<Point>*> graph;
    /// Matrix containing adjacencies.
    std::vector<std::vector<int>> adjacencies;

};


#endif //SRC_ADJACENCYGRAPH_H
