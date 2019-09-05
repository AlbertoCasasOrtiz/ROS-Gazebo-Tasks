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
    std::vector<Node<Point>*> graph;
    std::vector<std::vector<int>> adjacencies;

    void readNodes();
    void readAdjacencies();
    std::vector<std::string> split(std::string str, const std::string& del);
    std::vector<Node<Point>*> getAdjacents(Node<Point> *node);

    ~AdjacencyGraph();

    void printNodes();
    void printAdjacencies();
};


#endif //SRC_ADJACENCYGRAPH_H
