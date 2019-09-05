//
// Created by alberto on 5/09/19.
//

#include "AdjacencyGraph.h"
#include <fstream>
#include <iostream>
#include "ros/ros.h"

std::vector<std::string> AdjacencyGraph::split(std::string str, const std::string &del) {
    std::vector<std::string> res;
    size_t pos = 0;
    std::string token;
    while ((pos = str.find(del)) != std::string::npos) {
        token = str.substr(0, pos);
        res.push_back(token);
        str.erase(0, pos + del.length());
    }
    res.push_back(str);
    return res;
}


void AdjacencyGraph::readNodes() {
    std::ifstream file("nodes.txt");
    std::string line;
    if (file.is_open()) {
        while (std::getline(file, line)) {
            Point point;
            // Split into two values.
            std::vector<std::string> words;
            words = AdjacencyGraph::split(line, " ");
            if (words.size() == 2) {
                // Store values in a point.
                point.x = std::stof(words.at(0));
                point.y = std::stof(words.at(1));
                // Create node,
                Node<Point> *node = new Node<Point>();
                node->point = point;
                AdjacencyGraph::graph.push_back(node);
            }
        }
        file.close();
    }
}

void AdjacencyGraph::readAdjacencies() {
    std::ifstream file("adjacents.txt");
    std::string line;
    if (file.is_open()) {
        int n = 0;
        while (std::getline(file, line)) {
            // Split into two values.
            std::vector<std::string> words;
            words = AdjacencyGraph::split(line, " ");
            std::vector<int> vec;
            AdjacencyGraph::adjacencies.push_back(vec);
            for (int i = 0; i < words.size(); i++) {
                if(words.at(i).compare("") != 0)
                    AdjacencyGraph::adjacencies.at(n).push_back(std::stoi(words.at(i)));
            }
            n++;
        }
        file.close();
    }
}

std::vector<Node<Point> *> AdjacencyGraph::getAdjacents(Node<Point> *node) {
    int pos = 0;
    // Fisrt look for node position.
    for(int i = 0; i < graph.size(); i++) {
        if (graph.at(i) == node) {
            pos = i;
        }
    }

    //Second, get adjacents positions,
    std::vector<int> adjacents = AdjacencyGraph::adjacencies.at(pos);

    //Finally, return node at adjacents positions.
    std::vector<Node<Point>*> nodes;
    for(int i = 0; i < graph.size(); i++){
        for(int j = 0; j < adjacents.size(); j++){
            if(i == j)
                nodes.push_back(graph.at(adjacents.at(j)));
        }
    }

    return nodes;
}

AdjacencyGraph::~AdjacencyGraph() {
    for (int i = 0; i < graph.size(); i++)
        delete graph.at(i);
}

void AdjacencyGraph::printNodes() {
    std::ofstream file("nodes2.txt");
    for(int i = 0; i < AdjacencyGraph::graph.size(); i++){
        Point point = AdjacencyGraph::graph.at(i)->point;
        file << point.x << " " << point.y << "\n";
    }
    file.close();
}

void AdjacencyGraph::printAdjacencies() {
    std::ofstream file("adjacencies2.txt");
    for(int i = 0; i < AdjacencyGraph::adjacencies.size(); i++){
        for(int j = 0; j < AdjacencyGraph::adjacencies.at(i).size(); j++){
            file << AdjacencyGraph::adjacencies.at(i).at(j) << " ";
        }
        file << "\n";
    }
}

