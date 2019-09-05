//
// Created by alberto on 5/09/19.
//

#include <fstream>
#include "AStar.h"
#include "ros/ros.h"

std::vector<Node<Point> *> AStar::findPath(Node<Point> *origin, Node<Point> *goal){
    std::vector<Node<Point> *> there_is_no_path;
    std::priority_queue<Node<Point>*,  std::vector<Node<Point>*>, AStar::Compare> queue;
    std::vector<Node<Point>*> closed_list;
    std::vector<Node<Point>*> open_list;

    // Calculate cost.
    origin->cost = AStar::heuristic(origin, goal) + 0;


    // Insert origin nodo to begin.
    queue.push(origin);
    open_list.push_back(origin);

    // While are still open nodes, keep searching path.
    while(!open_list.empty()){
        // Get node with lower cost in queue.
        Node<Point> *node = queue.top();

        queue.pop();

        remove(&open_list, node);

        //ROS_INFO("Origin node: [%s]",  node->toString().c_str());

        //ROS_INFO("queue [%lui]", queue.size());

        // Add node to close list.
        closed_list.push_back(node);

        std::vector<Node<Point>*> adjacents = AStar::graph.getAdjacents(node);

        for (int i = 0; i < adjacents.size(); ++i) {
            Node<Point> *adjacent = adjacents.at(i);
            if(!contains(closed_list, adjacent)) {
                if (!contains(open_list, adjacent)) {
                    adjacent->path_cost = node->cost + 1;
                    adjacent->cost = AStar::heuristic(adjacent, goal) + adjacent->path_cost;
                    adjacent->parent = node;
                    open_list.push_back(adjacent);
                    queue.push(adjacent);
                } else if (node->cost + 1 < adjacent->path_cost) {
                    adjacent->path_cost = node->cost + 1;
                    adjacent->cost = AStar::heuristic(adjacent, goal) + adjacent->path_cost;
                    adjacent->parent = node;
                }

                if (adjacent == goal) {
                    ROS_INFO("%s", adjacent->toString().c_str());
                    return AStar::getPath(adjacent);
                }
            }
        }
    }
    return there_is_no_path;
}

float AStar::heuristic(Node<Point> *origin, Node<Point> *goal) {
    return Point::manhattanDistance(origin->point, goal->point);
}

bool AStar::contains(std::vector<Node<Point>*> vector, Node<Point> *node){
    for (int i = 0; i < vector.size(); ++i) {
        if(vector.at(i) == node)
            return true;
    }
    return false;
}


void AStar::remove(std::vector<Node<Point>*>* vector, Node<Point> *node){
    std::vector<Node<Point>*>::iterator position = std::find(vector->begin(), vector->end(), node);
    if (position != vector->end()) // == myVector.end() means the element was not found
        vector->erase(position);
}

AStar::AStar() {
    AStar::graph.readNodes();
    AStar::graph.readAdjacencies();
    AStar::graph.printNodes();
    AStar::graph.printAdjacencies();
}

std::vector<Node<Point>*> AStar::getPath(Node<Point>* node){
    std::vector<Node<Point>*> path;
    path.push_back(node);
    Node<Point>* parent = node->parent;
    while(parent != nullptr){
        path.push_back(parent);
        parent = parent->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void AStar::printPath(std::vector<Node<Point>*> path) {
    std::ofstream file("path.txt");
    for(int i = 0; i < path.size(); i++){
        Point point = path.at(i)->point;
        file << point.x << " " << point.y << "\n";
    }
    file.close();
}
