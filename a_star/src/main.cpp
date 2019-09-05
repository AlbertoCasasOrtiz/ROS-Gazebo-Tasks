#include "AStar.h"
#include "Commander.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    AStar aStar;
    std::vector<Node<Point> *> path = aStar.findPath(aStar.graph.graph.at(0), aStar.graph.graph.at(126));
    aStar.printPath(path);
    Commander commander;
    std::vector<Commander::Commands > commands = commander.calculateCommands(path);
    commander.printCommands(commands);
    return 0;
}
