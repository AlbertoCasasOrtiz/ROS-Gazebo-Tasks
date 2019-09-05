//
// Created by alberto on 5/09/19.
//

#ifndef SRC_COMMANDER_H
#define SRC_COMMANDER_H

#include <vector>
#include "Node.h"
#include "Point.h"
class Commander {
public:
    //Enumerates and structs
    /// Enumerate with commands of the robot.
    enum class Commands {FORWARD, LEFT, RIGHT};
    //Enumerates and structs
    /// Enumerate representing possible directions of the robot.
    enum class Dir{UP, DOWN, LEFT, RIGHT};

    std::string commandToString(Commands command);

    std::vector<Commands> calculateCommands(std::vector<Node<Point>*> path);

    Commands nextPointCommand(Point current, Point next);

    void printCommands(std::vector<Commands> commands);

    Dir heading;

    Commander();
};


#endif //SRC_COMMANDER_H
