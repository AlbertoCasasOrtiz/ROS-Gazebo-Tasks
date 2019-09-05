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

    //Class functions.
    /// Calculate a set of commands that takes the robot from one point to another.
    /// \param path Path of points from a point to another.
    /// \return Set of commands for the robot.
    std::vector<Commands> calculateCommands(std::vector<Node<Point>*> path);
    /// Get the command that takes the robot from one point to another.
    /// \param current Current position of the robot.
    /// \param next Next position of the robot.
    /// \return Command that takes the robot from one position to another.
    Commands nextPointCommand(Point current, Point next);

    //IO functions
    /// Print commands in a file.
    /// \param commands Commands to be printed.
    void printCommands(std::vector<Commands> commands);


    //Helpers
    /// Parse command into string.
    /// \param command Command to be parsed.
    /// \return String representation of the command.
    std::string commandToString(Commands command);
    /// Parse direction into string,
    /// \param direction Direction to be parsed,
    /// \return String representation of the direction.
    std::string dirToString(Commander::Dir direction);

    //Constructor and Destructor
    /// Constructor for class Commander.
    Commander();

    /// Where is heading the planner.
    Dir heading;

};


#endif //SRC_COMMANDER_H
