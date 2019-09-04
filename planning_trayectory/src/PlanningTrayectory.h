#ifndef ODOMETRY_MAP_SRC_PLANNINGTRAYECTORY_H_
	#include "Map.h"
	#include <vector>
	#include <queue>
#define ODOMETRY_MAP_SRC_PLANNINGTRAYECTORY_H_

class PlanningTrayectory {


public:
    /// Map of the robot
	Map *map;

    //Enumerates and structs
    /// Enumerate with commands of the robot.
	enum class Commands {FORWARD, LEFT, RIGHT};

	//Helpers
	/// Parse command into string.
	/// \param command Command to be parsed.
	/// \return String representation of the command.
	std::string commandToString(Commands command);

	//Class functions.
	/// Get shortest path from a point to another.
	/// \param point_origin Origin point.
	/// \param point_goal Goal point.
	/// \return Vector with ordered points forming a path from origin to goal-
	std::vector<Node<Map::point>*> getShortestPath(Map::point point_origin, Map::point point_goal);
    /// Get the las node from the generated linked tree and get a path to the root.
    /// \param node Leaf of the tree containing the goal.
    /// \return Path from the leaf to the root.
    std::vector<Node<Map::point>*> getPathFromLastNode(Node<Map::point>* node);
    /// Get the command that takes the robot from one point to another.
    /// \param current Current position of the robot.
    /// \param next Next position of the robot.
    /// \return Command that takes the robot from one position to another.
    PlanningTrayectory::Commands nextPointCommand(Map::point current, Map::point next);
    /// Calculate a set of commands that takes the robot from one point to another.
    /// \param path Path of points from a point to another.
    /// \return Set of commands for the robot.
    std::vector<Commands> calculateCommands(std::vector<Node<Map::point>*> path);

    //IO functions
    /// Print commands in a file.
    /// \param commands Commands to be printed.
    void printCommands(std::vector<Commands> commands);
    /// Print a path to a file.
    /// \param path Path to be printed.
    void printPath(std::vector<Node<Map::point>*> path);

    //Constructor and destructor.
    /// Costructor for PlanningTrayectory.
	PlanningTrayectory(int argc, char **argv);
	/// Destructor for PlanningTrayectory.
	virtual ~PlanningTrayectory();
};

#endif /* ODOMETRY_MAP_SRC_PLANNINGTRAYECTORY_H_ */
