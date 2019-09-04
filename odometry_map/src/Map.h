
#ifndef POTENTIAL_FIELDS_SRC_MAP_H
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "Node.h"
#define POTENTIAL_FIELDS_SRC_MAP_H_


class Map {
public:
    //Enumerates and structs
	/// Enumerate representing possible directions of the robot.
	enum class Dir{UP, DOWN, LEFT, RIGHT};
	/// Structure representing a point.
	struct point{
		int x;
		int y;
	};

	//Class functions
	/// Add a point to the map.
	/// \param point Point to be added.
	void addPoint(Map::point point);
	/// Return number of appearances of a point in the map to indicate if it has been already visited.
	/// \param point Point to be tested.
	/// \return Number of appearances of the point.
	int numAppearances(Map::point point);
	/// Return if a point is adyacent to another.
	/// \param point1 Point to test.
	/// \param point2 Point to test.
	/// \return True if adyacents.
	bool isAdyacent(Map::point point1, Map::point point2);
	/// Get all adyacent points.
	/// \param point Point with adyacences.
	/// \return Adyacence points to point.
	std::vector<point> getAdyacents(Map::point point);
	/// Get node containing a point.
	/// \param point Point in a node.
	/// \return Node containing point.
	Node<point>* getNode(Map::point point);

	// IO functions
	/// Print map into a file.
	void printMap();
	/// Read map from a file.
	void readMap();

	// Helper functions
	/// Split string with a delimiter.
	/// \param str String to be splitted.
	/// \param del Delimiter of the split.
	/// \return Vector with parts of the string.
	std::vector<std::string> split(std::string str, const std::string& del);
	/// Get string representation of the enumerate Dir.
	/// \param dir Dir to be parsed.
	/// \return String representation of dir.
	static std::string DirToString(Dir dir);

	//Constructor and Destructor
	/// Constructor for class Map.
	Map();
	/// Destructor for class map.
	virtual ~Map();

	/// Vector containing points of the map.
	std::vector<Node<point>*> map;
};

#endif /* POTENTIAL_FIELDS_SRC_MAP_H_ */
