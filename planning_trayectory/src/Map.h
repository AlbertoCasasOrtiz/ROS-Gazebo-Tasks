
#ifndef POTENTIAL_FIELDS_SRC_MAP_H
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include "Node.h"
#define POTENTIAL_FIELDS_SRC_MAP_H_

class Map {
public:

	// Enums and structs.
	enum class Status {UNKNOWN, CLEAR, BLOCKED, NUL};

	enum class Dir{UP, DOWN, LEFT, RIGHT};

	struct point{
		int x;
		int y;
	};

	// Class methods-
	void addPoint(Map::point point);
	int numAppearances(Map::point point);
	bool isAdyacent(Map::point point1, Map::point point2);
	std::vector<point> getAdyacents(Map::point point);
	Node<point>* getNode(Map::point point);

	// IO classes
	void printMap();
	void readMap();

	// Helper classes
	std::vector<std::string> split(std::string str, std::string del);
	static std::string DirToString(Dir dir);

	Map();
	virtual ~Map();

	std::vector<Node<point>*> map;
private:
	int size;
};

#endif /* POTENTIAL_FIELDS_SRC_MAP_H_ */
