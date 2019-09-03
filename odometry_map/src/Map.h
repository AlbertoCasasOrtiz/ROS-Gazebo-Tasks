
#ifndef POTENTIAL_FIELDS_SRC_MAP_H
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#define POTENTIAL_FIELDS_SRC_MAP_H_

class Map {
public:

	enum class Status {UNKNOWN, CLEAR, BLOCKED, NUL};
	enum class Commands {FORWARD, LEFT, RIGHT};

	enum class Dir{UP, DOWN, LEFT, RIGHT};

	struct point{
		int x;
		int y;
	};

	static std::string DirToString(Dir dir);
	int numAppearances(int x, int y);
	void addPoint(int x, int y);
	Commands* calculatePath(int origen_x, int origen_y, int destino_x, int destino_y);

	void printMap();

	Map();
	virtual ~Map();

private:
	std::vector<point> map;
	int size;
};

#endif /* POTENTIAL_FIELDS_SRC_MAP_H_ */
