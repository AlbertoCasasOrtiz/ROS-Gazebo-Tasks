
#ifndef POTENTIAL_FIELDS_SRC_MAP_H
#include <ros/ros.h>
#include <iostream>
#define POTENTIAL_FIELDS_SRC_MAP_H_

class Map {
public:
	enum class Status {UNKNOWN, CLEAR, BLOCKED, NUL};
	enum class Dir {UP, DOWN, LEFT, RIGHT};

	struct Direction{
		Status up = Status::UNKNOWN;
		Status down = Status::UNKNOWN;
		Status left = Status::UNKNOWN;
		Status right = Status::UNKNOWN;
	};

	struct Point {
		int x;
		int y;
		Direction direction;
	};

	Map();
	virtual ~Map();

	void addPoint(int x, int y, Dir direction, Status status);
	void updatePointStatus(Point *point, Dir direction, Status status);
	Status getPointStatusInDir(Point *point, Dir direction);
	Point* existPoint(int x, int y);
	void saveMap(std::string name);
	void loadMap(std::string name);

private:
	std::vector<Point*> *map;

};

#endif /* POTENTIAL_FIELDS_SRC_MAP_H_ */
