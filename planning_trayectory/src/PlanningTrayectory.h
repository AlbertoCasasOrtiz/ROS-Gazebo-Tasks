#ifndef ODOMETRY_MAP_SRC_PLANNINGTRAYECTORY_H_
	#include "Map.h"
	#include <vector>
	#include <queue>
#define ODOMETRY_MAP_SRC_PLANNINGTRAYECTORY_H_

class PlanningTrayectory {


public:
	Map *map;

	enum class Commands {FORWARD, LEFT, RIGHT};

	std::vector<Commands> calculateCommands(Map::point point_origin, Map::point point_goal);
	std::vector<Node<Map::point>*> getShortestPath(Map::point point_origin, Map::point point_goal);

    std::vector<Node<Map::point>*> getPathFromLastNode(Node<Map::point>* node);

	PlanningTrayectory(int argc, char **argv);
	virtual ~PlanningTrayectory();
};

#endif /* ODOMETRY_MAP_SRC_PLANNINGTRAYECTORY_H_ */
