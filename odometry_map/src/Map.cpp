#include "Map.h"

Map::Map() {
	map = new std::vector<Map::Point*>();
}

Map::~Map() {
	// Delete all points.
	for(int i = 0; i < Map::map->size(); i++){
		delete Map::map->at(i);
	}
	// Delete map.
	delete map;
}

void Map::addPoint(int x, int y, Map::Dir direction, Map::Status status){
	//If the point exists, update and return.
	Map::Point *mapPoint = Map::existPoint(x, y);
	if(mapPoint != nullptr){
		this->updatePointStatus(mapPoint, direction, status);
		return;
	}

	//If the point does not exist, insert a new one.
	Map::Point *point = new Map::Point();

	// Set x, y coordinates.
	point->x = x;
	point->y = y;

	// Set direction status. Remember that other directions remain unknown as default.
	Map::updatePointStatus(point, direction, status);

	// Insert point into map.
	Map::map->push_back(point);
}

void Map::updatePointStatus(Map::Point *point, Map::Dir direction, Map::Status status){
	// Set direction status. Remember that other directions remain unknown as default.
	switch(direction){
		case Map::Dir::UP:
			point->direction.up = status;
			break;
		case Map::Dir::DOWN:
			point->direction.down = status;
			break;
		case Map::Dir::LEFT:
			point->direction.left = status;
			break;
		case Map::Dir::RIGHT:
			point->direction.right = status;
			break;
	}
}

Map::Status Map::getPointStatusInDir(Map::Point *point, Map::Dir direction){
	// Get direction status.
	Map::Point *mapPoint = Map::existPoint(point->x, point->y);
	if(mapPoint != nullptr){
		switch(direction){
			case Map::Dir::UP:
				return point->direction.up;
			case Map::Dir::DOWN:
				return point->direction.down;
			case Map::Dir::LEFT:
				return point->direction.left;
			case Map::Dir::RIGHT:
				return point->direction.right;
			default:
				return Map::Status::NUL;
		}
	} else return Map::Status::NUL;
}

Map::Point* Map::existPoint(int x, int y){
	for(int i = 0; i < Map::map->size(); i++){
		Map::Point *mapPoint = Map::map->at(i);
		if(mapPoint->x == x and mapPoint->y == y){
			return mapPoint;
		}
	}
	return nullptr;
}






