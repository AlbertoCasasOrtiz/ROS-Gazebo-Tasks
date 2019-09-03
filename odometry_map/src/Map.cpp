#include "Map.h"

#include <iostream>
#include <fstream>
#include <iterator>
#include <map>

Map::Map() {
	Map::size = 0;
}

Map::~Map() {

}

void Map::addPoint(int x, int y){
	Map::point point;
	point.x = x;
	point.y = y;

	Map::map.push_back(point);

	Map::size++;
}

void Map::printMap(){
	std::ofstream file("map.txt");
	for(int i = 0; i < Map::map.size(); i++){
		Map::point point = Map::map.at(i);
		file << point.x << " " << point.y << "\n";
	}
	file.close();
}


std::string Map::DirToString(Map::Dir dir){
	switch(dir){
		case Map::Dir::UP:
			return "UP";
		case Map::Dir::DOWN:
			return "DOWN";
		case Map::Dir::LEFT:
			return "LEFT";
		case Map::Dir::RIGHT:
			return "RIGHT";
		default:
			return "";
	}
}

int Map::numAppearances(int x, int y){
	int n = 0;
	for(int i = 0; i < Map::map.size(); i++){
		Map::point point = Map::map.at(i);
		if(point.x == x && point.y == y)
			n++;
	}
	return n;
}
