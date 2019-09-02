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
	Map::map.insert(std::pair<int, int>(x, y));
	Map::size++;
}

void Map::printMap(){
	std::ofstream file("map.txt");
	std::map<int, int>::iterator itr;
	for(itr = Map::map.begin(); itr != Map::map.end(); ++itr){
		file << itr->first << " " << itr->second << "\n";
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
