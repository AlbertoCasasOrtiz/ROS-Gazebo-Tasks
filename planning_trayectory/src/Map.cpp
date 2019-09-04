#include "Map.h"

#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <queue>

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include "Node.h"

Map::Map() {
	Map::size = 0;
	Map::heading = Dir::UP;
}

Map::~Map() = default;

void Map::addPoint(Map::point point){
	Node<Map::point> *node = new Node<Map::point>();
	node->data = point;

	Map::map.push_back(node);

	Map::size++;
}

void Map::printMap(){
	std::ofstream file("map.txt");
	for(int i = 0; i < Map::map.size(); i++){
		Map::point point = Map::map.at(i)->data;
		file << point.x << " " << point.y << "\n";
	}
	file.close();
}

std::vector<std::string> Map::split(std::string str, std::string del){
	std::vector<std::string> res;
	size_t pos = 0;
	std::string token;
	while ((pos = str.find(del)) != std::string::npos) {
	    token = str.substr(0, pos);
	    res.push_back(token);
	    str.erase(0, pos + del.length());
	    res.push_back(str);
	}
    return res;
}

void Map::readMap(){
	std::ifstream file("map.txt");
	std::string line;
	if (file.is_open()){
	    while ( std::getline (file,line) )
	    {
	    	Map::point point;
	    	// Split into two values.
	    	std::vector<std::string> words;
	    	words = Map::split(line, " ");
	    	// Store values in a point.
	    	point.x = std::stoi(words.at(0));
	    	point.y = std::stoi(words.at(1));
	    	// If duplicated, do not insert.
	    	if(Map::numAppearances(point) == 0){
	    		Map::addPoint(point);
	    	}
	    }
	    file.close();
	  }
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

int Map::numAppearances(Map::point point){
	int n = 0;
	for(int i = 0; i < Map::map.size(); i++){
		Map::point point_aux = Map::map.at(i)->data;
		if(point_aux.x == point.x && point_aux.y == point.y)
			n++;
	}
	return n;
}

bool Map::isAdyacent(Map::point point1, Map::point point2){
	if(point1.x+1 == point2.x && point1.y == point2.y)
		return true;
	if(point1.x-1 == point2.x && point1.y == point2.y)
		return true;
	if(point2.x+1 == point1.x && point1.y == point2.y)
		return true;
	if(point2.x-1 == point1.x && point1.y == point2.y)
		return true;

	if(point1.y+1 == point2.y && point1.x == point2.x)
		return true;
	if(point1.y-1 == point2.y && point1.x == point2.x)
		return true;
	if(point2.y+1 == point1.y && point1.x == point2.x)
		return true;
	if(point2.y-1 == point1.y && point1.x == point2.x)
		return true;

	return false;
}

std::vector<Map::point> Map::getAdyacents(Map::point point){
	std::vector<Map::point> adyacents;
	for(int i = 0; i < Map::map.size(); i++){
		Map::point point_aux = Map::map.at(i)->data;
		if(Map::isAdyacent(point_aux, point))
			adyacents.push_back(point_aux);
	}
	return adyacents;
}

Node<Map::point>* Map::getNode(Map::point point){
	for(int i = 0; i < Map::map.size(); i++){
		Node<Map::point>* node_aux = Map::map.at(i);
		if(node_aux->data.x == point.x && node_aux->data.y == point.y){
			return node_aux;
		}
	}
	return nullptr;
}


















