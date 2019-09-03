
#include "PlanningTrayectory.h"

PlanningTrayectory::PlanningTrayectory(int argc, char **argv) {
	PlanningTrayectory::map = new Map();
    PlanningTrayectory::map->readMap();


    Map::point origin;
	origin.x = 0;
	origin.y = 0;

	Map::point goal;
	goal.x = 5;
	goal.y = 0;

	std::vector<Node<Map::point>*> path = PlanningTrayectory::getShortestPath(origin, goal);

	for(int i = 0; i < path.size(); i++){
	    ROS_INFO("[%i], [%i]", path.at(i)->data.x, path.at(i)->data.y);
	}

	PlanningTrayectory::printPath(path);
}

PlanningTrayectory::~PlanningTrayectory() {
    delete PlanningTrayectory::map;
}

std::vector<Node<Map::point>*> PlanningTrayectory::getShortestPath(Map::point point_origin, Map::point point_goal){
	std::vector<Node<Map::point>*> path;
	std::queue<Node<Map::point>*> queue;
	if(PlanningTrayectory::map->numAppearances(point_origin) == 0){
		ROS_ERROR("Given origin point ([%i], [%i] does not exist.", point_origin.x, point_origin.y);
		return path;
	}
	if(PlanningTrayectory::map->numAppearances(point_goal) == 0){
        ROS_ERROR("Given goal point ([%i], [%i]) does not exist.", point_goal.x, point_goal.y);
		return path;
	}
	Node<Map::point>* origin = PlanningTrayectory::map->getNode(point_origin);
	origin->marked = true;
	queue.push(origin);
	while(!queue.empty()){
		Node<Map::point>* node = queue.front();
		queue.pop();
		if(node->data.x == point_goal.x && node->data.y == point_goal.y){
			path = PlanningTrayectory::getPathFromLastNode(node);
			return path;
		}
		std::vector<Map::point> adyacents = PlanningTrayectory::map->getAdyacents(node->data);

		for(int i = 0; i < adyacents.size(); i++){
			Node<Map::point>* adyacent = PlanningTrayectory::map->getNode(adyacents.at(i));
			if(!adyacent->marked){
				adyacent->marked = true;
				adyacent->parent = node;
				queue.push(adyacent);
			}
		}
	}
	return path;
}

std::vector<Node<Map::point>*> PlanningTrayectory::getPathFromLastNode(Node<Map::point>* node){
    std::vector<Node<Map::point>*> path;
    path.push_back(node);

    Node<Map::point> * parent = node->parent;

    while(parent != nullptr){
        path.push_back(parent);
        parent = parent->parent;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

void PlanningTrayectory::printPath(std::vector<Node<Map::point> *> path) {
    std::ofstream file("path.txt");
    for(int i = 0; i < path.size(); i++){
        Map::point point = path.at(i)->data;
        file << point.x << " " << point.y << "\n";
    }
    file.close();

}

std::vector<PlanningTrayectory::Commands> PlanningTrayectory::calculateCommands(Map::point point_origin, Map::point point_goal) {


    return std::vector<PlanningTrayectory::Commands>();
}
