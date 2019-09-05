
#include "PlanningTrayectory.h"

PlanningTrayectory::PlanningTrayectory(int argc, char **argv) {
	PlanningTrayectory::map = new Map();
    PlanningTrayectory::map->readMap();

    // Set origin point here.
    Map::point origin;
	origin.x = 0;
	origin.y = 0;

	// Set goal point here.
	Map::point goal;
	goal.x = 5;
	goal.y = -5;

	// Get path and commands.
	std::vector<Node<Map::point>*> path = PlanningTrayectory::getShortestPath(origin, goal);
    std::vector<PlanningTrayectory::Commands> commands = PlanningTrayectory::calculateCommands(path);

    // Print path and commands in console.
    for(int i = 0; i < path.size(); i++){
        ROS_INFO("[%i], [%i]", path.at(i)->data.x, path.at(i)->data.y);
    }
    for(int i = 0; i < commands.size(); i++){
        ROS_INFO("[%s]", PlanningTrayectory::commandToString(commands.at(i)).c_str());
    }

    // Print path and commands in a file-
	PlanningTrayectory::printPath(path);
    PlanningTrayectory::printCommands(commands);
}

PlanningTrayectory::~PlanningTrayectory() {
    delete PlanningTrayectory::map;
}

std::vector<Node<Map::point>*> PlanningTrayectory::getShortestPath(Map::point point_origin, Map::point point_goal){
	std::vector<Node<Map::point>*> path;
	std::queue<Node<Map::point>*> queue;

	// If origin not in map, abort.
	if(PlanningTrayectory::map->numAppearances(point_origin) == 0){
		ROS_ERROR("Given origin point ([%i], [%i] does not exist.", point_origin.x, point_origin.y);
		return path;
	}
	// If goal not in map, abort.
	if(PlanningTrayectory::map->numAppearances(point_goal) == 0){
        ROS_ERROR("Given goal point ([%i], [%i]) does not exist.", point_goal.x, point_goal.y);
		return path;
	}

	//Breadth first search algorithm to detect the sortest path.
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

void PlanningTrayectory::printCommands(std::vector<Commands> commands) {
    std::ofstream file("commands.txt");
    for(int i = 0; i < commands.size(); i++){
        PlanningTrayectory::Commands command = commands.at(i);
        file << PlanningTrayectory::commandToString(command) << "\n";
    }
    file.close();

}

void PlanningTrayectory::printPath(std::vector<Node<Map::point> *> path) {
    std::ofstream file("path.txt");
    for(int i = 0; i < path.size(); i++){
        Map::point point = path.at(i)->data;
        file << point.x << " " << point.y << "\n";
    }
    file.close();

}


std::string PlanningTrayectory::commandToString(PlanningTrayectory::Commands command){
    switch (command){
        case Commands::FORWARD:
            return "FORWARD";
        case Commands::RIGHT:
            return "RIGHT";
        case Commands::LEFT:
            return "LEFT";
    }
}

std::vector<PlanningTrayectory::Commands> PlanningTrayectory::calculateCommands(std::vector<Node<Map::point>*> path) {
    std::vector<PlanningTrayectory::Commands> commands;
    for(int i = 1; i < path.size(); i++){
        PlanningTrayectory::Commands command = PlanningTrayectory::nextPointCommand(path.at(i-1)->data, path.at(i)->data);
        commands.push_back(command);
        if(command == PlanningTrayectory::Commands::LEFT || command == PlanningTrayectory::Commands::RIGHT)
            commands.push_back(Commands::FORWARD);
    }

    return commands;
}


PlanningTrayectory::Commands PlanningTrayectory::nextPointCommand(Map::point current, Map::point next){
    if(next.x == current.x+1 && PlanningTrayectory::map->heading == Map::Dir::UP){
        PlanningTrayectory::map->heading = Map::Dir::UP;
        return PlanningTrayectory::Commands::FORWARD;
    }
    if(next.y == current.y-1 && PlanningTrayectory::map->heading == Map::Dir::UP){
        PlanningTrayectory::map->heading = Map::Dir::LEFT;
        return PlanningTrayectory::Commands::LEFT;
    }
    if(next.y == current.y+1 && PlanningTrayectory::map->heading == Map::Dir::UP){
        PlanningTrayectory::map->heading = Map::Dir::RIGHT;
        return PlanningTrayectory::Commands::RIGHT;
    }

    if(next.x == current.x+1 && PlanningTrayectory::map->heading == Map::Dir::LEFT){
        PlanningTrayectory::map->heading = Map::Dir::UP;
        return PlanningTrayectory::Commands ::RIGHT;
    }
    if(next.x == current.x-1 && PlanningTrayectory::map->heading == Map::Dir::LEFT){
        PlanningTrayectory::map->heading = Map::Dir::DOWN;
        return PlanningTrayectory::Commands ::LEFT;
    }
    if(next.y == current.y-1 && PlanningTrayectory::map->heading == Map::Dir::LEFT){
        PlanningTrayectory::map->heading = Map::Dir::LEFT;
        return PlanningTrayectory::Commands ::FORWARD;
    }

    if(next.x == current.x+1 && PlanningTrayectory::map->heading == Map::Dir::RIGHT){
        PlanningTrayectory::map->heading = Map::Dir::UP;
        return PlanningTrayectory::Commands ::LEFT;
    }
    if(next.x == current.x-1 && PlanningTrayectory::map->heading == Map::Dir::RIGHT){
        PlanningTrayectory::map->heading = Map::Dir::DOWN;
        return PlanningTrayectory::Commands ::RIGHT;
    }
    if(next.y == current.y+1 && PlanningTrayectory::map->heading == Map::Dir::RIGHT){
        PlanningTrayectory::map->heading = Map::Dir::RIGHT;
        return PlanningTrayectory::Commands ::FORWARD;
    }

    if(next.x == current.x-1 && PlanningTrayectory::map->heading == Map::Dir::DOWN){
        PlanningTrayectory::map->heading = Map::Dir::DOWN;
        return PlanningTrayectory::Commands ::FORWARD;
    }
    if(next.y == current.y+1 && PlanningTrayectory::map->heading == Map::Dir::DOWN){
        PlanningTrayectory::map->heading = Map::Dir::LEFT;
        return PlanningTrayectory::Commands ::LEFT;
    }
    if(next.y == current.y-1 && PlanningTrayectory::map->heading == Map::Dir::DOWN){
        PlanningTrayectory::map->heading = Map::Dir::RIGHT;
        return PlanningTrayectory::Commands ::RIGHT;
    }
}