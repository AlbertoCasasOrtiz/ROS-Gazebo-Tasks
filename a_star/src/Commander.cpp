//
// Created by alberto on 5/09/19.
//

#include <string>
#include <fstream>
#include "Commander.h"
#include "ros/ros.h"

std::string Commander::commandToString(Commander::Commands command) {
    switch (command) {
        case Commands::FORWARD:
            return "FORWARD";
        case Commands::RIGHT:
            return "RIGHT";
        case Commands::LEFT:
            return "LEFT";
    }
}

std::string Commander::dirToString(Commander::Dir direction){
    switch(direction){
        case Dir::UP:
            return "UP";
        case Dir::DOWN:
            return "DOWN";
        case Dir::LEFT:
            return "LEFT";
        case Dir::RIGHT:
            return "RIGHT";
    }
}

std::vector<Commander::Commands> Commander::calculateCommands(std::vector<Node<Point>*> path) {
    std::vector<Commander::Commands> commands;
    for(int i = 1; i < path.size(); i++){
        Commander::Commands command = Commander::nextPointCommand(path.at(i-1)->point, path.at(i)->point);
        commands.push_back(command);
        if(command == Commander::Commands::LEFT || command == Commander::Commands::RIGHT)
            commands.push_back(Commands::FORWARD);
        }

    return commands;
}


Commander::Commands Commander::nextPointCommand(Point current, Point next) {

    ROS_INFO("HEADING TO: [%s]", Commander::dirToString(Commander::heading).c_str());

    if (next.y > current.y && Commander::heading == Commander::Dir::UP) {
        Commander::heading = Commander::Dir::UP;
        return Commander::Commands::FORWARD;
    }
    if (next.x < current.x && Commander::heading == Commander::Dir::UP) {
        Commander::heading = Commander::Dir::RIGHT;
        return Commander::Commands::LEFT;
    }
    if (next.x > current.x && Commander::heading == Commander::Dir::UP) {
        Commander::heading = Commander::Dir::LEFT;
        return Commander::Commands::RIGHT;
    }

    if (next.y > current.y && Commander::heading == Commander::Dir::RIGHT) {
        Commander::heading = Commander::Dir::DOWN;
        return Commander::Commands::RIGHT;
    }
    if (next.y < current.y && Commander::heading == Commander::Dir::RIGHT) {
        Commander::heading = Commander::Dir::UP;
        return Commander::Commands::LEFT;
    }
    if (next.x < current.x && Commander::heading == Commander::Dir::RIGHT) {
        Commander::heading = Commander::Dir::RIGHT;
        return Commander::Commands::FORWARD;
    }

    if (next.y > current.y && Commander::heading == Commander::Dir::LEFT) {
        Commander::heading = Commander::Dir::DOWN;
        return Commander::Commands::LEFT;
    }
    if (next.y < current.y && Commander::heading == Commander::Dir::LEFT) {
        Commander::heading = Commander::Dir::UP;
        return Commander::Commands::RIGHT;
    }
    if (next.x > current.x && Commander::heading == Commander::Dir::LEFT) {
        Commander::heading = Commander::Dir::LEFT;
        return Commander::Commands::FORWARD;
    }

    if (next.y < current.y && Commander::heading == Commander::Dir::DOWN) {
        Commander::heading = Commander::Dir::UP;
        return Commander::Commands::FORWARD;
    }
    if (next.x > current.x && Commander::heading == Commander::Dir::DOWN) {
        Commander::heading = Commander::Dir::RIGHT;
        return Commander::Commands::LEFT;
    }
    if (next.x < current.x && Commander::heading == Commander::Dir::DOWN) {
        Commander::heading = Commander::Dir::LEFT;
        return Commander::Commands::RIGHT;
    }

    return Commander::Commands ::FORWARD;
}


void Commander::printCommands(std::vector<Commander::Commands> commands) {
    std::ofstream file("commands.txt");
    for(int i = 0; i < commands.size(); i++){
        Commander::Commands command = commands.at(i);
        file << Commander::commandToString(command) << "\n";
        ROS_INFO("%s", Commander::commandToString(command).c_str());
    }
    file.close();

}

Commander::Commander() {
    Commander::heading = Commander::Dir::DOWN;
}
