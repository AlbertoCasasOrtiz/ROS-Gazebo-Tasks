/*
 * Node.h
 *
 *  Created on: 3 sept. 2019
 *      Author: alberto
 */

#ifndef ODOMETRY_MAP_SRC_NODE_H_
	#include <vector>
#define ODOMETRY_MAP_SRC_NODE_H_

template <class T>
class Node {
public:


	T data;
	Node<T> *parent;
	bool marked;
	std::vector<Node<T>*> adyacents;


	Node(){
		marked = false;
		parent = nullptr;
	}

	virtual ~Node(){
		if(parent != nullptr)
			delete parent;
	}

};

#endif /* ODOMETRY_MAP_SRC_NODE_H_ */
