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

    /// Data of the node.
	T data;
    /// Node parent of the node,
	Node<T> *parent;
	/// Set if is marked or not.
	bool marked;

	//Constructor and destructor
    /// Constructor of the node.
	Node(){
		marked = false;
		parent = nullptr;
	}
	/// Destructor of the node.
	virtual ~Node(){
		if(parent != nullptr)
			delete parent;
	}

};

#endif /* ODOMETRY_MAP_SRC_NODE_H_ */
