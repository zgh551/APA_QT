/*
 * node.h
 *
 *  Created on: 2019年4月25日
 *      Author: Henry Zhu
 */

#ifndef UTILS_NODE_H_
#define UTILS_NODE_H_

#include "./Interaction/Ultrasonic/Ultrasonic.h"

template <typename TYPE> class Node
{
public:
	Node();
	virtual ~Node();

    TYPE data;//值域
	Node *next;//指针域，指向下一个节点的指针
};

template <typename TYPE>
Node<TYPE>::Node() {
    // TODO Auto-generated constructor stub
    next = NULL;
}

template <typename TYPE>
Node<TYPE>::~Node() {
    // TODO Auto-generated destructor stub
}

#endif /* UTILS_NODE_H_ */
