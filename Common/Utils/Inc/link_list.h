/*
 * link_list.h
 *
 *  Created on: 2019年4月25日
 *      Author: Henry Zhu
 */

#ifndef UTILS_LINK_LIST_H_
#define UTILS_LINK_LIST_H_

#include <QMainWindow>
#include "stdlib.h"
#include "node.h"

template <typename TYPE> class LinkList
{
public:
	LinkList();
	virtual ~LinkList();

    void Init();
    void Add(TYPE dat);
    void Delete(void);

    uint32_t Length();//返回链表个数
    Node<TYPE>* getHeadNode();
    void setHeadNode(Node<TYPE>* node);

    Node<TYPE>* getEndNode();

private:
	uint32_t _list_length;
    Node<TYPE>* _head_node;//头节点
    Node<TYPE>* _end_node;//尾节点
    Node<TYPE>* _node;//
};

template <typename TYPE>
LinkList<TYPE>::LinkList() {
    // TODO Auto-generated constructor stub
    _list_length = 0;

    _node = NULL;//头节点
    _end_node  = NULL;//尾节点
    _head_node = NULL;//
}

template <typename TYPE>
LinkList<TYPE>::~LinkList() {
    // TODO Auto-generated destructor stub
//	_list_length = 0;
//	delete _head_node;//头节点
//	delete _end_node;//尾节点
//	delete _node;//

//	_head_node = new Node();
//	_end_node  = new Node();
//	_node      = new Node();
}

template <typename TYPE>
void LinkList<TYPE>::Init()
{
    _list_length = 0;

    _node = NULL;//头节点
    _end_node  = NULL;//尾节点
    _head_node = NULL;//
}

template <typename TYPE>
void LinkList<TYPE>::Add(TYPE dat)
{
    _node = new Node<TYPE>;//申请一个新的节点
    _node->next = NULL;
    if( _node != NULL)
    {
        _node->data = dat;
        if(!_end_node)
        {
            _head_node = _node;//头节点
            _end_node  = _node;//尾节点
        }
        else
        {
            _end_node->next = _node;
            _end_node = _node;
        }
        _list_length++;
    }
}

template<typename TYPE>
void LinkList<TYPE>::Delete(void)
{
    Node<TYPE>* _free_node;//
    if(_list_length > 0)
    {
        while(NULL != _head_node->next)
        {
            _free_node = _head_node;
            _head_node = _head_node->next;
            delete _free_node;
        }
        _list_length = 0;
        delete _head_node;
        _head_node = NULL;
        _end_node  = NULL;
    }
}

//返回链表个数
template <typename TYPE>
uint32_t LinkList<TYPE>::Length() { return _list_length;}

template <typename TYPE>
Node<TYPE>* LinkList<TYPE>::getHeadNode() { return  _head_node;}

template <typename TYPE>
void LinkList<TYPE>::setHeadNode(Node<TYPE>* node){_head_node = node;}

template <typename TYPE>
Node<TYPE>* LinkList<TYPE>::getEndNode()  { return  _end_node; }

#endif /* UTILS_LINK_LIST_H_ */
