/*
 * property.h
 *
 *  Created on:
 *      Author: Henry Zhu
 */
#include <assert.h>

#ifndef _PROPERTY_H_
#define _PROPERTY_H_

#define READ_ONLY  1
#define WRITE_ONLY 2
#define READ_WRITE 3

template <typename Container, typename ValueType, int nPropType>
class Property {
public:
	Property()
	{
		m_cObject = 0;
		Set = 0;
		Get = 0;
	}
	//-- This to set a pointer to the class that contain the
	//聽聽 property --
	void setContainer(Container* cObject)
	{
		m_cObject = cObject;
	}
	//-- Set the set member function that will change the value --
	void setter(void (Container::*pSet)(ValueType value))
	{
		if ((nPropType == WRITE_ONLY) || (nPropType == READ_WRITE))
			Set = pSet;
		else
			Set = 0;
	}

	//-- Set the get member function that will retrieve the value --
	void getter(ValueType(Container::*pGet)())
	{
		if ((nPropType == READ_ONLY) || (nPropType == READ_WRITE))
			Get = pGet;
		else
			Get = 0;
	}
	//-- Overload the '=' sign to set the value using the set
	//聽聽 member --
	ValueType operator =(const ValueType& value)
	{
		assert(m_cObject != 0);
		assert(Set != 0);
		(m_cObject->*Set)(value);
		return value;
	}

	//-- To make possible to cast the property class to the
	//聽聽 internal type --
	operator ValueType()
	{
		assert(m_cObject != 0);
		assert(Get != 0);
		return (m_cObject->*Get)();
	}
private:
	Container* m_cObject;
	//-- Pointer to the module that
    //聽聽 contains the property --
	void (Container::*Set)(ValueType value);
	//-- Pointer to set member function --
	ValueType(Container::*Get)();
	//-- Pointer to get member function --
};

#endif /* SRC_PROPERTY_H_ */
