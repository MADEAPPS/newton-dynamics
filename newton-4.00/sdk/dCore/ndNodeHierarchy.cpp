/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndCoreStdafx.h"
#include "ndNodeHierarchy.h"

ndNodeBaseHierarchy::ndNodeBaseHierarchy (const ndNodeBaseHierarchy &clone)
{
	Clear ();
	SetName (clone.m_name.GetStr());
	for (ndNodeBaseHierarchy* obj = clone.m_child; obj; obj = obj->m_sibling) 
	{
		ndNodeBaseHierarchy* const newObj = obj->CreateClone ();
		newObj->Attach (this);
	}
}

ndNodeBaseHierarchy::~ndNodeBaseHierarchy () 
{
	Detach();
	while (m_child) 
	{
		delete m_child;
	}
}

void ndNodeBaseHierarchy::Attach (ndNodeBaseHierarchy* const parentArg, bool addFirst)
{
	m_parent = parentArg;
	if (m_parent->m_child) 
	{
		if (addFirst) 
		{
			m_sibling = m_parent->m_child;
			m_parent->m_child = this;
		} 
		else 
		{
			ndNodeBaseHierarchy* obj = m_parent->m_child;
			for (; obj->m_sibling; obj = obj->m_sibling);
			obj->m_sibling = this;
		}
	} 
	else 
	{
		m_parent->m_child = this;
	}
}

void ndNodeBaseHierarchy::Detach ()
{
 	if (m_parent) 
	{
		if (m_parent->m_child == this) 
		{
			m_parent->m_child = m_sibling;
		} 
		else 
		{
			ndNodeBaseHierarchy* ptr = m_parent->m_child;
			for (; ptr->m_sibling != this; ptr = ptr->m_sibling);
			ptr->m_sibling = m_sibling;
		}
		m_parent = nullptr;
		m_sibling = nullptr;
	}
}
	
ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetRoot() const
{
	const ndNodeBaseHierarchy* root = this;
	for (; root->m_parent; root = root->m_parent);
	return (ndNodeBaseHierarchy*)root;
}

ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetFirst() const
{
	ndNodeBaseHierarchy* ptr = (ndNodeBaseHierarchy*) this;
	for (; ptr->m_child; ptr = ptr->m_child);
	return ptr;
}

ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetNext() const
{
	if (m_sibling) 
	{
		return m_sibling->GetFirst();
	}

	ndNodeBaseHierarchy* ptr = m_parent;
	ndNodeBaseHierarchy* x = (ndNodeBaseHierarchy *)this;
	for (; ptr && (x == ptr->m_sibling); ptr = ptr->m_parent) 
	{
		x = ptr;
	}
	return ptr;
}

ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetLast() const
{
	ndNodeBaseHierarchy* ptr = (ndNodeBaseHierarchy*) this;
	for (; ptr->m_sibling; ptr = ptr->m_sibling);
	return ptr;
}

ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetPrev() const
{
	if (m_child) 
	{
		return m_child->GetNext();
	}

	ndNodeBaseHierarchy* ptr = m_parent;
	ndNodeBaseHierarchy* x = (ndNodeBaseHierarchy *)this;
	for (; ptr && (x == ptr->m_child); ptr = ptr->m_child) 
	{
		x = ptr;
	}
	return ptr;
}

ndNodeBaseHierarchy* ndNodeBaseHierarchy::Find (dUnsigned64 nameCRC) const 
{
	if (nameCRC == GetNameID()) 
	{
		return (ndNodeBaseHierarchy*)this;
	} 
	else 
	{
		for (ndNodeBaseHierarchy* ptr = GetFirst(); ptr && (ptr != this); ptr = ptr->GetNext()) 
		{
			if (nameCRC == ptr->GetNameID()) 
			{
				return ptr;
			}
		}
	}

	return nullptr;
}







