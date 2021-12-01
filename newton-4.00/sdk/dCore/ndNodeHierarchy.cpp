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

dNodeBaseHierarchy::dNodeBaseHierarchy (const dNodeBaseHierarchy &clone)
{
	Clear ();
	SetName (clone.m_name.GetStr());
	for (dNodeBaseHierarchy* obj = clone.m_child; obj; obj = obj->m_sibling) 
	{
		dNodeBaseHierarchy* const newObj = obj->CreateClone ();
		newObj->Attach (this);
	}
}

dNodeBaseHierarchy::~dNodeBaseHierarchy () 
{
	Detach();
	while (m_child) 
	{
		delete m_child;
	}
}

void dNodeBaseHierarchy::Attach (dNodeBaseHierarchy* const parentArg, bool addFirst)
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
			dNodeBaseHierarchy* obj = m_parent->m_child;
			for (; obj->m_sibling; obj = obj->m_sibling);
			obj->m_sibling = this;
		}
	} 
	else 
	{
		m_parent->m_child = this;
	}
}

void dNodeBaseHierarchy::Detach ()
{
 	if (m_parent) 
	{
		if (m_parent->m_child == this) 
		{
			m_parent->m_child = m_sibling;
		} 
		else 
		{
			dNodeBaseHierarchy* ptr = m_parent->m_child;
			for (; ptr->m_sibling != this; ptr = ptr->m_sibling);
			ptr->m_sibling = m_sibling;
		}
		m_parent = nullptr;
		m_sibling = nullptr;
	}
}
	
dNodeBaseHierarchy* dNodeBaseHierarchy::GetRoot() const
{
	const dNodeBaseHierarchy* root = this;
	for (; root->m_parent; root = root->m_parent);
	return (dNodeBaseHierarchy*)root;
}

dNodeBaseHierarchy* dNodeBaseHierarchy::GetFirst() const
{
	dNodeBaseHierarchy* ptr = (dNodeBaseHierarchy*) this;
	for (; ptr->m_child; ptr = ptr->m_child);
	return ptr;
}

dNodeBaseHierarchy* dNodeBaseHierarchy::GetNext() const
{
	if (m_sibling) 
	{
		return m_sibling->GetFirst();
	}

	dNodeBaseHierarchy* ptr = m_parent;
	dNodeBaseHierarchy* x = (dNodeBaseHierarchy *)this;
	for (; ptr && (x == ptr->m_sibling); ptr = ptr->m_parent) 
	{
		x = ptr;
	}
	return ptr;
}

dNodeBaseHierarchy* dNodeBaseHierarchy::GetLast() const
{
	dNodeBaseHierarchy* ptr = (dNodeBaseHierarchy*) this;
	for (; ptr->m_sibling; ptr = ptr->m_sibling);
	return ptr;
}

dNodeBaseHierarchy* dNodeBaseHierarchy::GetPrev() const
{
	if (m_child) 
	{
		return m_child->GetNext();
	}

	dNodeBaseHierarchy* ptr = m_parent;
	dNodeBaseHierarchy* x = (dNodeBaseHierarchy *)this;
	for (; ptr && (x == ptr->m_child); ptr = ptr->m_child) 
	{
		x = ptr;
	}
	return ptr;
}

dNodeBaseHierarchy* dNodeBaseHierarchy::Find (dUnsigned64 nameCRC) const 
{
	if (nameCRC == GetNameID()) 
	{
		return (dNodeBaseHierarchy*)this;
	} 
	else 
	{
		for (dNodeBaseHierarchy* ptr = GetFirst(); ptr && (ptr != this); ptr = ptr->GetNext()) 
		{
			if (nameCRC == ptr->GetNameID()) 
			{
				return ptr;
			}
		}
	}

	return nullptr;
}







