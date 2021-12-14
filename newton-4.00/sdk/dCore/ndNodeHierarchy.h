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

#ifndef __ND_NODE_HIERARCHY_H__
#define __ND_NODE_HIERARCHY_H__

#include "ndCoreStdafx.h"
#include "ndCRC.h"
#include "ndString.h"
#include "ndContainersAlloc.h"

class ndNodeBaseHierarchy: public ndClassAlloc
{
	public:
	D_CORE_API ndNodeBaseHierarchy* GetChild () const;
	D_CORE_API ndNodeBaseHierarchy* GetParent () const;
	D_CORE_API ndNodeBaseHierarchy* GetSibling () const;

	D_CORE_API void Detach ();
	D_CORE_API void Attach (ndNodeBaseHierarchy* const parent, bool addFirst = false);
	
	D_CORE_API ndNodeBaseHierarchy* GetRoot () const;
	D_CORE_API ndNodeBaseHierarchy* GetFirst() const;
	D_CORE_API ndNodeBaseHierarchy* GetLast() const;
	D_CORE_API ndNodeBaseHierarchy* GetNext() const;
	D_CORE_API ndNodeBaseHierarchy* GetPrev() const;

	D_CORE_API ndNodeBaseHierarchy* Find (ndUnsigned64 nameCRC) const; 
	D_CORE_API ndNodeBaseHierarchy* Find (const char* const name) const;

	ndUnsigned64 GetNameID() const;
	const ndString& GetName() const;
	void SetName(const char* const name);
	
	protected:
	D_CORE_API ndNodeBaseHierarchy ();
	D_CORE_API ndNodeBaseHierarchy (const char* const name);
	D_CORE_API ndNodeBaseHierarchy (const ndNodeBaseHierarchy &clone);
	D_CORE_API virtual  ~ndNodeBaseHierarchy ();

	D_CORE_API virtual  ndNodeBaseHierarchy* CreateClone () const = 0;

	private:
	inline void Clear();

	ndString m_name;
	ndUnsigned64 m_nameID;
	ndNodeBaseHierarchy* m_parent;
	ndNodeBaseHierarchy* m_child;
	ndNodeBaseHierarchy* m_sibling;
};

template<class T>
class ndNodeHierarchy: public ndNodeBaseHierarchy
{
	public:
	ndNodeHierarchy ();
	ndNodeHierarchy (const char* const name);
	void Attach (T* const parent, bool addFirst = false);
	void Detach ();
	T* GetChild () const;
	T* GetParent () const;
	T* GetSibling () const;
	T* GetRoot () const;
	T* GetFirst() const;
	T* GetLast() const;
	T* GetNext() const;
	T* GetPrev() const;
	T* Find (ndUnsigned64 nameCRC) const;
	T* Find (const char* const name) const;

	protected:
	ndNodeHierarchy (const T &clone);
	virtual ~ndNodeHierarchy ();
};


inline ndNodeBaseHierarchy::ndNodeBaseHierarchy ()
{
	Clear ();
}

inline ndNodeBaseHierarchy::ndNodeBaseHierarchy (const char* const name)
{
	Clear ();
	SetName (name);
}

inline void ndNodeBaseHierarchy::Clear()
{
	m_child = nullptr;
	m_parent = nullptr;
	m_sibling = nullptr;
	m_nameID = 0;
	m_name = (char*)nullptr;
}

inline ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetChild () const
{
	return m_child;
}

inline ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetSibling () const
{
	return m_sibling;
}

inline ndNodeBaseHierarchy* ndNodeBaseHierarchy::GetParent () const
{
	return m_parent;
}


inline ndNodeBaseHierarchy* ndNodeBaseHierarchy::Find (const char* const name) const
{
	return Find (dCRC64 (name)); 
} 

inline void ndNodeBaseHierarchy::SetName(const char* const name)
{
	m_name = name;
	m_nameID = dCRC64 (name);
}

inline ndUnsigned64  ndNodeBaseHierarchy::GetNameID() const
{
	return m_nameID;
}

inline const ndString& ndNodeBaseHierarchy::GetName() const
{
	return m_name;
}

template<class T>
ndNodeHierarchy<T>::ndNodeHierarchy ()
	:ndNodeBaseHierarchy ()
{
}

template<class T>
ndNodeHierarchy<T>::ndNodeHierarchy (const T &clone)
	:ndNodeBaseHierarchy (clone)
{
}

template<class T>
ndNodeHierarchy<T>::ndNodeHierarchy (const char* const name)
	:ndNodeBaseHierarchy (name)
{
}

template<class T>
ndNodeHierarchy<T>::~ndNodeHierarchy () 
{
}


//template<class T>
//dNodeBaseHierarchy* ndNodeHierarchy<T>::CreateClone () const
//{
//	return new T (*(T*)this);
//}

template<class T>
void ndNodeHierarchy<T>::Attach (T* const parent, bool addFirst)
{
	ndNodeBaseHierarchy::Attach(parent, addFirst);
}

template<class T>
void ndNodeHierarchy<T>::Detach ()
{
	ndNodeBaseHierarchy::Detach ();
}

template<class T>
T* ndNodeHierarchy<T>::GetChild () const
{
	return (T*) ndNodeBaseHierarchy::GetChild();
}

template<class T>
T* ndNodeHierarchy<T>::GetSibling () const
{
	return (T*) ndNodeBaseHierarchy::GetSibling ();
}

template<class T>
T* ndNodeHierarchy<T>::GetParent () const
{
	return (T*) ndNodeBaseHierarchy::GetParent ();
}


template<class T>
T* ndNodeHierarchy<T>::GetRoot () const
{
	return (T*) ndNodeBaseHierarchy::GetRoot ();
}


template<class T>
T* ndNodeHierarchy<T>::GetFirst() const
{
	return (T*) ndNodeBaseHierarchy::GetFirst ();
}

template<class T>
T* ndNodeHierarchy<T>::GetLast() const
{
	return (T*) ndNodeBaseHierarchy::GetLast ();
}


template<class T>
T* ndNodeHierarchy<T>::GetNext() const
{
	return (T*) ndNodeBaseHierarchy::GetNext ();
}

template<class T>
T* ndNodeHierarchy<T>::GetPrev() const
{
	return (T*) ndNodeBaseHierarchy::GetPrev ();
}


template<class T>
T* ndNodeHierarchy<T>::Find (ndUnsigned64 nameCRC) const 
{
	return (T*) ndNodeBaseHierarchy::Find (nameCRC);
}

template<class T>
T* ndNodeHierarchy<T>::Find (const char* const name) const
{
	return (T*) ndNodeBaseHierarchy::Find (name);
} 


#endif

