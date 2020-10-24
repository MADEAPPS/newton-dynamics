/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_NODE_HIERARCHY_H__
#define __D_NODE_HIERARCHY_H__

#include "dCoreStdafx.h"
#include "dCRC.h"
#include "dString.h"
#include "dContainersAlloc.h"

class dNodeBaseHierarchy: public dClassAlloc
{
	public:
	D_CORE_API dNodeBaseHierarchy* GetChild () const;
	D_CORE_API dNodeBaseHierarchy* GetParent () const;
	D_CORE_API dNodeBaseHierarchy* GetSibling () const;

	D_CORE_API void Detach ();
	D_CORE_API void Attach (dNodeBaseHierarchy* const parent, bool addFirst = false);
	
	D_CORE_API dNodeBaseHierarchy* GetRoot () const;
	D_CORE_API dNodeBaseHierarchy* GetFirst() const;
	D_CORE_API dNodeBaseHierarchy* GetLast() const;
	D_CORE_API dNodeBaseHierarchy* GetNext() const;
	D_CORE_API dNodeBaseHierarchy* GetPrev() const;

	D_CORE_API dNodeBaseHierarchy* Find (dUnsigned64 nameCRC) const; 
	D_CORE_API dNodeBaseHierarchy* Find (const char* const name) const;

	dUnsigned64 GetNameID() const;
	const dString& GetName() const;
	void SetName(const char* const name);
	
	protected:
	D_CORE_API dNodeBaseHierarchy ();
	D_CORE_API dNodeBaseHierarchy (const char* const name);
	D_CORE_API dNodeBaseHierarchy (const dNodeBaseHierarchy &clone);
	D_CORE_API virtual  ~dNodeBaseHierarchy ();

	D_CORE_API virtual  dNodeBaseHierarchy* CreateClone () const = 0;

	private:
	inline void Clear();

	dString m_name;
	dUnsigned64 m_nameID;
	dNodeBaseHierarchy* m_parent;
	dNodeBaseHierarchy* m_child;
	dNodeBaseHierarchy* m_sibling;
};

template<class T>
class dNodeHierarchy: public dNodeBaseHierarchy
{
	public:
	dNodeHierarchy ();
	dNodeHierarchy (const char* const name);
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
	T* Find (dUnsigned64 nameCRC) const;
	T* Find (const char* const name) const;

	protected:
	dNodeHierarchy (const T &clone);
	virtual ~dNodeHierarchy ();
};


inline dNodeBaseHierarchy::dNodeBaseHierarchy ()
{
	Clear ();
}

inline dNodeBaseHierarchy::dNodeBaseHierarchy (const char* const name)
{
	Clear ();
	SetName (name);
}


inline void dNodeBaseHierarchy::Clear()
{
	m_child = nullptr;
	m_parent = nullptr;
	m_sibling = nullptr;
	m_nameID = 0;
	m_name = (char*)nullptr;
}


inline dNodeBaseHierarchy* dNodeBaseHierarchy::GetChild () const
{
	return m_child;
}

inline dNodeBaseHierarchy* dNodeBaseHierarchy::GetSibling () const
{
	return m_sibling;
}

inline dNodeBaseHierarchy* dNodeBaseHierarchy::GetParent () const
{
	return m_parent;
}


inline dNodeBaseHierarchy* dNodeBaseHierarchy::Find (const char* const name) const
{
	return Find (dCRC64 (name)); 
} 

inline void dNodeBaseHierarchy::SetName(const char* const name)
{
	m_name = name;
	m_nameID = dCRC64 (name);
}

inline dUnsigned64  dNodeBaseHierarchy::GetNameID() const
{
	return m_nameID;
}

inline const dString& dNodeBaseHierarchy::GetName() const
{
	return m_name;
}


template<class T>
dNodeHierarchy<T>::dNodeHierarchy ()
	:dNodeBaseHierarchy ()
{
}

template<class T>
dNodeHierarchy<T>::dNodeHierarchy (const T &clone)
	:dNodeBaseHierarchy (clone)
{
}

template<class T>
dNodeHierarchy<T>::dNodeHierarchy (const char* const name)
	:dNodeBaseHierarchy (name)
{
}

template<class T>
dNodeHierarchy<T>::~dNodeHierarchy () 
{
}


//template<class T>
//dNodeBaseHierarchy* dNodeHierarchy<T>::CreateClone () const
//{
//	return new T (*(T*)this);
//}

template<class T>
void dNodeHierarchy<T>::Attach (T* const parent, bool addFirst)
{
	dNodeBaseHierarchy::Attach(parent, addFirst);
}

template<class T>
void dNodeHierarchy<T>::Detach ()
{
	dNodeBaseHierarchy::Detach ();
}

template<class T>
T* dNodeHierarchy<T>::GetChild () const
{
	return (T*) dNodeBaseHierarchy::GetChild();
}

template<class T>
T* dNodeHierarchy<T>::GetSibling () const
{
	return (T*) dNodeBaseHierarchy::GetSibling ();
}

template<class T>
T* dNodeHierarchy<T>::GetParent () const
{
	return (T*) dNodeBaseHierarchy::GetParent ();
}


template<class T>
T* dNodeHierarchy<T>::GetRoot () const
{
	return (T*) dNodeBaseHierarchy::GetRoot ();
}


template<class T>
T* dNodeHierarchy<T>::GetFirst() const
{
	return (T*) dNodeBaseHierarchy::GetFirst ();
}

template<class T>
T* dNodeHierarchy<T>::GetLast() const
{
	return (T*) dNodeBaseHierarchy::GetLast ();
}


template<class T>
T* dNodeHierarchy<T>::GetNext() const
{
	return (T*) dNodeBaseHierarchy::GetNext ();
}

template<class T>
T* dNodeHierarchy<T>::GetPrev() const
{
	return (T*) dNodeBaseHierarchy::GetPrev ();
}


template<class T>
T* dNodeHierarchy<T>::Find (dUnsigned64 nameCRC) const 
{
	return (T*) dNodeBaseHierarchy::Find (nameCRC);
}

template<class T>
T* dNodeHierarchy<T>::Find (const char* const name) const
{
	return (T*) dNodeBaseHierarchy::Find (name);
} 


#endif

