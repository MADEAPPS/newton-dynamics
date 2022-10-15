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

#ifndef _ND_SHARED_PTR_H_
#define _ND_SHARED_PTR_H_

template <typename T>
class ndSharedPtr
{
	private:	
	class ndRefCounter : public ndClassAlloc, public ndAtomic<ndInt32>
	{
		public: 
		ndRefCounter();
		void AddRef();
		ndInt32 Release();
	};

	public:
	ndSharedPtr();
	ndSharedPtr(T* const ptr);
	ndSharedPtr(const ndSharedPtr<T>& sp);
	~ndSharedPtr();
	ndSharedPtr<T>& operator = (const ndSharedPtr<T>& sp);

	void Swap(ndSharedPtr& src);

	T* operator->();

	T* operator* ();
	const T* operator* () const;

	private:
	T* m_ptr;
	ndRefCounter* m_references;
};

template <typename T>
ndSharedPtr<T>::ndRefCounter::ndRefCounter()
	:ndClassAlloc()
	,ndAtomic<ndInt32>(0)
{
}

template <typename T>
void ndSharedPtr<T>::ndRefCounter::AddRef()
{
	fetch_add(1);
}

template <typename T>
ndInt32 ndSharedPtr<T>::ndRefCounter::Release()
{
	int ref = fetch_add(-1);
	return ref - 1;
}

template <typename T>
ndSharedPtr<T>::ndSharedPtr()
	:m_ptr(nullptr)
	,m_references(new ndRefCounter)
{
	m_references->AddRef();
}

template <typename T>
ndSharedPtr<T>::ndSharedPtr(T* const ptr)
	:m_ptr(ptr)
	,m_references(new ndRefCounter)
{
	m_references->AddRef();
}

template <typename T>
ndSharedPtr<T>::ndSharedPtr(const ndSharedPtr<T>& sp)
	:m_ptr(sp.m_ptr)
	,m_references(sp.m_references)
{
	m_references->AddRef();
}

template <typename T>
ndSharedPtr<T>::~ndSharedPtr()
{
	int ref = m_references->Release();
	if (ref == 0)
	{
		if (m_ptr)
		{
			delete m_ptr;
		}
		delete m_references;
	}
}

template <typename T>
ndSharedPtr<T>& ndSharedPtr<T>::operator = (const ndSharedPtr<T>& sp)
{
	if (this != &sp)
	{
		if (m_references->Release() == 0)
		{
			if (m_ptr)
			{
				delete m_ptr;
			}
			delete m_references;
		}

		m_ptr = sp.m_ptr;
		m_references = sp.m_references;
		m_references->AddRef();
	}
	return *this;
}

template <typename T>
void ndSharedPtr<T>::Swap(ndSharedPtr& src)
{
	ndSwap(m_ptr, src.m_ptr);
	ndSwap(m_references, src.m_references);
}

template <typename T>
T* ndSharedPtr<T>::operator* ()
{
	return m_ptr;
}

template <typename T>
const T* ndSharedPtr<T>::operator* () const
{
	return m_ptr;
}

template <typename T>
T* ndSharedPtr<T>::operator-> ()
{
	return m_ptr;
}

#endif 

