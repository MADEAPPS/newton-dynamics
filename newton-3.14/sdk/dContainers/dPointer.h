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


#ifndef __D_POINTER_H__
#define __D_POINTER_H__


template <typename T>
class dPointer
{
	public:
	dPointer();
	dPointer(T* const data);
	~dPointer();
	T& operator* ();
	T* operator-> ();

	T* GetData() const;
	void SetData(T* const data);

	private:
	T* m_data; 
};

template<class T>
dPointer<T>::dPointer()
	:m_data(NULL)
{
}


template<class T>
dPointer<T>::dPointer(T* const data)
	:m_data(data)
{
}

template<class T>
dPointer<T>::~dPointer()
{
	if (m_data) {
		delete m_data;
	}
}

template<class T>
T& dPointer<T>::operator* ()
{
	return *m_data;
}

template<class T>
T* dPointer<T>::operator-> ()
{
	return m_data;
}

template<class T>
T* dPointer<T>::GetData() const
{
	return m_data;
}

template<class T>
void dPointer<T>::SetData(T* const data)
{
	m_data = data;
}

#endif