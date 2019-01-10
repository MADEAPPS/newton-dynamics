/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_ARRAY__
#define __D_ARRAY__


template<class OBJECT>
class dArray
{
	public:
	dArray()
		:m_capacity(0)
		,m_data(NULL)
	{
	}

	~dArray()
	{
		if (m_data) {
			delete[] m_data;
		}
	}

	OBJECT& operator[] (int i)
	{
		dAssert(i >= 0);
		while (i >= m_capacity) {
			Resize(i * 2);
		}
		return m_data[i];
	}

	const OBJECT& operator[] (int i) const
	{
		dAssert(i >= 0);
		while (i >= m_capacity) {
			Resize(i * 2);
		}
		return m_data[i];
	}

	int GetSize() const
	{
		return m_capacity;
	}

	void Resize(int size) const
	{
		if (size >= m_capacity) {
			//size = dMax(size, 16);
			OBJECT* const newArray = new OBJECT[size];
			if (m_data) {
				for (int i = 0; i < m_capacity; i++) {
					newArray[i] = m_data[i];
				}
				delete[] m_data;
			}
			m_data = newArray;
			m_capacity = size;
		} else if (size < m_capacity) {
			//size = dMax(size, 16);
			OBJECT* const newArray = new OBJECT[size];
			if (m_data) {
				for (int i = 0; i < size; i++) {
					newArray[i] = m_data[i];
				}
				delete[] m_data;
			}
			m_data = newArray;
			m_capacity = size;
		}
	}

	mutable int m_capacity;
	mutable OBJECT* m_data;
};

#endif
