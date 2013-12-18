/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __dVector__
#define __dVector__


#include "dMathDefines.h"


// small but very effective 4 dimensional template vector class 

template<class T>
class TemplateVector
{
	public:
	TemplateVector ();
	TemplateVector (const T *ptr);
	TemplateVector (T m_x, T m_y, T m_z, T m_w); 
	TemplateVector Scale (T s) const;

	T& operator[] (int i);
	const T& operator[] (int i) const;

	TemplateVector operator+ (const TemplateVector &A) const; 
	TemplateVector operator- (const TemplateVector &A) const; 
	TemplateVector &operator+= (const TemplateVector &A);
	TemplateVector &operator-= (const TemplateVector &A); 

	// return dot product
	T operator% (const TemplateVector &A) const; 

	// return cross product
	TemplateVector operator* (const TemplateVector &B) const; 

	// component wise multiplication
	TemplateVector CompProduct (const TemplateVector &A) const;

	T m_x;
	T m_y;
	T m_z;
	T m_w;
};

D_MSC_VECTOR_ALIGMENT
class dVector: public TemplateVector<dFloat>
{
	public:
	dVector();
	dVector (const TemplateVector<dFloat>& v);
	dVector (const dFloat *ptr);
	dVector (dFloat x, dFloat y, dFloat z, dFloat w = 1.0); 
};


class dBigVector: public TemplateVector<double>
{
	public: 
	dBigVector(){};
	dBigVector (const TemplateVector<dFloat>& v)
		:TemplateVector<dFloat64> (v.m_x, v.m_y, v.m_z, v.m_w)
	{
	}

#ifndef _NEWTON_USE_DOUBLE
	dBigVector (const TemplateVector<dFloat64>& v)
		:TemplateVector<dFloat64> (v.m_x, v.m_y, v.m_z, v.m_w)
	{
	}
#endif

	//	dBigVector (const dFloat *ptr);
	dBigVector (dFloat x, dFloat y, dFloat z, dFloat w = 1.0)
		:TemplateVector<dFloat64> (x, y, z, w)
	{
	}

#ifndef _NEWTON_USE_DOUBLE
	dBigVector (dFloat64 x, dFloat64 y, dFloat64 z, dFloat64 w = 1.0)
		:TemplateVector<dFloat64> (x, y, z, w)
	{
	}
#endif
};




template<class T>
TemplateVector<T>::TemplateVector() {}

template<class T>
TemplateVector<T>::TemplateVector(const T *ptr)
	:m_x (ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w(0.0f)
{
}

template<class T>
TemplateVector<T>::TemplateVector(T x, T y, T z, T w) 
	:m_x (x), m_y(y), m_z(z), m_w(w)
{
}


template<class T>
T& TemplateVector<T>::operator[] (int i)
{
	return (&m_x)[i];
}	

template<class T>
const T& TemplateVector<T>::operator[] (int i) const
{
	return (&m_x)[i];
}

template<class T>
TemplateVector<T> TemplateVector<T>::Scale (T scale) const
{
	return TemplateVector<T> (m_x * scale, m_y * scale, m_z * scale, m_w);
}


template<class T>
TemplateVector<T> TemplateVector<T>::operator+ (const TemplateVector<T>& B) const
{
	return TemplateVector<T> (m_x + B.m_x, m_y + B.m_y, m_z + B.m_z, m_w);
}

template<class T>
TemplateVector<T>& TemplateVector<T>::operator+= (const TemplateVector<T>& A) 
{
	m_x += A.m_x;
	m_y += A.m_y;
	m_z += A.m_z;
	return *this;
}

template<class T>
TemplateVector<T> TemplateVector<T>::operator- (const TemplateVector<T>& A) const
{
	return TemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w);
}

template<class T>
TemplateVector<T>& TemplateVector<T>::operator-= (const TemplateVector<T>& A) 
{
	m_x -= A.m_x;
	m_y -= A.m_y;
	m_z -= A.m_z;
	return *this;
}


template<class T>
T TemplateVector<T>::operator% (const TemplateVector<T>& A) const
{
	return m_x * A.m_x + m_y * A.m_y + m_z * A.m_z;
}


template<class T>
TemplateVector<T> TemplateVector<T>::operator* (const TemplateVector<T>& B) const
{
	return TemplateVector<T> (m_y * B.m_z - m_z * B.m_y,
 							    m_z * B.m_x - m_x * B.m_z,
								m_x * B.m_y - m_y * B.m_x, m_w);
}



template<class T>
TemplateVector<T> TemplateVector<T>::CompProduct (const TemplateVector<T>& A) const
{
	return TemplateVector<T> (m_x * A.m_x, m_y * A.m_y, m_z * A.m_z, A.m_w);
}



inline dVector::dVector()
	:TemplateVector<dFloat>()
{
}

inline dVector::dVector (const TemplateVector<dFloat>& v)
	:TemplateVector<dFloat>(v)
{
}

inline dVector::dVector (const dFloat *ptr)
	:TemplateVector<dFloat>(ptr)
{
}

inline dVector::dVector (dFloat x, dFloat y, dFloat z, dFloat w) 
	:TemplateVector<dFloat>(x, y, z, w)
{
}

#endif

