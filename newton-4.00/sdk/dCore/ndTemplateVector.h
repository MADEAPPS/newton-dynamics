/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __ND_TEMPLATE_VECTOR_H__
#define __ND_TEMPLATE_VECTOR_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndClassAlloc.h"

//#define dCheckVector(x) (dCheckFloat(x[0]) && dCheckFloat(x[1]) && dCheckFloat(x[2]) && dCheckFloat(x[3]))

template<class T>
class ndTemplateVector: public ndClassAlloc
{
	public:
	ndTemplateVector () 
	{
	}
	
	ndTemplateVector (const T* const ptr)
		:m_x(ptr[0]), m_y(ptr[1]), m_z(ptr[2]), m_w (ptr[3])
	{
		//dAssert (dCheckVector ((*this)));
	}

	ndTemplateVector (const ndTemplateVector<T>& copy)
		:m_x(copy.m_x), m_y(copy.m_y), m_z(copy.m_z), m_w (copy.m_w)
	{
		//	dAssert (dCheckVector ((*this)));
	}

	ndTemplateVector (T x, T y, T z, T w) 
		:m_x(x), m_y(y), m_z(z), m_w (w)
	{
	}
	
	T& operator[] (dInt32 i)
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}	

	const T& operator[] (dInt32 i) const
	{
		dAssert (i < 4);
		dAssert (i >= 0);
		return (&m_x)[i];
	}

	T GetScalar() const
	{
		return m_x;
	}

	ndTemplateVector<T> Scale (T scale) const
	{
		return ndTemplateVector<T> (m_x * scale, m_y * scale, m_z * scale, m_w * scale);
	}

	ndTemplateVector<T> operator+ (const ndTemplateVector<T>& B) const
	{
		return ndTemplateVector<T> (m_x + B.m_x, m_y + B.m_y, m_z + B.m_z, m_w + B.m_w);
	}

	ndTemplateVector<T>& operator+= (const ndTemplateVector<T>& A) 
	{
		return (*this = ndTemplateVector<T> (m_x + A.m_x, m_y + A.m_y, m_z + A.m_z, m_w + A.m_w));
	}

	ndTemplateVector<T> operator- (const ndTemplateVector<T>& A) const
	{
		return ndTemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w);
	}

	ndTemplateVector<T>& operator-= (const ndTemplateVector<T>& A) 
	{
		return (*this = ndTemplateVector<T> (m_x - A.m_x, m_y - A.m_y, m_z - A.m_z, m_w - A.m_w));
	}

	ndTemplateVector<T> operator* (const ndTemplateVector<T>& B) const
	{
		return ndTemplateVector<T>(m_x * B.m_x, m_y * B.m_y, m_z * B.m_z, m_w * B.m_w);
	}

	ndTemplateVector<T> operator*= (const ndTemplateVector<T>& B) const
	{
		return (*this = ndTemplateVector<T>(m_x * B.m_x, m_y * B.m_y, m_z * B.m_z, m_w * B.m_w));
	}

	ndTemplateVector<T> AddHorizontal() const
	{
		T val(m_x + m_y + m_z + m_w);
		return ndTemplateVector<T>(val, val, val, val);
	}

	ndTemplateVector<T> MulAdd(const ndTemplateVector<T>& A, const ndTemplateVector<T>& B) const
	{
		return *this + A * B;
	}

	ndTemplateVector<T> MulSub(const ndTemplateVector<T>& A, const ndTemplateVector<T>& B) const
	{
		return *this - A * B;
	}

	// return cross product
	ndTemplateVector<T> CrossProduct (const ndTemplateVector<T>& B) const
	{
		return ndTemplateVector<T> (m_y * B.m_z - m_z * B.m_y,
									m_z * B.m_x - m_x * B.m_z,
									m_x * B.m_y - m_y * B.m_x, m_w);
	}

	ndTemplateVector<T> CrossProduct(const ndTemplateVector &A, const ndTemplateVector &B) const
	{
		T cofactor[3][3];
		T array[4][4];

		const ndTemplateVector<T>& me = *this;
		for (dInt32 i = 0; i < 4; i++) {
			array[0][i] = me[i];
			array[1][i] = A[i];
			array[2][i] = B[i];
			array[3][i] = T(1.0f);
		}

		ndTemplateVector<T> normal;
		T sign = T(-1.0f);
		for (dInt32 i = 0; i < 4; i++) 
		{
			for (dInt32 j = 0; j < 3; j++) 
			{
				dInt32 k0 = 0;
				for (dInt32 k = 0; k < 4; k++) 
				{
					if (k != i) 
					{
						cofactor[j][k0] = array[j][k];
						k0++;
					}
				}
			}
			T x = cofactor[0][0] * (cofactor[1][1] * cofactor[2][2] - cofactor[1][2] * cofactor[2][1]);
			T y = cofactor[0][1] * (cofactor[1][2] * cofactor[2][0] - cofactor[1][0] * cofactor[2][2]);
			T z = cofactor[0][2] * (cofactor[1][0] * cofactor[2][1] - cofactor[1][1] * cofactor[2][0]);
			T det = x + y + z;

			normal[i] = sign * det;
			sign *= T(-1.0f);
		}

		return normal;
	}

	// return dot 4d dot product
	ndTemplateVector<T> DotProduct (const ndTemplateVector &A) const
	{
		T val (m_x * A.m_x + m_y * A.m_y + m_z * A.m_z + m_w * A.m_w);
		return ndTemplateVector<T> (val, val, val, val);
	}


	T GetMax () const
	{
		return dMax(dMax(m_x, m_y), dMax(m_z, m_w));
	}

	ndTemplateVector<T> GetMax(const ndTemplateVector<T>& data) const
	{
		return ndTemplateVector<T>((m_x > data.m_x) ? m_x : data.m_x, (m_y > data.m_y) ? m_y : data.m_y, (m_z > data.m_z) ? m_z : data.m_z,	(m_w > data.m_w) ? m_w : data.m_w);
	}

	ndTemplateVector<T> GetMin(const ndTemplateVector<T>& data) const
	{
		return ndTemplateVector<T>((m_x < data.m_x) ? m_x : data.m_x, (m_y < data.m_y) ? m_y : data.m_y, (m_z < data.m_z) ? m_z : data.m_z,	(m_w < data.m_w) ? m_w : data.m_w);
	}

	// check validity of floats
#ifdef _DEBUG
	void Trace (char* const name) const
	{
		dTrace (("%s %f %f %f %f\n", name, m_x, m_y, m_z, m_w));
	}
#endif

	T m_x;
	T m_y;
	T m_z;
	T m_w;
};

#endif