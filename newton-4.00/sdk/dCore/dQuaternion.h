/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __dQuaternion__
#define __dQuaternion__

#include "dCoreStdafx.h"
#include "dTypes.h"

class dVector;
class dMatrix;

D_MSC_VECTOR_ALIGNMENT
class dQuaternion: public dClassAlloc
{
	public:
	dQuaternion (); 
	dQuaternion (const dMatrix& matrix);
	dQuaternion (dFloat32 q0, dFloat32 q1, dFloat32 q2, dFloat32 q3); 
	dQuaternion (const dVector &unit_Axis, dFloat32 angle = dFloat32 (0.0f));

//	dFloat32& operator[] (dgInt32 i);
//	const dFloat32& operator[] (dgInt32 i) const;

	void Scale (dFloat32 scale); 
	void Normalize (); 
	dQuaternion Inverse () const; 
	dQuaternion Slerp (const dQuaternion &q1, dFloat32 t) const;

	dFloat32 DotProduct (const dQuaternion &QB) const;
	dVector CalcAverageOmega (const dQuaternion &q1, dFloat32 invdt) const;

	dQuaternion operator* (const dQuaternion &B) const;
	dQuaternion operator+ (const dQuaternion &B) const; 
	dQuaternion operator- (const dQuaternion &B) const; 

	dFloat32 m_x;
	dFloat32 m_y;
	dFloat32 m_z;
	dFloat32 m_w;
} D_GCC_VECTOR_ALIGNMENT;


D_INLINE dQuaternion::dQuaternion()
	:m_x(dFloat32(0.0f))
	,m_y(dFloat32(0.0f))
	,m_z(dFloat32(0.0f))
	,m_w(dFloat32(1.0f))
{
}

D_INLINE dQuaternion::dQuaternion(dFloat32 Q0, dFloat32 Q1, dFloat32 Q2, dFloat32 Q3)
	:m_x(Q1)
	,m_y(Q2)
	,m_z(Q3)
	,m_w(Q0)
{
//	dgAssert (dgAbs (DotProduct (*this) -dFloat32 (1.0f)) < dFloat32(1.0e-4f));
}

/*
D_INLINE dFloat32& dQuaternion::operator[] (dgInt32 i)
{
	dgAssert(i < 4);
	dgAssert(i >= 0);
	return (&m_w)[i];
}

D_INLINE const dFloat32& dQuaternion::operator[] (dgInt32 i) const
{
	dgAssert(i < 4);
	dgAssert(i >= 0);
	return (&m_w)[i];
}
*/

D_INLINE void dQuaternion::Scale (dFloat32 scale) 
{
	m_w *= scale;
	m_x *= scale;
	m_y *= scale;
	m_z *= scale;
}

D_INLINE void dQuaternion::Normalize () 
{
	Scale (dRsqrt (DotProduct (*this)));
}

D_INLINE dFloat32 dQuaternion::DotProduct (const dQuaternion &q1) const
{
	return m_w * q1.m_w + m_x * q1.m_x + m_y * q1.m_y + m_z * q1.m_z;
}

D_INLINE dQuaternion dQuaternion::Inverse () const 
{
	return dQuaternion (m_w, -m_x, -m_y, -m_z);
}

D_INLINE dQuaternion dQuaternion::operator+ (const dQuaternion &q) const
{
	return dQuaternion (m_w + q.m_w, m_x + q.m_x, m_y + q.m_y, m_z + q.m_z);
}

D_INLINE dQuaternion dQuaternion::operator- (const dQuaternion &q) const
{
	return dQuaternion (m_w - q.m_w, m_x - q.m_x, m_y - q.m_y, m_z - q.m_z);
}

D_INLINE dQuaternion dQuaternion::operator* (const dQuaternion &q) const
{
	return dQuaternion (q.m_w * m_w - q.m_x * m_x - q.m_y * m_y - q.m_z * m_z, 
				 		 q.m_x * m_w + q.m_w * m_x - q.m_z * m_y + q.m_y * m_z, 
						 q.m_y * m_w + q.m_z * m_x + q.m_w * m_y - q.m_x * m_z, 
						 q.m_z * m_w - q.m_y * m_x + q.m_x * m_y + q.m_w * m_z); 
}

#endif

