/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_QUATERNION_H__
#define __ND_QUATERNION_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"

class ndMatrix;

class ndQuaternion: public ndVector
{
	public:
	ndQuaternion(); 
	ndQuaternion(const ndVector& quat);
	//ndQuaternion(const ndQuaternion& quat);
	D_CORE_API ndQuaternion (const ndMatrix& matrix);
	ndQuaternion (ndFloat32 q0, ndFloat32 q1, ndFloat32 q2, ndFloat32 q3);
	D_CORE_API ndQuaternion (const ndVector &unit_Axis, ndFloat32 angle);

	ndQuaternion Normalize() const;
	ndQuaternion Scale(ndFloat32 scale) const;
	ndQuaternion Inverse () const; 
	ndQuaternion operator+ (const ndQuaternion &B) const; 
	ndQuaternion operator- (const ndQuaternion &B) const; 

	D_CORE_API ndQuaternion operator* (const ndQuaternion &B) const;

	D_CORE_API ndVector GetEulerAngles(ndVector& euler) const;
	D_CORE_API ndVector RotateVector(const ndVector& point) const;
	D_CORE_API ndVector UnrotateVector(const ndVector& point) const;
	D_CORE_API ndQuaternion Slerp(const ndQuaternion &q1, ndFloat32 t) const;
	D_CORE_API ndVector CalcAverageOmega(const ndQuaternion &q1, ndFloat32 invdt) const;
	D_CORE_API ndQuaternion IntegrateOmega(const ndVector& omega, ndFloat32 timestep) const;
};

inline ndQuaternion::ndQuaternion()
	//:ndVector(ndVector::m_wOne)
	:ndVector(ndFloat32 (0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f))
{
}

inline ndQuaternion::ndQuaternion(const ndVector& quat)
	:ndVector(quat)
{
}

//inline ndQuaternion::ndQuaternion(const ndQuaternion& quat)
//	:ndVector(quat)
//{
//}

inline ndQuaternion::ndQuaternion(ndFloat32 q0, ndFloat32 q1, ndFloat32 q2, ndFloat32 q3)
	:ndVector(q0, q1, q2, q3)
{
	*this = Normalize();
}

inline ndQuaternion ndQuaternion::Inverse () const 
{
	return ndQuaternion (-m_x, -m_y, -m_z, m_w);
}

inline ndQuaternion ndQuaternion::operator+ (const ndQuaternion &q) const
{
	//return ndQuaternion (m_x + q.m_x, m_y + q.m_y, m_z + q.m_z, m_w + q.m_w);
	return ndVector::operator+(q);
}

inline ndQuaternion ndQuaternion::operator- (const ndQuaternion &q) const
{
	//return ndQuaternion (m_x - q.m_x, m_y - q.m_y, m_z - q.m_z, m_w - q.m_w);
	return ndVector::operator-(q);
}

inline ndQuaternion ndQuaternion::Normalize() const
{
	return ndVector::Normalize();
}

inline ndQuaternion ndQuaternion::Scale(ndFloat32 scale) const
{
	return ndVector::Scale(scale);
}

#endif

