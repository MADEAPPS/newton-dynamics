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

#ifndef __ND_QUATERNION_H__
#define __ND_QUATERNION_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"

class dMatrix;

class dQuaternion: public dVector
{
	public:
	dQuaternion(); 
	dQuaternion(const dVector& quat);
	dQuaternion(const dQuaternion& quat);
	D_CORE_API dQuaternion (const dMatrix& matrix);
	dQuaternion (dFloat32 q0, dFloat32 q1, dFloat32 q2, dFloat32 q3);
	D_CORE_API dQuaternion (const dVector &unit_Axis, dFloat32 angle);

	dQuaternion Normalize() const;
	dQuaternion Scale(dFloat32 scale) const;
	dQuaternion Inverse () const; 
	dQuaternion operator+ (const dQuaternion &B) const; 
	dQuaternion operator- (const dQuaternion &B) const; 

	D_CORE_API dQuaternion operator* (const dQuaternion &B) const;
	D_CORE_API dQuaternion Slerp(const dQuaternion &q1, dFloat32 t) const;
	D_CORE_API dVector CalcAverageOmega(const dQuaternion &q1, dFloat32 invdt) const;

};

inline dQuaternion::dQuaternion()
	:dVector(dVector::m_wOne)
{
}

inline dQuaternion::dQuaternion(const dVector& quat)
	:dVector(quat)
{
}

inline dQuaternion::dQuaternion(const dQuaternion& quat)
	:dVector(quat)
{
}

inline dQuaternion::dQuaternion(dFloat32 q0, dFloat32 q1, dFloat32 q2, dFloat32 q3)
	:dVector(q0, q1, q2, q3)
{
	*this = Normalize();
}

inline dQuaternion dQuaternion::Inverse () const 
{
	return dQuaternion (-m_x, -m_y, -m_z, m_w);
}

inline dQuaternion dQuaternion::operator+ (const dQuaternion &q) const
{
	//return dQuaternion (m_x + q.m_x, m_y + q.m_y, m_z + q.m_z, m_w + q.m_w);
	return dVector::operator+(q);
}

inline dQuaternion dQuaternion::operator- (const dQuaternion &q) const
{
	//return dQuaternion (m_x - q.m_x, m_y - q.m_y, m_z - q.m_z, m_w - q.m_w);
	return dVector::operator-(q);
}

inline dQuaternion dQuaternion::Normalize() const
{
	return dVector::Normalize();
}

inline dQuaternion dQuaternion::Scale(dFloat32 scale) const
{
	return dVector::Scale(scale);
}

#endif

