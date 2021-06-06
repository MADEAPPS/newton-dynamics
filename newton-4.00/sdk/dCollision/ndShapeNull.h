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

#ifndef __D_SHAPE_NULL_H__ 
#define __D_SHAPE_NULL_H__ 

#include "ndCollisionStdafx.h"
#include "ndShape.h"

class ndShapeNull : public ndShape
{
	public:
	ndShapeNull();
	virtual ~ndShapeNull();

	virtual ndShapeNull* GetAsShapeNull();

	virtual dFloat32 GetVolume() const;
	virtual ndShapeInfo GetShapeInfo() const;
	virtual dFloat32 GetBoxMinRadius() const;
	virtual dFloat32 GetBoxMaxRadius() const;
	virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	virtual dVector CalculateVolumeIntegral(const dMatrix& globalMatrix, const dVector& globalPlane, const ndShapeInstance& parentScale) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
};

inline ndShapeNull::ndShapeNull()
	:ndShape(m_nullCollision)
{
	m_inertia = dVector::m_one | dVector::m_triplexMask;
}

inline ndShapeNull::~ndShapeNull()
{
}

inline ndShapeNull* ndShapeNull::GetAsShapeNull()
{ 
	return this; 
}

inline dVector ndShapeNull::SupportVertex(const dVector&, dInt32* const) const
{
	return dVector::m_zero;
}

inline dVector ndShapeNull::SupportVertexSpecial(const dVector&, dFloat32, dInt32* const) const
{
	return dVector::m_zero;
}

inline dFloat32 ndShapeNull::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	return dFloat32(1.2f);
}

inline void ndShapeNull::DebugShape(const dMatrix&, ndShapeDebugCallback&) const
{
}

inline void ndShapeNull::CalcAABB(const dMatrix&, dVector& p0, dVector& p1) const
{
	p0 = dVector::m_zero;
	p1 = dVector::m_zero;
}

inline ndShapeInfo ndShapeNull::GetShapeInfo() const
{
	dAssert(0);
	ndShapeInfo info;
	return info;
}

inline dFloat32 ndShapeNull::GetVolume() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeNull::GetBoxMinRadius() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeNull::GetBoxMaxRadius() const
{
	return dFloat32(0.0f);
}

inline dVector ndShapeNull::CalculateVolumeIntegral(const dMatrix&, const dVector&, const ndShapeInstance&) const
{
	return dVector::m_zero;
}

inline dVector ndShapeNull::SupportVertexSpecialProjectPoint(const dVector& point, const dVector&) const
{
	return point;
}

inline dInt32 ndShapeNull::CalculatePlaneIntersection(const dVector&, const dVector&, dVector* const) const
{
	return 0;
}

#endif 

