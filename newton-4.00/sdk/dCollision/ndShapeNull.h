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

#ifndef __ND_SHAPE_NULL_H__ 
#define __ND_SHAPE_NULL_H__ 

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
	virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	virtual ndVector SupportVertex(const ndVector& dir, dInt32* const vertexIndex) const;
	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	virtual ndVector SupportVertexSpecial(const ndVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	virtual ndVector CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& globalPlane, const ndShapeInstance& parentScale) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
};

inline ndShapeNull::ndShapeNull()
	:ndShape(m_nullCollision)
{
	m_inertia = ndVector::m_one | ndVector::m_triplexMask;
}

inline ndShapeNull::~ndShapeNull()
{
}

inline ndShapeNull* ndShapeNull::GetAsShapeNull()
{ 
	return this; 
}

inline ndVector ndShapeNull::SupportVertex(const ndVector&, dInt32* const) const
{
	return ndVector::m_zero;
}

inline ndVector ndShapeNull::SupportVertexSpecial(const ndVector&, dFloat32, dInt32* const) const
{
	return ndVector::m_zero;
}

inline dFloat32 ndShapeNull::RayCast(ndRayCastNotify&, const ndVector&, const ndVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	return dFloat32(1.2f);
}

inline void ndShapeNull::DebugShape(const ndMatrix&, ndShapeDebugNotify&) const
{
}

inline void ndShapeNull::CalculateAabb(const ndMatrix&, ndVector& p0, ndVector& p1) const
{
	p0 = ndVector::m_zero;
	p1 = ndVector::m_zero;
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

inline ndVector ndShapeNull::CalculateVolumeIntegral(const ndMatrix&, const ndVector&, const ndShapeInstance&) const
{
	return ndVector::m_zero;
}

inline ndVector ndShapeNull::SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector&) const
{
	return point;
}

inline dInt32 ndShapeNull::CalculatePlaneIntersection(const ndVector&, const ndVector&, ndVector* const) const
{
	return 0;
}

#endif 

