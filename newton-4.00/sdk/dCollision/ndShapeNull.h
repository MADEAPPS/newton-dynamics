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

#ifndef __D_SHAPE_NULL_H__ 
#define __D_SHAPE_NULL_H__ 

#include "ndShapeConvex.h"

class ndShapeNull: public ndShapeConvex
{
	public:
	D_COLLISION_API ndShapeNull();
	virtual ~ndShapeNull() {}

	virtual ndShapeNull* GetAsShapeNull() { return this; }
	
	dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const
	{
		return dVector::m_zero;
	}

	dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const
	{
		return dVector::m_zero;
	}

	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
	{
		return dFloat32 (1.2f);
	}

	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
	{
	}

	virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
	{
		p0 = dVector::m_zero;
		p1 = dVector::m_zero;
	}

	virtual ndShapeInfo GetShapeInfo() const
	{
		dAssert(0);
		ndShapeInfo info;
		return info;
	}

	virtual dFloat32 GetBoxMinRadius() const
	{
		return dFloat32(0.0f);
	}

	virtual dFloat32 GetBoxMaxRadius() const
	{
		return dFloat32(0.0f);
	}
};

#endif 

