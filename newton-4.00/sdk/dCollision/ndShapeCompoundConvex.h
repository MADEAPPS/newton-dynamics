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

#ifndef __D_SHAPE_COMPOUND_CONVEX_H__
#define __D_SHAPE_COMPOUND_CONVEX_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"


class ndShapeCompoundConvex: public ndShape
{
	public:
	D_COLLISION_API ndShapeCompoundConvex();
	D_COLLISION_API virtual ~ndShapeCompoundConvex();

	virtual void GetCollidingFaces(ndPolygonMeshDesc* const data) const = 0;
	protected:
	virtual dFloat32 GetVolume() const;
	virtual dFloat32 GetBoxMinRadius() const;
	virtual dFloat32 GetBoxMaxRadius() const;
	virtual ndShapeCompoundConvex* GetAsShapeCompoundConvex();
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	virtual dVector CalculateVolumeIntegral(const dMatrix& globalMatrix, const dVector& plane, const ndShapeInstance& parentScale) const;

	D_COLLISION_API virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	D_COLLISION_API dInt32 CalculatePlaneIntersection(const dFloat32* const vertex, const dInt32* const index, dInt32 indexCount, dInt32 strideInFloat, const dPlane& localPlane, dVector* const contactsOut) const;

	D_MSV_NEWTON_ALIGN_32 
	class ndMeshVertexListIndexList
	{
		public:
		dInt32* m_indexList;
		dInt32* m_userDataList;
		dFloat32* m_veterxArray;
		dInt32 m_triangleCount; 
		dInt32 m_maxIndexCount;
		dInt32 m_vertexCount;
		dInt32 m_vertexStrideInBytes;
	} D_GCC_NEWTON_ALIGN_32;
};

inline dFloat32 ndShapeCompoundConvex::GetVolume() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeCompoundConvex::GetBoxMinRadius() const
{
	return dFloat32(0.0f);
}

inline dFloat32 ndShapeCompoundConvex::GetBoxMaxRadius() const
{
	return dFloat32(0.0f);
}

inline dVector ndShapeCompoundConvex::SupportVertex(const dVector&, dInt32* const) const
{
	dAssert(0);
	return dVector::m_zero;
}

inline dVector ndShapeCompoundConvex::SupportVertexSpecial(const dVector& dir, dFloat32, dInt32* const vertexIndex) const
{
	dAssert(0);
	return SupportVertex(dir, vertexIndex);
}

inline dVector ndShapeCompoundConvex::SupportVertexSpecialProjectPoint(const dVector& point, const dVector&) const
{ 
	return point; 
}

inline dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dVector&, const dVector&, dVector* const) const
{
	return 0;
}

inline dVector ndShapeCompoundConvex::CalculateVolumeIntegral(const dMatrix&, const dVector&, const ndShapeInstance&) const
{
	return dVector::m_zero;
}

inline ndShapeCompoundConvex* ndShapeCompoundConvex::GetAsShapeCompoundConvex()
{ 
	return this; 
}

#endif 



