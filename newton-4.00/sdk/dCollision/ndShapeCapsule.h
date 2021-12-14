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

#ifndef __ND_SHAPE_CAPSULE_H__
#define __ND_SHAPE_CAPSULE_H__

#include "ndShapeConvex.h"

D_MSV_NEWTON_ALIGN_32
class ndShapeCapsule : public ndShapeConvex
{
	public:
	D_CLASS_REFLECTION(ndShapeCapsule);
	D_COLLISION_API ndShapeCapsule(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API ndShapeCapsule (ndFloat32 radio0, ndFloat32 radio1, ndFloat32 height);

	virtual ndShapeCapsule* GetAsShapeCapsule() { return this; }

	protected:
	D_COLLISION_API void Init (ndFloat32 radio0, ndFloat32 radio1, ndFloat32 height);

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	D_COLLISION_API virtual ndVector SupportVertex(const ndVector& dir, ndInt32* const vertexIndex) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32 skinThickness, ndInt32* const vertexIndex) const;
	D_COLLISION_API virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	void TesselateTriangle(ndInt32 level, const ndVector& p0, const ndVector& p1, const ndVector& p2, ndInt32& count, ndVector* ouput) const;
	
	ndVector m_p0;
	ndVector m_p1;
	ndVector m_normal;
	ndVector m_transform;
	ndFloat32 m_height;
	ndFloat32 m_radius0;
	ndFloat32 m_radius1;
} D_GCC_NEWTON_ALIGN_32;

#endif 

