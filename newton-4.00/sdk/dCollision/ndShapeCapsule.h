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
	D_COLLISION_API ndShapeCapsule (dFloat32 radio0, dFloat32 radio1, dFloat32 height);

	virtual ndShapeCapsule* GetAsShapeCapsule() { return this; }

	protected:
	D_COLLISION_API void Init (dFloat32 radio0, dFloat32 radio1, dFloat32 height);

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	D_COLLISION_API virtual ndVector SupportVertex(const ndVector& dir, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecial(const ndVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	virtual dInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	void TesselateTriangle(dInt32 level, const ndVector& p0, const ndVector& p1, const ndVector& p2, dInt32& count, ndVector* ouput) const;
	
	ndVector m_p0;
	ndVector m_p1;
	ndVector m_normal;
	ndVector m_transform;
	dFloat32 m_height;
	dFloat32 m_radius0;
	dFloat32 m_radius1;
} D_GCC_NEWTON_ALIGN_32;

#endif 

