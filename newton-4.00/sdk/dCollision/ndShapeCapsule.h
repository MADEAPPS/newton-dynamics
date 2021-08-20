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

#ifndef __D_SHAPE_CAPSULE_H__
#define __D_SHAPE_CAPSULE_H__

#include "ndShapeConvex.h"

D_MSV_NEWTON_ALIGN_32
class ndShapeCapsule : public ndShapeConvex
{
	public:
	D_CLASS_REFLECTION(ndShapeCapsule);
	D_COLLISION_API ndShapeCapsule(const dLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API ndShapeCapsule (dFloat32 radio0, dFloat32 radio1, dFloat32 height);

	virtual ndShapeCapsule* GetAsShapeCapsule() { return this; }

	protected:
	D_COLLISION_API void Init (dFloat32 radio0, dFloat32 radio1, dFloat32 height);

	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual void CalculateAabb(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	D_COLLISION_API virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	D_COLLISION_API virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	D_COLLISION_API virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	D_COLLISION_API virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeHash) const;

	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	void TesselateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dInt32& count, dVector* ouput) const;
	
	dVector m_p0;
	dVector m_p1;
	dVector m_normal;
	dVector m_transform;
	dFloat32 m_height;
	dFloat32 m_radius0;
	dFloat32 m_radius1;
} D_GCC_NEWTON_ALIGN_32;

#endif 

