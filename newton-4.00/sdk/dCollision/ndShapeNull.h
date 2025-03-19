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

#ifndef __ND_SHAPE_NULL_H__ 
#define __ND_SHAPE_NULL_H__ 

#include "ndCollisionStdafx.h"
#include "ndShape.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndShapeNull : public ndShape
{
	public:
	D_CLASS_REFLECTION(ndShapeNull, ndShape)
	D_COLLISION_API ndShapeNull();
	D_COLLISION_API virtual ~ndShapeNull();

	D_COLLISION_API virtual ndShapeNull* GetAsShapeNull();

	D_COLLISION_API virtual ndFloat32 GetVolume() const;
	D_COLLISION_API virtual ndShapeInfo GetShapeInfo() const;
	D_COLLISION_API virtual ndFloat32 GetBoxMinRadius() const;
	D_COLLISION_API virtual ndFloat32 GetBoxMaxRadius() const;
	D_COLLISION_API virtual ndUnsigned64 GetHash(ndUnsigned64 hash) const;
	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	D_COLLISION_API virtual ndVector SupportVertex(const ndVector& dir) const;
	D_COLLISION_API virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	D_COLLISION_API virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32 skinMargin) const;
	D_COLLISION_API virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	D_COLLISION_API virtual ndVector CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& globalPlane, const ndShapeInstance& parentScale) const;
	D_COLLISION_API virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

