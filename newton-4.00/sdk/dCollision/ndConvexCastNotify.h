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

#ifndef __ND_CONVEXCAST_NOTIFY_H__
#define __ND_CONVEXCAST_NOTIFY_H__

#include "ndCollisionStdafx.h"
#include "ndContact.h"

class ndBody;
class ndScene;
class ndShapeInstance;

D_MSV_NEWTON_ALIGN_32
class ndConvexCastNotify : public ndClassAlloc
{
	public: 
	ndConvexCastNotify()
		:m_normal(ndVector::m_zero)
		,m_closestPoint0(ndVector::m_zero)
		,m_closestPoint1(ndVector::m_zero)
		,m_contacts()
		,m_param(ndFloat32 (1.2f))
	{
	}

	virtual ~ndConvexCastNotify()
	{
	}
	
	virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const, const ndShapeInstance* const)
	{
		dAssert(0);
		return 1;
	}

	//virtual ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam)
	virtual ndFloat32 OnRayCastAction(const ndContactPoint&, ndFloat32)
	{
		dAssert(0);
		return 0;
	}

	D_COLLISION_API bool CastShape(const ndShapeInstance& castingInstance, const ndMatrix& globalOrigin, const ndVector& globalDest, ndBodyKinematic* const targetBody);
	D_COLLISION_API bool CastShape(const ndShapeInstance& castingInstance, const ndMatrix& globalOrigin, const ndVector& globalDest, const ndShapeInstance& targetShape, const ndMatrix& targetMatrix);
	
	ndVector m_normal;
	ndVector m_closestPoint0;
	ndVector m_closestPoint1;
	ndFixSizeArray<ndContactPoint, 8> m_contacts;
	ndFloat32 m_param;
} D_GCC_NEWTON_ALIGN_32;

#endif
