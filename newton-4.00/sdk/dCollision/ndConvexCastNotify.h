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

#ifndef __D_CONVEXCAST_NOTIFY_H__
#define __D_CONVEXCAST_NOTIFY_H__

#include "ndCollisionStdafx.h"
//#include "ndBody.h"
#include "ndContact.h"
//#include "ndScene.h"
//#include "ndShapeInstance.h"

class ndBody;
class ndScene;
class ndShapeInstance;

D_MSV_NEWTON_ALIGN_32
class ndConvexCastNotify
{
	public: 
	ndConvexCastNotify(const ndScene* const scene)
		:m_normal(dVector::m_zero)
		,m_closetPoint0(dVector::m_zero)
		,m_closetPoint1(dVector::m_zero)
		,m_contacts()
		,m_param(dFloat32 (1.2f))
		,m_scene(scene)
	{
	}

	virtual ~ndConvexCastNotify()
	{
	}
	
	virtual dUnsigned32 OnRayPrecastAction(const ndBody* const, const ndShapeInstance* const)
	{
		dAssert(0);
		return 1;
	}

	//virtual dFloat32 OnRayCastAction(const ndContactPoint& contact, dFloat32 intersetParam)
	virtual dFloat32 OnRayCastAction(const ndContactPoint&, dFloat32)
	{
		dAssert(0);
		return 0;
	}

	D_COLLISION_API dFloat32 CastShape(const ndShapeInstance& convexShape, const dMatrix& globalOrigin, const dVector& globalDest);
	
	dVector m_normal;
	dVector m_closetPoint0;
	dVector m_closetPoint1;
	dFixSizeArray<ndContactPoint, 8> m_contacts;
	dFloat32 m_param;

	protected:
	const ndScene* m_scene;
} D_GCC_NEWTON_ALIGN_32;

//D_MSV_NEWTON_ALIGN_32
//class ndRayCastClosestHitCallback: public ndConvexCastNotify
//{
//	public:
//	ndRayCastClosestHitCallback(const ndScene* const scene)
//		:ndConvexCastNotify(scene)
//		,m_param(dFloat32(1.2f))
//	{
//	}
//
//	dUnsigned32 OnRayPrecastAction(const ndBody* const, const ndShapeInstance* const)
//	{
//		//return ((ndBody*)body)->GetAsBodyKinematic() ? 1 : 0;
//		return 1;
//	}
//
//	dFloat32 OnRayCastAction(const ndContactPoint& contact, dFloat32 intersetParam)
//	{
//		if (intersetParam < m_param)
//		{
//			m_contact = contact;
//			m_param = intersetParam;
//		}
//		return intersetParam;
//	}
//
//	ndContactPoint m_contact;
//	dFloat32 m_param;
//} D_GCC_NEWTON_ALIGN_32 ;


#endif
