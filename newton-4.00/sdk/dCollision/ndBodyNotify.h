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

#ifndef __ND_BODY_NOTIFY_H__
#define __ND_BODY_NOTIFY_H__

#include "ndCollisionStdafx.h"

class ndBody;
class ndUrdfBodyNotify;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndBodyNotify : public ndContainersFreeListAlloc<ndBodyNotify>
{
	public:  
	D_BASE_CLASS_REFLECTION(ndBodyNotify)

	D_COLLISION_API ndBodyNotify(const ndBodyNotify& src);
	D_COLLISION_API ndBodyNotify(const ndVector& defaultGravity);
	D_COLLISION_API virtual ~ndBodyNotify();
	D_COLLISION_API virtual ndBodyNotify* Clone() const;

	D_COLLISION_API ndBody* GetBody();
	D_COLLISION_API const ndBody* GetBody() const;
	D_COLLISION_API virtual ndBodyNotify* GetAsBodyNotify();
	D_COLLISION_API virtual ndUrdfBodyNotify* GetAsUrdfBodyNotify();

	D_COLLISION_API ndVector GetGravity() const;
	D_COLLISION_API void SetGravity(const ndVector& defaultGravity);

	D_COLLISION_API virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const;
	D_COLLISION_API virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);
	D_COLLISION_API virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	private:
	ndVector m_defaultGravity;
	ndBody* m_body;

	friend class ndBody;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif 

