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

#ifndef __D_BROADPHASE_DEFAULT_H__
#define __D_BROADPHASE_DEFAULT_H__

#include "ndCollisionStdafx.h"
#include "ndScene.h"

class ndRayCastNotify;

D_MSV_NEWTON_ALIGN_32
class ndSceneMixed : public ndScene
{
	public:
	D_COLLISION_API ndSceneMixed();
	D_COLLISION_API virtual ~ndSceneMixed();

	protected:
	D_COLLISION_API virtual bool AddBody(ndBodyKinematic* const body);
	D_COLLISION_API virtual bool RemoveBody(ndBodyKinematic* const body);
	D_COLLISION_API virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& p0, const dVector& p1) const;
	D_COLLISION_API virtual void Cleanup();
	D_COLLISION_API void BalanceBroadPhase();

	private:
	D_COLLISION_API void AddNode(ndSceneNode* const newNode);
	D_COLLISION_API void RemoveNode(ndSceneNode* const newNode);
	D_COLLISION_API void FindCollidinPairs(dInt32 threadIndex, ndBodyKinematic* const body, bool oneWay);

	dFloat64 m_treeEntropy;
	ndFitnessList m_fitness;

} D_GCC_NEWTON_ALIGN_32 ;




#endif
