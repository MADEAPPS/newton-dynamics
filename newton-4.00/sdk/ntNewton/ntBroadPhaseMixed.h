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

#include "ntNewtonStdafx.h"
#include "ntBroadPhase.h"

class ntRayCastNotify;

D_MSV_NEWTON_ALIGN_32
class ntBroadPhaseMixed : public ntBroadPhase
{
	public:
	D_NEWTON_API ntBroadPhaseMixed(ntWorld* const world);
	D_NEWTON_API virtual ~ntBroadPhaseMixed();

	protected:
	D_NEWTON_API virtual void AddBody(ntBodyKinematic* const body);
	D_NEWTON_API virtual void RemoveBody(ntBodyKinematic* const body);
	D_NEWTON_API virtual dFloat32 RayCast(ntRayCastNotify& callback, const dVector& p0, const dVector& p1) const;

	private:
	void AddNode(ntBroadPhaseNode* const newNode);
	void RemoveNode(ntBroadPhaseNode* const newNode);
	void BalanceBroadPhase();
	void FindCollidinPairs(dInt32 threadIndex, dFloat32 timestep, ntBodyKinematic* const body);

	dFloat64 m_treeEntropy;
	ntFitnessList m_fitness;

} D_GCC_NEWTON_ALIGN_32 ;




#endif
