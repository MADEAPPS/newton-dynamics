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

#include "dNewtonStdafx.h"
#include "dBroadPhase.h"

#if 0
class dBroadPhaseMixed: public dBroadPhase
{
	public:
	dBroadPhaseMixed(dNewton* const world);
	virtual ~dBroadPhaseMixed();

	protected:
	virtual dgInt32 GetType() const;
	virtual void Add(dgBody* const body);
	virtual void Remove(dgBody* const body);
	virtual void UpdateFitness();
	virtual void InvalidateCache();
	virtual dBroadPhaseAggregate* CreateAggregate();
	virtual void DestroyAggregate(dBroadPhaseAggregate* const aggregate);

	virtual void LinkAggregate (dBroadPhaseAggregate* const aggregate); 
	virtual void UnlinkAggregate (dBroadPhaseAggregate* const aggregate); 
	virtual void CheckStaticDynamic(dgBody* const body, dgFloat32 mass) {}
	virtual void FindCollidingPairs (dgBroadphaseSyncDescriptor* const descriptor, dgList<dBroadPhaseNode*>::dgListNode* const node, dgInt32 threadID);

	void RayCast (const dgVector& p0, const dgVector& p1, OnRayCastAction filter, OnRayPrecastAction prefilter, void* const userData) const;
	dgInt32 Collide(dgCollisionInstance* const shape, const dgMatrix& matrix, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const;
	dgInt32 ConvexCast (dgCollisionInstance* const shape, const dgMatrix& p0, const dgVector& p1, dgFloat32* const param, OnRayPrecastAction prefilter, void* const userData, dgConvexCastReturnInfo* const info, dgInt32 maxContacts, dgInt32 threadIndex) const;
	void ForEachBodyInAABB (const dgVector& q0, const dgVector& q1, OnBodiesInAABB callback, void* const userData) const;

	void ResetEntropy ();
	void AddNode(dBroadPhaseNode* const node);	
	void RemoveNode(dBroadPhaseNode* const node);	

	void ApplyDeformableForceAndtorque (dgBroadphaseSyncDescriptor* const descriptor, dgInt32 threadID);
	
	dgFloat64 m_treeEntropy;
	dgFitnessList m_fitness;
	
};
#endif

D_MSC_VECTOR_ALIGNMENT
class dBroadPhaseMixed : public dBroadPhase
{
	public:
		D_NEWTON_API dBroadPhaseMixed(dNewton* const world);
	D_NEWTON_API virtual ~dBroadPhaseMixed();

	D_NEWTON_API virtual void AddBody(dBody* const body);
} D_GCC_VECTOR_ALIGNMENT;




#endif
