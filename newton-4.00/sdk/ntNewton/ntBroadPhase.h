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

#ifndef __D_BROADPHASE_H__
#define __D_BROADPHASE_H__

#include "ntStdafx.h"
#include "ntContactList.h"
#include "ntBroadPhaseNode.h"

#define D_BROADPHASE_MAX_STACK_DEPTH	256

class ntWorld;
class ntContact;
class ntBilateralJoint;

D_MSC_VECTOR_ALIGNMENT
class ntBroadPhase: public dClassAlloc
{
	protected:
	class ntSpliteInfo;
	class ntFitnessList: public dList <ntBroadPhaseTreeNode*>
	{
		public:
		ntFitnessList();
		dFloat64 TotalCost() const;
		
		dFloat64 m_prevCost;
		dInt32 m_index;
	};

	public:
	D_NEWTON_API virtual ~ntBroadPhase();

	virtual void AddBody(ntBody* const body) = 0;
	virtual void RemoveBody(ntBody* const body) = 0;

	D_NEWTON_API void Cleanup();
	D_NEWTON_API virtual void Update(dFloat32 timestep);

	dFloat32 CalculateSurfaceArea(const ntBroadPhaseNode* const node0, const ntBroadPhaseNode* const node1, dVector& minBox, dVector& maxBox) const;

	private:
	void UpdateAabb(dFloat32 timestep);
	void FindCollidingPairs(dFloat32 timestep);

	virtual void BalanceBroadPhase() = 0;
	virtual void UpdateAabb(dInt32 threadIndex, dFloat32 timestep, ntBody* const body);
	virtual void FindCollidinPairs(dInt32 threadIndex, dFloat32 timestep, ntBody* const body) = 0;

	void RotateLeft(ntBroadPhaseTreeNode* const node, ntBroadPhaseNode** const root);
	void RotateRight(ntBroadPhaseTreeNode* const node, ntBroadPhaseNode** const root);
	dFloat64 ReduceEntropy(ntFitnessList& fitness, ntBroadPhaseNode** const root);
	void ImproveNodeFitness(ntBroadPhaseTreeNode* const node, ntBroadPhaseNode** const root);
	static dInt32 CompareNodes(const ntBroadPhaseNode* const nodeA, const ntBroadPhaseNode* const nodeB, void* const);
	ntBroadPhaseNode* BuildTopDown(ntBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ntFitnessList::dListNode** const nextNode);
	ntBroadPhaseNode* BuildTopDownBig(ntBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, ntFitnessList::dListNode** const nextNode);

	protected:
	D_NEWTON_API ntBroadPhase(ntWorld* const world);
	D_NEWTON_API ntBroadPhaseTreeNode* InsertNode (ntBroadPhaseNode* const root, ntBroadPhaseNode* const node);
	D_NEWTON_API void UpdateFitness(ntFitnessList& fitness, dFloat64& oldEntropy, ntBroadPhaseNode** const root);

	ntContact* FindContactJoint(ntBody* const body0, ntBody* const body1) const;
	ntBilateralJoint* FindBilateralJoint(ntBody* const body0, ntBody* const body1) const;

	void AddPair(ntBody* const body0, ntBody* const body1, const dFloat32 timestep);
	bool TestOverlaping(const ntBody* const body0, const ntBody* const body1, dFloat32 timestep) const;
	void SubmitPairs(ntBroadPhaseNode* const leaftNode, ntBroadPhaseNode* const node, dFloat32 timestep);
	
	ntWorld* m_newton;
	ntBroadPhaseNode* m_rootNode;
	ntContactList m_contactList;
	ntContactFreeList m_contactCreator;
	bool m_fullScan;
} D_GCC_VECTOR_ALIGNMENT;

D_INLINE dFloat32 ntBroadPhase::CalculateSurfaceArea(const ntBroadPhaseNode* const node0, const ntBroadPhaseNode* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	dVector side0(maxBox - minBox);
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

#endif
