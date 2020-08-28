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

#include "dNewtonStdafx.h"
#include "dBroadPhaseNode.h"
class dNewton;

D_MSC_VECTOR_ALIGNMENT
class dBroadPhase: public dClassAlloc
{
	protected:
	class dSpliteInfo;

	class dFitnessList: public dList <dBroadPhaseTreeNode*>
	{
		public:
		dFitnessList();
		dFloat64 TotalCost() const;
		
		dFloat64 m_prevCost;
		dInt32 m_index;
	};

	public:
	D_NEWTON_API virtual ~dBroadPhase();

	virtual void AddBody(dBody* const body) = 0;
	virtual void RemoveBody(dBody* const body) = 0;

	D_NEWTON_API virtual void Update(dFloat32 timestep);

	dFloat32 CalculateSurfaceArea(const dBroadPhaseNode* const node0, const dBroadPhaseNode* const node1, dVector& minBox, dVector& maxBox) const;

	private:
	void UpdateAabb(dFloat32 timestep);
	virtual void BalanceBroadPhase() = 0;
	void UpdateAabb(dInt32 threadIndex, dFloat32 timestep, dBody* const body);


	void RotateLeft(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root);
	void RotateRight(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root);
	dFloat64 ReduceEntropy(dFitnessList& fitness, dBroadPhaseNode** const root);
	void ImproveNodeFitness(dBroadPhaseTreeNode* const node, dBroadPhaseNode** const root);
	static dInt32 CompareNodes(const dBroadPhaseNode* const nodeA, const dBroadPhaseNode* const nodeB, void* const);
	dBroadPhaseNode* BuildTopDown(dBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, dFitnessList::dListNode** const nextNode);
	dBroadPhaseNode* BuildTopDownBig(dBroadPhaseNode** const leafArray, dInt32 firstBox, dInt32 lastBox, dFitnessList::dListNode** const nextNode);

	protected:
	D_NEWTON_API dBroadPhase(dNewton* const world);
	D_NEWTON_API dBroadPhaseTreeNode* InsertNode (dBroadPhaseNode* const root, dBroadPhaseNode* const node);
	D_NEWTON_API void UpdateFitness(dFitnessList& fitness, dFloat64& oldEntropy, dBroadPhaseNode** const root);

	dNewton* m_newton;
	dBroadPhaseNode* m_rootNode;
} D_GCC_VECTOR_ALIGNMENT;

D_INLINE dFloat32 dBroadPhase::CalculateSurfaceArea(const dBroadPhaseNode* const node0, const dBroadPhaseNode* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_minBox.GetMin(node1->m_minBox);
	maxBox = node0->m_maxBox.GetMax(node1->m_maxBox);
	dVector side0(maxBox - minBox);
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}

#endif
