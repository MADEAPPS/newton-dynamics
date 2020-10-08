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

#ifndef __D_SKELETON_CONTAINER_H__
#define __D_SKELETON_CONTAINER_H__

#include "ndNewtonStdafx.h"
//#include "dgConstraint.h"
//#include "dgContact.h"
//#include "ndJointBilateralConstraint.h"

//class ndBodyKinematic;

class ndJointBilateralConstraint;

class ndSkeletonContainer 
{
	public:
	D_MSV_NEWTON_ALIGN_32
	class ndForcePair
	{
		public:
		dSpatialVector m_joint;
		dSpatialVector m_body;
	} D_GCC_NEWTON_ALIGN_32;

	D_MSV_NEWTON_ALIGN_32
	class ndMatriData
	{
		public:
		dSpatialMatrix m_jt;
		dSpatialMatrix m_invMass;
	} D_GCC_NEWTON_ALIGN_32;

	D_MSV_NEWTON_ALIGN_32
	class ndBodyJointMatrixDataPair
	{
		public:
		ndMatriData m_body;
		ndMatriData m_joint;
	} D_GCC_NEWTON_ALIGN_32;

	class ndNode
	{
		public:
		ndNode();
		~ndNode();

		ndBodyJointMatrixDataPair m_data;
		ndBodyKinematic* m_body;
		ndJointBilateralConstraint* m_joint;
		ndNode* m_parent;
		ndNode* m_child;
		ndNode* m_sibling;
		union
		{
			dInt64 m_ordinals;
			dInt8 m_sourceJacobianIndex[8];
		};
		dInt16 m_index;
		dInt8 m_dof;
		dInt8 m_swapJacobianBodiesIndex;
		static dInt64 m_ordinalInit;
	};

	class ndNodeList : public dList<ndNode, dContainersFreeListAlloc<ndSkeletonContainer::ndNode> >
	{
		public:
		ndNodeList()
			:dList<ndSkeletonContainer::ndNode, dContainersFreeListAlloc<ndSkeletonContainer::ndNode> >()
		{
		}
	};

	//class ndNodePair;
	//class ndForcePair;
	//class ndMatriData;
	//class ndBodyJointMatrixDataPair;
	class ndNodeList;

	ndSkeletonContainer();
	~ndSkeletonContainer();

	void Init(ndBodyKinematic* const rootBody);

	ndNode* GetRoot() const;
	ndNode* AddChild(ndJointBilateralConstraint* const joint, ndNode* const parent);
	void Finalize(dInt32 loopJoints, ndJointBilateralConstraint** const loopJointArray);

	void ClearSelfCollision();
	void AddSelfCollisionJoint(ndConstraint* const joint);

	private:
	void SortGraph(ndNode* const root, dInt32& index);

#if 0
	ndSkeletonContainer(dgWorld* const world, ndBodyKinematic* const rootBody);
	

	dgWorld* GetWorld() const; 
	dInt32 GetJointCount () const {return m_nodeCount - 1;}
	
	void RemoveLoopJoint(ndJointBilateralConstraint* const joint);  
	
	ndNode* GetRoot () const;
	ndBodyKinematic* GetBody(ndNode* const node) const;
	ndJointBilateralConstraint* GetJoint(ndNode* const node) const;
	ndNode* GetParent (ndNode* const node) const;
	ndNode* GetFirstChild (ndNode* const parent) const;
	ndNode* GetNextSiblingChild (ndNode* const sibling) const;

	dInt32 GetLru() const { return m_lru; }
	void SetLru(dInt32 lru) { m_lru = lru; }

	virtual void CalculateJointForce (dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces);
	virtual void InitMassMatrix (const dgJointInfo* const jointInfoArray, const dgLeftHandSide* const matrixRow, dgRightHandSide* const rightHandSide, bool m_consideredCloseLoop = true);
	
	private:
	bool SanityCheck(const ndForcePair* const force, const ndForcePair* const accel) const;
	void ConditionMassMatrix() const;
	void RebuildMassMatrix(const dgFloat32* const diagDamp) const;
	void CalculateLoopMassMatrixCoefficients(dgFloat32* const diagDamp);
	
	DG_INLINE void CalculateForce(ndForcePair* const force, const ndForcePair* const accel) const;
	DG_INLINE void SolveBackward(ndForcePair* const force) const;
	DG_INLINE void SolveForward(ndForcePair* const force, const ndForcePair* const accel, dInt32 startNode = 0) const;
	DG_INLINE void UpdateForces(dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const ndForcePair* const force) const;
	DG_INLINE void CalculateJointAccel (dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, ndForcePair* const accel) const;

	ndNode* FindNode(ndBodyKinematic* const node) const;
	
		
	void InitLoopMassMatrix (const dgJointInfo* const jointInfoArray);
	dInt8* CalculateBufferSizeInBytes (const dgJointInfo* const jointInfoArray);
	void SolveAuxiliary (const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, const ndForcePair* const accel, ndForcePair* const force) const;
	void SolveLcp(dInt32 stride, dInt32 size, const dgFloat32* const matrix, const dgFloat32* const x0, dgFloat32* const x, const dgFloat32* const b, const dgFloat32* const low, const dgFloat32* const high, const dInt32* const normalIndex) const;
	void SolveBlockLcp(dInt32 size, dInt32 blockSize, const dgFloat32* const x0, dgFloat32* const x, dgFloat32* const b, const dgFloat32* const low, const dgFloat32* const high, const dInt32* const normalIndex) const;
	void FactorizeMatrix(dInt32 size, dInt32 stride, dgFloat32* const matrix, dgFloat32* const diagDamp) const;

	dgWorld* m_world;
	ndNodePair* m_pairs;
	dgFloat32* m_deltaForce;
	dgFloat32* m_massMatrix11;
	dgFloat32* m_massMatrix10;

	dgRightHandSide* m_rightHandSide;
	const dgLeftHandSide* m_leftHandSide;
	dInt32* m_frictionIndex;
	dInt32* m_matrixRowsIndex;
	dgSkeletonList::dgListNode* m_listNode;
	
	dgArray<dInt8> m_auxiliaryMemoryBuffer;
	dInt32 m_lru;
	dInt32 m_blockSize;
	dInt16 m_nodeCount;
	
	dInt16 m_rowCount;
	dInt16 m_loopRowCount;
	dInt16 m_auxiliaryRowCount;
	dInt16 m_consideredCloseLoop;

	friend class dgWorld;
	friend class dgParallelBodySolver;
	friend class dgWorldDynamicUpdate;
#endif

	ndNode* m_skeleton;
	ndNode** m_nodesOrder;
	ndNodeList m_nodeList;
	dArray<ndConstraint*> m_loopingJoints;
	dInt16 m_loopCount;
	dInt16 m_dynamicsLoopCount;
};

inline ndSkeletonContainer::ndNode* ndSkeletonContainer::GetRoot() const
{
	return m_skeleton;
}

#endif


