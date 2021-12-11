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

#ifndef __ND_SKELETON_CONTAINER_H__
#define __ND_SKELETON_CONTAINER_H__

#include "ndNewtonStdafx.h"

class ndJointBilateralConstraint;

class ndSkeletonContainer 
{
	public:
	class ndNodePair
	{
		public:
		dInt32 m_m0;
		dInt32 m_m1;
	};

	D_MSV_NEWTON_ALIGN_32
	class ndForcePair
	{
		public:
		ndSpatialVector m_joint;
		ndSpatialVector m_body;
	} D_GCC_NEWTON_ALIGN_32;

	D_MSV_NEWTON_ALIGN_32
	class ndMatriData
	{
		public:
		ndSpatialMatrix m_jt;
		ndSpatialMatrix m_invMass;
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
		dInt32 Factorize(const ndLeftHandSide* const leftHandSide, const ndRightHandSide* const rightHandSide, ndSpatialMatrix* const bodyMassArray, ndSpatialMatrix* const jointMassArray);

		inline void CalculateJacobianBlock();
		inline void CalculateInertiaMatrix(ndSpatialMatrix* const bodyMassArray) const;
		inline void CalculateJointDiagonal(const ndSpatialMatrix* const bodyMassArray, ndSpatialMatrix* const jointMassArray);
		inline void CalculateBodyDiagonal(ndNode* const child, ndSpatialMatrix* const bodyMassArray, const ndSpatialMatrix* const jointMassArray);
		inline void GetJacobians(const ndLeftHandSide* const leftHandSide, const ndRightHandSide* const rightHandSide, ndSpatialMatrix* const jointMassArray);

		inline void BodyDiagInvTimeSolution(ndForcePair& force);
		inline void JointDiagInvTimeSolution(ndForcePair& force);
		inline void JointJacobianTimeMassForward(ndForcePair& force);
		inline void BodyJacobianTimeSolutionBackward(ndForcePair& force) const;
		inline dInt32 GetAuxiliaryRows(const ndRightHandSide* const rightHandSide) const;
		inline void BodyJacobianTimeMassForward(const ndForcePair& force, ndForcePair& parentForce) const;
		inline void JointJacobianTimeSolutionBackward(ndForcePair& force, const ndForcePair& parentForce) const;

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

	class ndNodeList : public ndList<ndNode, ndContainersFreeListAlloc<ndSkeletonContainer::ndNode> >
	{
		public:
		ndNodeList()
			:ndList<ndSkeletonContainer::ndNode, ndContainersFreeListAlloc<ndSkeletonContainer::ndNode> >()
		{
		}
	};

	ndSkeletonContainer();
	~ndSkeletonContainer();

	void Init(ndBodyKinematic* const rootBody);

	ndNode* GetRoot() const;
	ndNode* AddChild(ndJointBilateralConstraint* const joint, ndNode* const parent);
	void Finalize(dInt32 loopJoints, ndJointBilateralConstraint** const loopJointArray);

	void ClearSelfCollision();
	void AddSelfCollisionJoint(ndConstraint* const joint);
	void CalculateJointForce(const ndBodyKinematic** const bodyArray, ndJacobian* const internalForces);
	void InitMassMatrix(const ndLeftHandSide* const matrixRow, ndRightHandSide* const rightHandSide, bool m_consideredCloseLoop = true);

	void CheckSleepState();

	private:
	void InitLoopMassMatrix();
	void CalculateBufferSizeInBytes();
	void ConditionMassMatrix() const;
	void SortGraph(ndNode* const root, dInt32& index);
	void RebuildMassMatrix(const dFloat32* const diagDamp) const;
	void CalculateLoopMassMatrixCoefficients(dFloat32* const diagDamp);
	void FactorizeMatrix(dInt32 size, dInt32 stride, dFloat32* const matrix, dFloat32* const diagDamp) const;
	void SolveAuxiliary(ndJacobian* const internalForces, const ndForcePair* const accel, ndForcePair* const force) const;
	void SolveBlockLcp(dInt32 size, dInt32 blockSize, const dFloat32* const x0, dFloat32* const x, dFloat32* const b, const dFloat32* const low, const dFloat32* const high, const dInt32* const normalIndex) const;
	void SolveLcp(dInt32 stride, dInt32 size, const dFloat32* const matrix, const dFloat32* const x0, dFloat32* const x, const dFloat32* const b, const dFloat32* const low, const dFloat32* const high, const dInt32* const normalIndex) const;

	inline void SolveBackward(ndForcePair* const force) const;
	inline void CalculateForce(ndForcePair* const force, const ndForcePair* const accel) const;
	inline void UpdateForces(ndJacobian* const internalForces, const ndForcePair* const force) const;
	inline void CalculateJointAccel(const ndJacobian* const internalForces, ndForcePair* const accel) const;
	inline void SolveForward(ndForcePair* const force, const ndForcePair* const accel, dInt32 startNode) const;

	ndNode* m_skeleton;
	ndNode** m_nodesOrder;
	ndRightHandSide* m_rightHandSide;
	const ndLeftHandSide* m_leftHandSide;
	ndNodePair* m_pairs;
	dInt32* m_frictionIndex;
	dInt32* m_matrixRowsIndex;
	dFloat32* m_massMatrix11;
	dFloat32* m_massMatrix10;
	dFloat32* m_deltaForce;

	ndNodeList m_nodeList;
	ndArray<ndConstraint*> m_loopingJoints;
	ndArray<dInt8> m_auxiliaryMemoryBuffer;
	ndSpinLock m_lock;
	dInt32 m_blockSize;
	dInt16 m_rowCount;
	dInt16 m_loopRowCount;
	dInt16 m_auxiliaryRowCount;
	dInt16 m_loopCount;
	dInt16 m_dynamicsLoopCount;
	dInt16 m_consideredCloseLoop;
	bool m_isResting;
};

inline ndSkeletonContainer::ndNode* ndSkeletonContainer::GetRoot() const
{
	return m_skeleton;
}

#endif


