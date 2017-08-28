/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_SKELETON_CONTAINER_H__
#define _DG_SKELETON_CONTAINER_H__

#define DG_SKELETON_BASE_UNIQUE_ID	10

#include "dgConstraint.h"

class dgDynamicBody;

class dgSkeletonContainer
{
	public:
	class dgNode;
	class dgNodePair;
	class dgForcePair;
	class dgMatriData;
	class dgBodyJointMatrixDataPair;
	class dgLoopingJoint
	{
		public: 
		dgLoopingJoint(dgBilateralConstraint* const joint, dgInt32 index0, dgInt32 index1)
			:m_joint(joint)
			,m_m0(dgInt16 (index0))
			,m_m1(dgInt16 (index1))
		{
		}

		dgBilateralConstraint* m_joint;
		dgInt16 m_m0;
		dgInt16 m_m1;
	};

	DG_CLASS_ALLOCATOR(allocator)
	dgSkeletonContainer(dgWorld* const world, dgDynamicBody* const rootBody);
	~dgSkeletonContainer();

	dgWorld* GetWorld() const; 
	dgInt32 GetId () const {return m_id;}
	dgInt32 GetJointCount () const {return m_nodeCount - 1;}
	dgNode* AddChild (dgBilateralConstraint* const joint, dgNode* const parent);
	void RemoveLoopJoint(dgBilateralConstraint* const joint);  
	void Finalize (dgInt32 loopJoints, dgBilateralConstraint** const loopJointArray);
	
	dgNode* GetRoot () const;
	dgDynamicBody* GetBody(dgNode* const node) const;
	dgBilateralConstraint* GetJoint(dgNode* const node) const;
	dgNode* GetParent (dgNode* const node) const;
	dgNode* GetFirstChild (dgNode* const parent) const;
	dgNode* GetNextSiblingChild (dgNode* const sibling) const;
	
	private:
	bool SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const;
	DG_INLINE void CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const;
	DG_INLINE void UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force) const;
	DG_INLINE void CalculateJointAccel (dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const;

	static void ResetUniqueId(dgInt32 id);

	dgNode* FindNode(dgDynamicBody* const node) const;
	void SortGraph(dgNode* const root, dgInt32& index);
		
	void InitMassMatrix (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgInt8* const memoryBuffer);
	void InitAuxiliaryMassMatrix (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgInt8* const memoryBuffer);
	dgInt32 GetMemoryBufferSizeInBytes (const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const;
	void SolveAuxiliary (const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force) const;

	void CalculateJointForce (dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow);
	
	DG_INLINE void CalculateBodyAccel (dgJacobian* const externalAccel, dgFloat32 timestep) const;
	DG_INLINE void CalculateJointAccel(const dgJacobian* const externalAccel, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const;
	void UpdateVelocity (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force, dgJacobian* const externalAccel, dgFloat32 timestep) const;
	void SolveAuxiliary (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force, dgJacobian* const externalAccel, dgFloat32 timestep) const;
	void UpdateForces (dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgFloat32 timestep);

	dgWorld* m_world;
	dgNode* m_skeleton;
	dgNode** m_nodesOrder;
	dgNodePair* m_pairs;
	dgFloat32* m_deltaForce;
	dgFloat32* m_massMatrix11;
	dgFloat32* m_massMatrix10;
	dgFloat32* m_lowerTriangularMassMatrix11;
	dgJacobianMatrixElement** m_rowArray;
	dgList<dgDynamicBody*> m_loopingBodies;
	dgList<dgLoopingJoint> m_loopingJoints;
	dgInt32 m_id;
	dgInt32 m_lru;
	dgInt16 m_nodeCount;
	dgInt16 m_rowCount;
	dgInt16 m_auxiliaryRowCount;
	static dgInt32 m_uniqueID;
	static dgInt32 m_lruMarker;

	friend class dgWorld;
	friend class dgWorldDynamicUpdate;
};

#endif

