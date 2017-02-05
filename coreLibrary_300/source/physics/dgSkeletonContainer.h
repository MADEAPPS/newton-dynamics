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
class dgSkeletonContainer;
typedef void (dgApi *dgOnSkeletonContainerDestroyCallback) (dgSkeletonContainer* const me);

class dgSkeletonContainer
{
	public:
	class dgGraph;
	class dgNodePair;
	class dgForcePair;
	class dgMatriData;
	class dgBodyJointMatrixDataPair;

	DG_CLASS_ALLOCATOR(allocator)
	dgSkeletonContainer(dgWorld* const world, dgDynamicBody* const rootBody);
	~dgSkeletonContainer();

	void Finalize ();
	dgWorld* GetWorld() const; 
	dgInt32 GetId () const {return m_id;}
	dgInt32 GetJointCount () const {return m_nodeCount - 1;}
	dgGraph* AddChild (dgBilateralConstraint* const joint, dgGraph* const parent);
	void SetDestructorCallback (dgOnSkeletonContainerDestroyCallback destructor);
	bool AttachCyclingJoint(dgBilateralConstraint* const jointPtr); 
	void RemoveCyclingJoint(dgBilateralConstraint* const jointPtr);  
	
	dgGraph* GetRoot () const;
	dgDynamicBody* GetBody(dgGraph* const node) const;
	dgBilateralConstraint* GetParentJoint(dgGraph* const node) const;
	dgGraph* GetParent (dgGraph* const node) const;
	dgGraph* GetFirstChild (dgGraph* const parent) const;
	dgGraph* GetNextSiblingChild (dgGraph* const sibling) const;
	void SetGrapfDepth(dgInt32 depth);
	
	private:
	bool SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const;
	DG_INLINE void CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const;
	DG_INLINE void UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force) const;
	DG_INLINE void CalculateJointAccel (dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const force) const;
		
	void InitMassMatrix (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgInt8* const memoryBuffer);
	dgInt32 GetMemoryBufferSizeInBytes (const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const;
	void SolveAuxiliary (const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force) const;
	void BruteForceSolve (const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force) const;
	void CalculateJointForce (dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow);
	
	dgGraph* FindNode (dgDynamicBody* const node) const;
	void SortGraph (dgGraph* const root, dgGraph* const parent, dgInt32& index);
	
	static void ResetUniqueId(dgInt32 id);

	dgWorld* m_world;
	dgGraph* m_skeleton;
	dgGraph** m_nodesOrder;
	dgNodePair* m_pairs;
	dgFloat32* m_deltaForce;
	dgFloat32* m_massMatrix11;
	dgFloat32* m_massMatrix10;
	dgFloat32* m_factoredMassMatrix11;
	dgJacobianMatrixElement** m_rowArray;
	dgOnSkeletonContainerDestroyCallback m_destructor;
	dgList<dgConstraint*> m_cyclingJoints;
	dgInt32 m_id;
	dgInt32 m_lru;
	mutable dgInt32 m_bufferSize;
	dgInt16 m_nodeCount;
	dgInt16 m_rowCount;
	dgInt16 m_auxiliaryRowCount;
	static dgInt32 m_uniqueID;
	static dgInt32 m_lruMarker;

	friend class dgWorld;
	friend class dgWorldDynamicUpdate;
};

#endif

