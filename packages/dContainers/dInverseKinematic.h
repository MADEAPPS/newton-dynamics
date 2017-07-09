/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _D_INVERSE_KINEMATICS_H__
#define _D_INVERSE_KINEMATICS_H__



#include "dContainersAlloc.h"


//class dDynamicBody;
//class dInverseKinematicSolver;

class dCustomJoint;

class dInverseKinematicSolver: public dContainersAlloc
{
/*
	class dNodePair;
	class dForcePair;
	class dMatriData;
	class dBodyJointMatrixDataPair;
	class dCyclingJoint
	{
		public: 
		dgCyclingJoint(dgBilateralConstraint* const joint, int index0, int index1)
			:m_joint(joint)
			,m_m0(dgInt16 (index0))
			,m_m1(dgInt16 (index1))
		{
		}

		dgBilateralConstraint* m_joint;
		dgInt16 m_m0;
		dgInt16 m_m1;
	};



	dgWorld* GetWorld() const; 
	int GetId () const {return m_id;}
	int GetJointCount () const {return m_nodeCount - 1;}
	dgGraph* AddChild (dgBilateralConstraint* const joint, dgGraph* const parent);
	void RemoveCyclingJoint(dgBilateralConstraint* const joint);  
	
	

	dgDynamicBody* GetBody(dgGraph* const node) const;
	dgBilateralConstraint* GetParentJoint(dgGraph* const node) const;
	dgGraph* GetParent (dgGraph* const node) const;
	dgGraph* GetFirstChild (dgGraph* const parent) const;
	dgGraph* GetNextSiblingChild (dgGraph* const sibling) const;
	
	private:
	bool SanityCheck(const dgForcePair* const force, const dgForcePair* const accel) const;
	inline void CalculateForce (dgForcePair* const force, const dgForcePair* const accel) const;
	inline void UpdateForces (dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force) const;
	inline void CalculateJointAccel (dgJointInfo* const jointInfoArray, const dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const;

	static void ResetUniqueId(int id);

	dgGraph* FindNode(dgDynamicBody* const node) const;
	int GetMemoryBufferSizeInBytes (const dgJointInfo* const jointInfoArray, const dgJacobianMatrixElement* const matrixRow) const;
	void SolveAuxiliary (const dgJointInfo* const jointInfoArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force) const;
	void CalculateJointForce (dgJointInfo* const jointInfoArray, const dgBodyInfo* const bodyArray, dgJacobian* const internalForces, dgJacobianMatrixElement* const matrixRow);
	
	inline void CalculateBodyAcceleration (dgJacobian* const externalAccel, dgFloat32 timestep) const;
	inline void CalculateJointAccel(const dgJacobian* const externalAccel, dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, dgForcePair* const accel) const;
	void UpdateVelocity (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const force, dgJacobian* const externalAccel, dgFloat32 timestep) const;
	void SolveAuxiliaryVelocity (const dgJointInfo* const jointInfoArray, dgJacobianMatrixElement* const matrixRow, const dgForcePair* const accel, dgForcePair* const force, dgJacobian* const externalAccel, dgFloat32 timestep) const;


	dgWorld* m_world;
	
	dgNodePair* m_pairs;
	dgFloat32* m_deltaForce;
	dgFloat32* m_massMatrix11;
	dgFloat32* m_massMatrix10;
	dgFloat32* m_lowerTriangularMassMatrix11;
	dgJacobianMatrixElement** m_rowArray;
	dgList<dgDynamicBody*> m_cyclingBodies;
	dgList<dgCyclingJoint> m_cyclingJoints;
	int m_id;
	int m_lru;
	
	dgInt16 m_rowCount;
	dgInt16 m_auxiliaryRowCount;
*/

	public: 
	class dJoint;

	class dJacobian: public dContainersAlloc
	{
		public:
		dJacobian(dCustomJoint* const joint)
			:dContainersAlloc()
			,m_customJoint(joint)
		{
		}

		~dJacobian()
		{
		}

		virtual void CalculateJacobians() = 0;
		virtual NewtonBody* GetBody0() const = 0;
		virtual NewtonBody* GetBody1() const = 0;
		virtual dFloat GetBodyMass(const NewtonBody* const body) const = 0;
		virtual dFloat GetBodyInvMass(const NewtonBody* const body) const = 0;

		dCustomJoint* m_customJoint;
	};

	class dJoint: public dContainersAlloc
	{
		public:
		DCONTAINERS_API dJoint(NewtonBody* const body);
		DCONTAINERS_API ~dJoint();

		private:
		dJoint(dJacobian* const jacobian, dJoint* const parent);

		NewtonBody* m_body;
		dJoint* m_parent;
		dJoint* m_child;
		dJoint* m_sibling;
		dJacobian* m_jacobian;

		short m_index;

		friend class dInverseKinematicSolver;  
	};


	DCONTAINERS_API dInverseKinematicSolver(NewtonBody* const rootBody);
	DCONTAINERS_API ~dInverseKinematicSolver();
	DCONTAINERS_API dJoint* GetRoot () const;
	DCONTAINERS_API dJoint* AddChild (dJacobian* const jacobian, dJoint* const parentBone);

	//void Finalize (int loopJoints, dgBilateralConstraint** const loopJointArray);
	void Finalize (int loopJoints);

	DCONTAINERS_API void UpdateJointAngles (dFloat timestep);

	private:
	void InitMassMatrix ();
	void SortGraph(dJoint* const root, int& index);

	protected:
	dJoint* m_skeleton;

	dJoint** m_nodesOrder;

	
	short m_nodeCount;
};

#endif

