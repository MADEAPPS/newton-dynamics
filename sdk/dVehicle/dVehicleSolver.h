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


#ifndef __D_VEHICLE_SOLVER_H__
#define __D_VEHICLE_SOLVER_H__

#include "dStdafxVehicle.h"


class dVehicleNode;
class dVehicleChassis;
class dKinematicLoopJoint;

class dVehicleSolver: public dContainersAlloc
{
	class dVectorPair;
	class dBodyJointMatrixDataPair;

	public:
	DVEHICLE_API dVehicleSolver();
	DVEHICLE_API virtual ~dVehicleSolver();
	DVEHICLE_API void Finalize(dVehicleChassis* const vehicle);

	void Update(dFloat timestep);

	private:
	int CalculateNodeCount () const;
	void SortGraph(dVehicleNode* const root, int& index);

	void InitMassMatrix();
	void Factorize(dVehicleNode* const node);
	void GetJacobians(dVehicleNode* const node);
	int BuildJacobianMatrix(dFloat timestep);
	int BuildJacobianMatrix(dFloat timestep, dComplementaritySolver::dBilateralJoint* const joint);

	void CalculateJointDiagonal(dVehicleNode* const node);
	void CalculateJacobianBlock(dVehicleNode* const node);
	void CalculateBodyDiagonal(dVehicleNode* const child);
	void CalculateInertiaMatrix(dVehicleNode* const node) const;

	void CalculateJointForce();
	void UpdateForces(const dVectorPair* const force) const;
	void CalculateJointAccel(dVectorPair* const accel) const;
	void CalculateForce(dVectorPair* const force, const dVectorPair* const accel) const;

	void SolveBackward(dVectorPair* const force, const dVectorPair* const accel) const;
	void SolveForward(dVectorPair* const force, const dVectorPair* const accel, int startNode) const;

	void BodyDiagInvTimeSolution(dVehicleNode* const node, dVectorPair& force) const;
	void JointDiagInvTimeSolution(dVehicleNode* const node, dVectorPair& force) const;
	void JointJacobianTimeMassForward(dVehicleNode* const node, dVectorPair& force) const;
	void BodyJacobianTimeSolutionBackward(dVehicleNode* const node, dVectorPair& force) const;
	void BodyJacobianTimeMassForward(dVehicleNode* const node, const dVectorPair& force, dVectorPair& parentForce) const;
	void JointJacobianTimeSolutionBackward(dVehicleNode* const node, dVectorPair& force, const dVectorPair& parentForce) const;

	dVehicleChassis* m_vehicle;
	dVehicleNode** m_nodesOrder;



	// cache temporary variables
	dBodyJointMatrixDataPair* m_data;
	dKinematicLoopJoint** m_kinematicLoop;	
	dComplementaritySolver::dJacobianPair* m_leftHandSide;
	dComplementaritySolver::dJacobianColum* m_rightHandSide;
	
	int m_rowCount;
	int m_nodeCount;
	
	int m_kinematicLoopCount;
	int m_auxiliaryRowCount;
};


#endif 

