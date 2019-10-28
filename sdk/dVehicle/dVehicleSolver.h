/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
class dVehicleMultiBody;
class dVehicleLoopJoint;

class dVehicleSolver
{
	class dNodePair;
	class dVectorPair;
	class dMatrixData;
	class dBodyJointMatrixDataPair;

	public:
	dVehicleSolver();
	virtual ~dVehicleSolver();
	void Finalize();

	void Update(dFloat timestep);

	private:
	int CalculateNodeCount () const;
	void SortGraph(dVehicleNode* const root, int& index);

	void InitMassMatrix();
	void InitLoopMassMatrix();
	void Factorize(dVehicleNode* const node);
	int BuildJacobianMatrix(dFloat timestep);
	void GetJacobians(dVehicleNode* const node);
	void CalculateLoopMassMatrixCoefficients();
	int BuildJacobianMatrix(dFloat timestep, dComplementaritySolver::dBilateralJoint* const joint);

	void CalculateJointDiagonal(dVehicleNode* const node);
	void CalculateJacobianBlock(dVehicleNode* const node);
	void CalculateBodyDiagonal(dVehicleNode* const child);
	void CalculateInertiaMatrix(dVehicleNode* const node) const;

	void UpdateForces(const dVectorPair* const force) const;
	void CalculateJointAccel(dVectorPair* const accel) const;
	void SolveAuxiliary(dVectorPair* const force, const dVectorPair* const accel) const;
	void CalculateOpenLoopForce(dVectorPair* const force, const dVectorPair* const accel) const;

	void SolveBackward(dVectorPair* const force, const dVectorPair* const accel) const;
	void SolveForward(dVectorPair* const force, const dVectorPair* const accel, int startNode) const;

	void BodyDiagInvTimeSolution(dVehicleNode* const node, dVectorPair& force) const;
	void JointDiagInvTimeSolution(dVehicleNode* const node, dVectorPair& force) const;
	void JointJacobianTimeMassForward(dVehicleNode* const node, dVectorPair& force) const;
	void BodyJacobianTimeSolutionBackward(dVehicleNode* const node, dVectorPair& force) const;
	void BodyJacobianTimeMassForward(dVehicleNode* const node, const dVectorPair& force, dVectorPair& parentForce) const;
	void JointJacobianTimeSolutionBackward(dVehicleNode* const node, dVectorPair& force, const dVectorPair& parentForce) const;
	void dGaussSeidelLcpSor(const int size, dFloat* const x, const dFloat* const b, const int* const normalIndex, 
							dFloat* const low, dFloat* const high, dComplementaritySolver::dBilateralJoint** const frictionCallback) const;

//	dVehicleNode** m_nodesOrder;
	dArray<dVehicleNode*> m_nodesOrder;
	int* m_matrixRowsIndex;
	dNodePair* m_pairs;
	dFloat* m_deltaForce;
	dFloat* m_massMatrix10;
	dFloat* m_massMatrix11;
	dBodyJointMatrixDataPair* m_data;
	dVehicleLoopJoint** m_loopJoints;
	dComplementaritySolver::dJacobianPair* m_leftHandSide;
	dComplementaritySolver::dJacobianColum* m_rightHandSide;
	
	int m_rowCount;
	int m_nodeCount;
	int m_maxNodeCount;
	int m_loopRowCount;
	int m_loopJointCount;
	int m_auxiliaryRowCount;

	friend class dVehicleMultiBody;
};

#endif 

