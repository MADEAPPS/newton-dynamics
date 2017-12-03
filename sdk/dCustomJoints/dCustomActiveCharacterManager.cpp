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

//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomRagdollMotor.h"
#include "dCustomActiveCharacterManager.h"


dCustomActiveCharacterManager::dCustomActiveCharacterManager(NewtonWorld* const world, const char* const name)
	:dCustomControllerManager<dCustomActiveCharacterController>(world, name)
{
}

dCustomActiveCharacterManager::~dCustomActiveCharacterManager()
{
}

dCustomActiveCharacterController* dCustomActiveCharacterManager::CreateTransformController()
{
	dCustomActiveCharacterController* const controller = (dCustomActiveCharacterController*)CreateController();
	return controller;
}


/*
void dCustomActiveCharacterManager::OnUpdateTransform(const dCustomActiveCharacterController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	//	dAssert (0);
}
*/





dCustomActiveCharacterController::dCustomActiveCharacterController()
	:m_kinematicSolver(NULL)
{
}

dCustomActiveCharacterController::~dCustomActiveCharacterController()
{
	if (m_kinematicSolver) {
		NewtonInverseDynamicsDestroy (m_kinematicSolver);
	}
}



/*
void dCustomActiveCharacterManager::OnUpdateTransform(const dCustomActiveCharacterController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
{
	//	dAssert (0);
}
*/




void dCustomActiveCharacterController::PostUpdate(dFloat timestep, int threadIndex)
{
//	dAssert (0);
/*
	if (m_calculateLocalTransform) {
		dCustomArticulaledTransformManager* const manager = (dCustomArticulaledTransformManager*)GetManager();
		for (int i = 0; i < m_boneCount; i++) {
			const dSkeletonBone& bone = m_bones[i];
			dMatrix matrix;
			NewtonBodyGetMatrix(bone.m_body, &matrix[0][0]);
			if (bone.m_parent) {
				dMatrix parentMatrix;
				NewtonBodyGetMatrix(bone.m_parent->m_body, &parentMatrix[0][0]);
				matrix = matrix * parentMatrix.Inverse() * bone.m_bindMatrix;
			}
			manager->OnUpdateTransform(&bone, matrix);
		}
	}
*/
}

void* dCustomActiveCharacterController::GetRoot() const
{
	return m_kinematicSolver ? NewtonInverseDynamicsGetRoot (m_kinematicSolver) : NULL;
}

NewtonBody* dCustomActiveCharacterController::GetBody(void* const node) const
{
	return m_kinematicSolver ? NewtonInverseDynamicsGetBody (m_kinematicSolver, node) : NULL;
}

dCustomJoint* const dCustomActiveCharacterController::GetJoint(void* const node) const
{
	NewtonJoint* const joint = m_kinematicSolver ? NewtonInverseDynamicsGetJoint (m_kinematicSolver, node) : NULL;
	//dAssert(!joint || ((dCustomRagdollMotor*)NewtonJointGetUserData(joint))->IsType(dCustomRagdollMotor::GetType()));
	return joint ? (dCustomJoint*) NewtonJointGetUserData(joint) : NULL;
}

void* dCustomActiveCharacterController::AddRoot(NewtonBody* const rootBody)
{
	dAssert (!m_kinematicSolver);
	if (m_kinematicSolver) {
		NewtonInverseDynamicsDestroy (m_kinematicSolver);
	}

	dCustomActiveCharacterManager* const manager = (dCustomActiveCharacterManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	m_body = rootBody;
	m_kinematicSolver = NewtonCreateInverseDynamics(world);
	return NewtonInverseDynamicsAddRoot(m_kinematicSolver, m_body);
}

void* dCustomActiveCharacterController::AddBone(dCustomRagdollMotor* const childJoint, void* const parentBone)
{
	dAssert (m_kinematicSolver);
	return NewtonInverseDynamicsAddChildNode (m_kinematicSolver, parentBone, childJoint->GetJoint());
}

void dCustomActiveCharacterController::Finalize ()
{
	if (m_kinematicSolver) {
		NewtonInverseDynamicsEndBuild (m_kinematicSolver);
	}
}

dCustomRagdollMotor_EndEffector* dCustomActiveCharacterController::AddEndEffector(void* const node, const dMatrix& pinAndPivot)
{
	dAssert (m_kinematicSolver);
	dCustomRagdollMotor_EndEffector* const effector = new dCustomRagdollMotor_EndEffector(m_kinematicSolver, node, pinAndPivot);
	m_effectorList.Append(effector);
	return effector;
}


void dCustomActiveCharacterController::Debug(dCustomJoint::dDebugDisplay* const debugContext) const
{
	for (dList<dCustomRagdollMotor_EndEffector*>::dListNode* node = m_effectorList.GetFirst(); node; node = node->GetNext()) {
		dCustomJoint* const joint = node->GetInfo();
		joint->Debug(debugContext);
	}
}


void dCustomActiveCharacterController::PreUpdate(dFloat timestep, int threadIndex)
{
	//	dCustomActiveCharacterManager* const manager = (dCustomActiveCharacterManager*)GetManager();
	//	manager->OnPreUpdate(this, timestep, threadIndex);

	if (m_kinematicSolver) {
		NewtonInverseDynamicsUpdate (m_kinematicSolver, timestep, threadIndex);
	}
}

