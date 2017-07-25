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
	:m_kinemativSolver(NULL)
{
}

dCustomActiveCharacterController::~dCustomActiveCharacterController()
{
	if (m_kinemativSolver) {
		NewtonInverseDynamicsDestroy (m_kinemativSolver);
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
	return m_kinemativSolver ? NewtonInverseDynamicsGetRoot (m_kinemativSolver) : NULL;
}

NewtonBody* dCustomActiveCharacterController::GetBody(void* const node) const
{
	return m_kinemativSolver ? NewtonInverseDynamicsGetBody (m_kinemativSolver, node) : NULL;
}

dCustomJoint* const dCustomActiveCharacterController::childJoint(void* const node) const
{
	NewtonJoint* const joint = m_kinemativSolver ? NewtonInverseDynamicsGetJoint (m_kinemativSolver, node) : NULL;
	return joint ? (dCustomJoint*) NewtonJointGetUserData(joint) : NULL;
}

void* dCustomActiveCharacterController::AddRoot(NewtonBody* const rootBody)
{
	dAssert (!m_kinemativSolver);
	if (m_kinemativSolver) {
		NewtonInverseDynamicsDestroy (m_kinemativSolver);
	}

	dCustomActiveCharacterManager* const manager = (dCustomActiveCharacterManager*)GetManager();
	NewtonWorld* const world = manager->GetWorld();

	m_body = rootBody;
	m_kinemativSolver = NewtonCreateInverseDynamics(world);
	return NewtonInverseDynamicsAddRoot(m_kinemativSolver, m_body);
}

void* dCustomActiveCharacterController::AddBone(dCustomJoint* const childJoint, void* const parentBone)
{
	dAssert (m_kinemativSolver);
	return NewtonInverseDynamicsAddChildNode (m_kinemativSolver, parentBone, childJoint->GetJoint());
}

void dCustomActiveCharacterController::Finalize ()
{
	if (m_kinemativSolver) {
		NewtonInverseDynamicsEndBuild (m_kinemativSolver);
	}
}

void dCustomActiveCharacterController::PreUpdate(dFloat timestep, int threadIndex)
{
	//	dCustomActiveCharacterManager* const manager = (dCustomActiveCharacterManager*)GetManager();
	//	manager->OnPreUpdate(this, timestep, threadIndex);

	if (m_kinemativSolver) {
		NewtonInverseDynamicsUpdate (m_kinemativSolver, timestep, threadIndex);
	}
}

void dCustomActiveCharacterManager::Debug() const
{
//	dAssert(0);
}

