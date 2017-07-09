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
#include <dCustomJointLibraryStdAfx.h>
#include <dCustomActiveCharacterManager.h>

IMPLEMENT_CUSTOM_JOINT(dCustomActiveCharacterJoint);


dCustomActiveCharacterJoint::dCustomActiveCharacterJoint(const dMatrix& childPinAndPivotFrame, NewtonBody* const child, const dMatrix& parentPinAndPivotFrame, NewtonBody* const parent)
	:dCustomBallAndSocket(childPinAndPivotFrame, child, parent)
	,m_jacobian(this)
	,m_rotationOffset(childPinAndPivotFrame * parentPinAndPivotFrame.Inverse())
{
	SetConeAngle(0.0f);
	SetTwistAngle(0.0f, 0.0f);
	dMatrix matrix;
	CalculateLocalMatrix(parentPinAndPivotFrame, matrix, m_localMatrix1);
}


dCustomActiveCharacterJoint::~dCustomActiveCharacterJoint()
{
}

dInverseKinematicJacobian* dCustomActiveCharacterJoint::GetJacobianCalculator()
{
	return &m_jacobian;
}


dCustomActiveCharacterJoint::dCustomActiveCharacterJoint(NewtonBody* const child, NewtonBody* const parent, NewtonDeserializeCallback callback, void* const userData)
	:dCustomBallAndSocket(child, parent, callback, userData)
	,m_jacobian(this)
{
	dAssert (0);
	callback(userData, &m_rotationOffset, sizeof (dMatrix));
	callback(userData, &m_minTwistAngle, sizeof (dFloat));
	callback(userData, &m_maxTwistAngle, sizeof (dFloat));
	callback(userData, &m_coneAngleCos, sizeof (dFloat));
	callback(userData, &m_coneAngleSin, sizeof (dFloat));
	callback(userData, &m_coneAngleHalfCos, sizeof (dFloat));
	callback(userData, &m_coneAngleHalfSin, sizeof (dFloat));
}

void dCustomActiveCharacterJoint::Serialize(NewtonSerializeCallback callback, void* const userData) const
{
	dCustomBallAndSocket::Serialize(callback, userData);

	callback(userData, &m_rotationOffset, sizeof (dMatrix));
	callback(userData, &m_minTwistAngle, sizeof (dFloat));
	callback(userData, &m_maxTwistAngle, sizeof (dFloat));
	callback(userData, &m_coneAngleCos, sizeof (dFloat));
	callback(userData, &m_coneAngleSin, sizeof (dFloat));
	callback(userData, &m_coneAngleHalfCos, sizeof (dFloat));
	callback(userData, &m_coneAngleHalfSin, sizeof (dFloat));
}


void dCustomActiveCharacterJoint::SetConeAngle(dFloat angle)
{
	m_coneAngle = angle;
	m_coneAngleCos = dCos(angle);
	m_coneAngleSin = dSin(angle);
	m_coneAngleHalfCos = dCos(angle * 0.5f);
	m_coneAngleHalfSin = dSin(angle * 0.5f);
}


void dCustomActiveCharacterJoint::SetTwistAngle(dFloat minAngle, dFloat maxAngle)
{
	m_minTwistAngle = minAngle;
	m_maxTwistAngle = maxAngle;
}

dFloat dCustomActiveCharacterJoint::GetConeAngle() const
{
	return m_coneAngle;
}

void dCustomActiveCharacterJoint::GetTwistAngle(dFloat& minAngle, dFloat& maxAngle) const
{
	minAngle = m_minTwistAngle;
	maxAngle = m_maxTwistAngle;
}


void dCustomActiveCharacterJoint::GetInfo(NewtonJointRecord* const info) const
{
	dCustomBallAndSocket::GetInfo(info);

	info->m_minAngularDof[0] = m_minTwistAngle;
	info->m_maxAngularDof[0] = m_maxTwistAngle;
	//	info->m_minAngularDof[1] = -dAcos (m_coneAngleCos);
	//	info->m_maxAngularDof[1] =  dAcos (m_coneAngleCos);
	//	info->m_minAngularDof[2] = -dAcos (m_coneAngleCos); 
	//	info->m_maxAngularDof[2] =  dAcos (m_coneAngleCos);

	info->m_minAngularDof[1] = -m_coneAngle;
	info->m_maxAngularDof[1] = m_coneAngle;
	info->m_minAngularDof[2] = -m_coneAngle;
	info->m_maxAngularDof[2] = m_coneAngle;

	strcpy(info->m_descriptionType, GetTypeName());
}

void dCustomActiveCharacterJoint::SubmitConstraints(dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

	matrix1 = m_rotationOffset * matrix1;

	// handle special case of the joint being a hinge
	if (m_coneAngleCos > 0.9999f) {
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
		NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);

		// the joint angle can be determined by getting the angle between any two non parallel vectors
		dFloat pitchAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
		if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix1.m_front[0]);
		} else {
			if (pitchAngle > m_maxTwistAngle) {
				pitchAngle -= m_maxTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, -0.0f);
			} else if (pitchAngle < m_minTwistAngle) {
				pitchAngle -= m_minTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}

	} else {

		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;
		dFloat cosAngle = coneDir0.DotProduct3(coneDir1);
		if (cosAngle <= m_coneAngleCos) {
			dVector lateralDir(coneDir0.CrossProduct(coneDir1));
			dFloat mag2 = lateralDir.DotProduct3(lateralDir);
			dAssert(mag2 > 1.0e-4f);
			lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));

			dQuaternion rot(m_coneAngleHalfCos, lateralDir.m_x * m_coneAngleHalfSin, lateralDir.m_y * m_coneAngleHalfSin, lateralDir.m_z * m_coneAngleHalfSin);
			dVector frontDir(rot.UnrotateVector(coneDir1));
			dVector upDir(lateralDir.CrossProduct(frontDir));
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &upDir[0]);
			NewtonUserJointAddAngularRow(m_joint, CalculateAngle(coneDir0, frontDir, lateralDir), &lateralDir[0]);
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
		}

		//handle twist angle
		dFloat pitchAngle = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
		if ((m_maxTwistAngle - m_minTwistAngle) < 1.0e-4f) {
			NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix1.m_front[0]);
		} else {
			if (pitchAngle > m_maxTwistAngle) {
				pitchAngle -= m_maxTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, -0.0f);
			} else if (pitchAngle < m_minTwistAngle) {
				pitchAngle -= m_minTwistAngle;
				NewtonUserJointAddAngularRow(m_joint, pitchAngle, &matrix0.m_front[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			}
		}
	}
}



dCustomActiveCharacterManager::dCustomActiveCharacterManager(NewtonWorld* const world, const char* const name)
	:dCustomControllerManager<dCustomActiveCharacterController>(world, name)
{
}

dCustomActiveCharacterManager::~dCustomActiveCharacterManager()
{
}


void dCustomActiveCharacterManager::Debug() const
{
	dAssert(0);
}




dCustomActiveCharacterController* dCustomActiveCharacterManager::CreateTransformController(NewtonBody* const rootBone)
{
	dCustomActiveCharacterController* const controller = (dCustomActiveCharacterController*)CreateController();
	controller->Init(rootBone);
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
		delete m_kinemativSolver;
	}
}


void dCustomActiveCharacterController::Init(NewtonBody* const rootBone)
{
	//dCustomActiveCharacterManager* const manager = (dCustomActiveCharacterManager*)GetManager();
	//NewtonWorld* const world = manager->GetWorld();

	m_body = rootBone;
	m_kinemativSolver = new dInverseKinematicSolver (m_body);
/*
	m_userData = userData;
	m_boneCount = 0;
	m_calculateLocalTransform = false;
	m_collisionAggregate = NewtonCollisionAggregateCreate(world);
*/
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



dInverseKinematicSolver::dJoint* dCustomActiveCharacterController::GetRoot() const
{
	return m_kinemativSolver ? m_kinemativSolver->GetRoot() : NULL;
}

dInverseKinematicSolver::dJoint* dCustomActiveCharacterController::AddBone(dInverseKinematicSolver::dJacobian* const jacobian, dInverseKinematicSolver::dJoint* const parentBone)
{
	dAssert (m_kinemativSolver);
	return m_kinemativSolver->AddChild(jacobian, parentBone);
}

void dCustomActiveCharacterController::Finalize ()
{
	if (m_kinemativSolver) {
		m_kinemativSolver->Finalize(0);
	}
}

void dCustomActiveCharacterController::PreUpdate(dFloat timestep, int threadIndex)
{
	//	dCustomActiveCharacterManager* const manager = (dCustomActiveCharacterManager*)GetManager();
	//	manager->OnPreUpdate(this, timestep, threadIndex);

	if (m_kinemativSolver) {
		m_kinemativSolver->UpdateJointAngles (timestep);
	}
}
