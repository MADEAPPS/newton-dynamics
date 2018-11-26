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

#include "dAnimationStdAfx.h"
#include "dAnimationRigLimb.h"
#include "dAnimationRigEffector.h"
#include "dAnimationCharacterRigManager.h"


dAnimationRigEffector::dAnimationRigEffector(dAnimationRigLimb* const parent, const dMatrix& matrix)
	:dAnimationKinematicLoopJoint()
	,m_targetMatrix(matrix)
	,m_linearSpeed(1.0f)
	,m_linearFriction(10000.0f)
{
	m_isActive = true;
	dAssert (!parent->m_effector);
	parent->m_effector = this;
	dAnimationCharacterRig* const root = parent->GetRoot();
	Init(parent->GetBody(), root->GetStaticWorld()->GetBody());
	SetOwners(parent, root->GetStaticWorld());

	NewtonBody* const newtonBody = parent->GetNewtonBody();
	dAssert (newtonBody);

	dMatrix parentMatrix;
	NewtonBodyGetMatrix(newtonBody, &parentMatrix[0][0]);
	m_pivotLocalMatrix = m_targetMatrix * parentMatrix.Inverse();
}

dAnimationRigEffector::~dAnimationRigEffector()
{
}

void dAnimationRigEffector::SetLinearSpeed(dFloat speed)
{
	m_linearSpeed = dAbs (speed);
}

void dAnimationRigEffector::SetMaxLinearFriction(dFloat friction)
{
	m_linearFriction = dAbs(friction);
}

void dAnimationRigEffector::JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
{
	const dMatrix& matrix1 = m_targetMatrix;
	dMatrix matrix0 (m_pivotLocalMatrix * m_state0->GetMatrix());
	
	dVector veloc0 (m_state0->CalculatePointVelocity(matrix0.m_posit));
	dVector veloc1 (m_state1->CalculatePointVelocity(matrix1.m_posit));

m_targetMatrix.m_posit.m_z = 0.45;
m_targetMatrix.m_posit.m_y = 0.7;

	dVector relVeloc(veloc1 - veloc0);
	dVector relPosit(m_targetMatrix.m_posit - matrix0.m_posit);

	const dFloat damp = 0.3f;
	const dFloat invTimestep = constraintParams->m_timestepInv;

	dVector omega (0.0f);
	for (int i = 0; i < 3; i++) {
		dFloat speed = relVeloc.DotProduct3(matrix1[i]);
		dFloat dist = relPosit.DotProduct3(matrix1[i]) * damp;
		dFloat relSpeed = dClamp(dist * invTimestep + speed, -m_linearSpeed, m_linearSpeed);
		dFloat relAccel = relSpeed * invTimestep;

		AddLinearRowJacobian(constraintParams, matrix0.m_posit, matrix1[i], omega);
		constraintParams->m_jointLowFrictionCoef[i] = -m_linearFriction;
		constraintParams->m_jointHighFrictionCoef[i] = m_linearFriction;
		constraintParams->m_jointAccel[i] = relAccel;
		constraintParams->m_normalIndex[i] = 0;
	}
	m_dof = 3;
	m_count = 3;
	constraintParams->m_count = 3;

m_dof = 1;
m_count = 1;
constraintParams->m_count = 1;
}
