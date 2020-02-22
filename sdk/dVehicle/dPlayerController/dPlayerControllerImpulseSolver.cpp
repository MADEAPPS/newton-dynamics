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


// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////

#include "dStdafxVehicle.h"

#include "dVehicleManager.h"
#include "dPlayerController.h"
#include "dPlayerControllerImpulseSolver.h"

dPlayerControllerImpulseSolver::dPlayerControllerImpulseSolver(dPlayerController* const controller)
	:m_zero(0.0f)
{
	m_mass = controller->m_mass;
	m_invMass = controller->m_invMass;
	NewtonBodyGetInvInertiaMatrix(controller->m_newtonBody, &m_invInertia[0][0]);
	Reset(controller);
}

void dPlayerControllerImpulseSolver::Reset(dPlayerController* const controller)
{
	m_rowCount = 0;
	NewtonBodyGetVelocity(controller->m_newtonBody, &m_veloc[0]);
}

void dPlayerControllerImpulseSolver::AddAngularRows()
{
	for (int i = 0; i < 3; i++) {
		m_contactPoint[m_rowCount] = NULL;
		m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear = m_zero;
		m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular = m_zero;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear = m_zero;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular = m_zero;
		m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular[i] = dFloat(1.0f);
		m_rhs[m_rowCount] = 0.0f;
		m_low[m_rowCount] = -1.0e12f;
		m_high[m_rowCount] = 1.0e12f;
		m_impulseMag[m_rowCount] = 0.0f;
		m_normalIndex[m_rowCount] = 0;
		m_rowCount++;
		dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	}
}

int dPlayerControllerImpulseSolver::AddLinearRow(const dVector& dir, const dVector& r, dFloat speed, dFloat low, dFloat high, int normalIndex)
{
	m_contactPoint[m_rowCount] = NULL;
	m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear = dir;
	m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular = r.CrossProduct(dir);
	m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear = m_zero;
	m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular = m_zero;

	m_low[m_rowCount] = low;
	m_high[m_rowCount] = high;
	m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;
	m_rhs[m_rowCount] = speed - m_veloc.DotProduct3(m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear);
	m_rowCount++;
	dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	return m_rowCount - 1;
}

int dPlayerControllerImpulseSolver::AddContactRow(const NewtonWorldConvexCastReturnInfo* const contact, const dVector& dir, const dVector& r, dFloat speed, dFloat low, dFloat high, int normalIndex)
{
	dFloat invIxx;
	dFloat invIyy;
	dFloat invIzz;
	dFloat invMass;
	dAssert(contact->m_hitBody);
	NewtonBodyGetInvMass(contact->m_hitBody, &invMass, &invIxx, &invIyy, &invIzz);
	if (invMass == 0.0f) {
		return AddLinearRow(dir, r, speed, low, high, normalIndex);
	}

	dMatrix matrix;
	dVector com;
	dVector veloc;
	dVector omega;

	NewtonBodyGetOmega(contact->m_hitBody, &omega[0]);
	NewtonBodyGetVelocity(contact->m_hitBody, &veloc[0]);
	NewtonBodyGetMatrix(contact->m_hitBody, &matrix[0][0]);
	NewtonBodyGetCentreOfMass(contact->m_hitBody, &com[0]);
	com = matrix.TransformVector(com);

	dVector p1(contact->m_point[0], contact->m_point[1], contact->m_point[2], dFloat(0.0f));
	dVector r1(p1 - com);
	dVector dir1(dir.Scale(-1.0f));

	m_contactPoint[m_rowCount] = contact;
	m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear = dir;
	m_jacobianPairs[m_rowCount].m_jacobian_J01.m_angular = r.CrossProduct(dir);
	m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear = dir1;
	m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular = r1.CrossProduct(dir1);

	m_low[m_rowCount] = low;
	m_high[m_rowCount] = high;
	m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;

	dVector s(m_veloc * m_jacobianPairs[m_rowCount].m_jacobian_J01.m_linear +
		veloc * m_jacobianPairs[m_rowCount].m_jacobian_J10.m_linear +
		omega * m_jacobianPairs[m_rowCount].m_jacobian_J10.m_angular);
	m_rhs[m_rowCount] = speed - s.m_x - s.m_y - s.m_z;

	m_rowCount++;
	dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	return m_rowCount - 1;
}

dVector dPlayerControllerImpulseSolver::CalculateImpulse()
{
	dFloat massMatrix[D_PLAYER_MAX_ROWS][D_PLAYER_MAX_ROWS];
	const NewtonBody* bodyArray[D_PLAYER_MAX_ROWS];
	for (int i = 0; i < m_rowCount; i++) {
		bodyArray[i] = m_contactPoint[i] ? m_contactPoint[i]->m_hitBody : NULL;
	}

	for (int i = 0; i < m_rowCount; i++) {
		dComplementaritySolver::dJacobianPair jInvMass(m_jacobianPairs[i]);

		jInvMass.m_jacobian_J01.m_linear = jInvMass.m_jacobian_J01.m_linear.Scale(m_invMass);
		jInvMass.m_jacobian_J01.m_angular = m_invInertia.RotateVector(jInvMass.m_jacobian_J01.m_angular);
		if (bodyArray[i]) {
			dMatrix invInertia;
			dFloat invIxx;
			dFloat invIyy;
			dFloat invIzz;
			dFloat invMass;

			NewtonBodyGetInvMass(bodyArray[i], &invMass, &invIxx, &invIyy, &invIzz);
			NewtonBodyGetInvInertiaMatrix(bodyArray[i], &invInertia[0][0]);
			jInvMass.m_jacobian_J10.m_linear = jInvMass.m_jacobian_J10.m_linear.Scale(invMass);
			jInvMass.m_jacobian_J10.m_angular = invInertia.RotateVector(jInvMass.m_jacobian_J10.m_angular);

		} else {
			jInvMass.m_jacobian_J10.m_linear = m_zero;
			jInvMass.m_jacobian_J10.m_angular = m_zero;
		}

		dVector tmp(jInvMass.m_jacobian_J01.m_linear * m_jacobianPairs[i].m_jacobian_J01.m_linear +
			jInvMass.m_jacobian_J01.m_angular * m_jacobianPairs[i].m_jacobian_J01.m_angular +
			jInvMass.m_jacobian_J10.m_linear * m_jacobianPairs[i].m_jacobian_J10.m_linear +
			jInvMass.m_jacobian_J10.m_angular * m_jacobianPairs[i].m_jacobian_J10.m_angular);
		dFloat a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * 1.0004f;

		massMatrix[i][i] = a00;

		m_impulseMag[i] = 0.0f;
		for (int j = i + 1; j < m_rowCount; j++) {
			dVector tmp1(jInvMass.m_jacobian_J01.m_linear * m_jacobianPairs[j].m_jacobian_J01.m_linear +
				jInvMass.m_jacobian_J01.m_angular * m_jacobianPairs[j].m_jacobian_J01.m_angular);
			if (bodyArray[i] == bodyArray[j]) {
				tmp1 += jInvMass.m_jacobian_J10.m_linear * m_jacobianPairs[j].m_jacobian_J10.m_linear;
				tmp1 += jInvMass.m_jacobian_J10.m_angular * m_jacobianPairs[j].m_jacobian_J10.m_angular;
			}

			dFloat a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
			massMatrix[i][j] = a01;
			massMatrix[j][i] = a01;
		}
	}

	dAssert(dTestPSDmatrix(m_rowCount, D_PLAYER_MAX_ROWS, &massMatrix[0][0]));
	dGaussSeidelLcpSor(m_rowCount, D_PLAYER_MAX_ROWS, &massMatrix[0][0], m_impulseMag, m_rhs, m_normalIndex, m_low, m_high, dFloat(1.0e-6f), 32, dFloat(1.1f));

	dVector netImpulse(0.0f);
	for (int i = 0; i < m_rowCount; i++) {
		netImpulse += m_jacobianPairs[i].m_jacobian_J01.m_linear.Scale(m_impulseMag[i]);
	}
	return netImpulse;
}

void dPlayerControllerImpulseSolver::ApplyReaction(dFloat timestep)
{
	dFloat invTimeStep = 0.1f / timestep;
	for (int i = 0; i < m_rowCount; i++) {
		if (m_contactPoint[i]) {
			dVector force(m_jacobianPairs[i].m_jacobian_J10.m_linear.Scale(m_impulseMag[i] * invTimeStep));
			dVector torque(m_jacobianPairs[i].m_jacobian_J10.m_angular.Scale(m_impulseMag[i] * invTimeStep));
			NewtonBodyAddForce(m_contactPoint[i]->m_hitBody, &force[0]);
			NewtonBodyAddTorque(m_contactPoint[i]->m_hitBody, &torque[0]);
		}
	}
}

