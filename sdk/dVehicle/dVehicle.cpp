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

#include "dStdafxVehicle.h"
#include "dVehicle.h"
#include "dVehicleNode.h"
#include "dVehicleTire.h"
#include "dVehicleEngine.h"
#include "dVehicleManager.h"
#include "dVehicleDifferential.h"

dVehicle::dVehicle(NewtonBody* const body, const dMatrix& localFrame, dFloat gravityMag)
	:dVehicleNode(NULL)
	,m_localFrame(localFrame)
	,m_gravity(0.0f, -dAbs(gravityMag), 0.0f, 0.0f)
	,m_obbSize(0.0f)
	,m_obbOrigin(0.0f)
	,m_control(4)
	,m_newtonBody(body)
	,m_managerNode(NULL)
	,m_manager(NULL)
	,m_controlerCount(0)
{
	m_localFrame.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);
	dAssert(m_localFrame.TestOrthogonal());

	// set linear and angular drag to zero
	dVector drag(0.0f);
	NewtonBodySetLinearDamping(m_newtonBody, 0.0f);
	NewtonBodySetAngularDamping(m_newtonBody, &drag[0]);

	// set the inertia matrix;
	dMatrix matrix(dGetIdentityMatrix());
	NewtonBodyGetCentreOfMass(m_newtonBody, &matrix.m_posit[0]);
	matrix.m_posit.m_w = 1.0f;
	m_proxyBody.SetLocalMatrix(matrix);
}

dVehicle::~dVehicle()
{
	if (m_managerNode) {
		m_manager->RemoveRoot(this);
	}
}


void dVehicle::AddControl(dVehicleDashControl* const control)
{
	m_control[m_controlerCount] = control;
	m_controlerCount ++;
}

void dVehicle::RemoveControl(dVehicleDashControl* const control)
{
	for (int i = 0; i < m_controlerCount; i ++) {
		if (control == m_control[m_controlerCount]) {
			for (int j = i + 1; i < m_controlerCount; i ++) {
				m_control[j - 1] = m_control[j];
			}
		}
		return;
	}
}
