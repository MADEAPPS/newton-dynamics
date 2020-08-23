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
#include "dVehicleManager.h"
#include "dMultiBodyVehicleTire.h"
#include "dMultiBodyVehicleEngine.h"
#include "dMultiBodyVehicleDifferential.h"

dVehicle::dVehicle(NewtonBody* const body, const dMatrix& localFrame, dFloat gravityMag)
	:dVehicleNode(NULL)
	,m_localFrame(localFrame)
	,m_gravity(0.0f, -dAbs(gravityMag), 0.0f, 0.0f)
	,m_obbSize(0.0f)
	,m_obbOrigin(0.0f)
	,m_newtonBody(body)
	,m_managerNode(NULL)
	,m_manager(NULL)
{
	m_localFrame.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);
	dAssert(m_localFrame.TestOrthogonal());
}

dVehicle::~dVehicle()
{
	if (m_managerNode) {
		m_manager->RemoveRoot(this);
	}
}

