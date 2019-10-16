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
#include "dVehicleTireContact.h"

dTireContact::dTireContact()
	:m_point(0.0f)
	,m_normal(0.0f)
	,m_lateralDir(0.0f)
	,m_longitudinalDir(0.0f)
	,m_penetration(0.0f)
	,m_staticFriction(1.0f)
	,m_kineticFriction(1.0f)
	,m_load(0.0f)
	,m_tireModel()
{
	//m_jointFeebackForce[0] = 0.0f;
	//m_jointFeebackForce[1] = 0.0f;
	//m_jointFeebackForce[2] = 0.0f;
	memset(m_normalFilter, 0, sizeof(m_normalFilter));
	memset(m_isActiveFilter, 0, sizeof(m_isActiveFilter));
}

void dTireContact::ResetContact()
{
	dTrace(("%s\n", __FUNCTION__));
/*
	if (m_isActive == false) {
		m_jointFeebackForce[0] = 0.0f;
		m_jointFeebackForce[1] = 0.0f;
		m_jointFeebackForce[2] = 0.0f;
		memset(m_normalFilter, 0, sizeof(m_normalFilter));
		memset(m_isActiveFilter, 0, sizeof(m_isActiveFilter));
	}
	m_load = 0.0f;
	m_isActive = false;
	for (int i = sizeof(m_isActiveFilter) / sizeof(m_isActiveFilter[0]) - 1; i > 0; i--) {
		m_normalFilter[i] = m_normalFilter[i - 1];
		m_isActiveFilter[i] = m_isActiveFilter[i - 1];
	}
*/
}

void dTireContact::SetContact(const dVector& posit, const dVector& normal, const dVector& longitudinalDir, dFloat penetration, dFloat staticFriction, dFloat kineticFriction)
{
	dTrace(("%s\n", __FUNCTION__));
/*
	m_point = posit;
	m_normal = normal;
	m_longitudinalDir = longitudinalDir;
	m_lateralDir = m_longitudinalDir.CrossProduct(m_normal);

	m_isActive = true;
	m_isActiveFilter[0] = true;
	m_normalFilter[0] = m_jointFeebackForce[0];

	dFloat load = 0.0f;
	for (int i = 0; i < sizeof(m_isActiveFilter) / sizeof(m_isActiveFilter[0]); i++) {
		load += m_normalFilter[i];
	}
	m_load = load * (1.0f / (sizeof(m_isActiveFilter) / sizeof(m_isActiveFilter[0])));

	m_staticFriction = staticFriction;
	m_kineticFriction = kineticFriction;
	m_penetration = dClamp(penetration, dFloat(-D_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_TIRE_MAX_ELASTIC_DEFORMATION));
*/
}

