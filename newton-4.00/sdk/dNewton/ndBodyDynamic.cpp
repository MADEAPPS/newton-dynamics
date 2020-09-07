/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


//dgVector ntDynamicBody::m_equilibriumError2 (DG_ERR_TOLERANCE2);

ndBodyDynamic::ndBodyDynamic()
	:ndBodyKinematic()
	,m_accel(dVector::m_zero)
	,m_alpha(dVector::m_zero)
	,m_externalForce(dVector::m_zero)
	,m_externalTorque(dVector::m_zero)
	,m_jointArray()
{
	SetMassMatrix(dVector::m_zero);
}

ndBodyDynamic::~ndBodyDynamic()
{
}

void ndBodyDynamic::ApplyExternalForces(dInt32 threadIndex, dFloat32 timestep)
{
	m_externalForce = dVector::m_zero;
	m_externalTorque = dVector::m_zero;
	if (m_notifyCallback)
	{
		m_notifyCallback->OnApplyExternalForce(threadIndex, timestep);
		if (m_invMass.m_w == dFloat32(0.0f)) 
		{
			m_externalForce = dVector::m_zero;
			m_externalForce = dVector::m_zero;
		}
		m_externalForce = m_externalForce & dVector::m_triplexMask;
		m_externalTorque = m_externalTorque & dVector::m_triplexMask;
	}
}

void ndBodyDynamic::AddDampingAcceleration(dFloat32 timestep)
{
	//dgVector damp(GetDampCoeffcient(timestep));
	//dgVector omegaDamp(damp & dgVector::m_triplexMask);
	//dgVector omega(m_matrix.UnrotateVector(m_omega) * omegaDamp);
	//
	//m_veloc = m_veloc.Scale(damp.m_w);
	//m_omega = m_matrix.RotateVector(omega);
}

