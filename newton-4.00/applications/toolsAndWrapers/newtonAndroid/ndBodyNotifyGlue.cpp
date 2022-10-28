/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndNewton.h"
#include "ndVectorGlue.h"
#include "ndBodyNotifyGlue.h"

ndBodyNotifyGlue::ndBodyNotifyGlue()
	:ndBodyNotify(ndVectorGlue::m_zero)
	,m_world(nullptr)
{
}

ndBodyNotifyGlue::~ndBodyNotifyGlue()
{
}
	
void ndBodyNotifyGlue::SetGravity(const ndVectorGlue& gravity)
{
	ndBodyNotify::SetGravity(gravity);
}
	
void ndBodyNotifyGlue::OnApplyExternalForce(ndFloat32 timestep)
{
	ndAssert(GetBody()->GetAsBodyKinematic());
	ndBodyKinematic* const body = (ndBodyKinematic*)GetBody();
	ndVector force(GetGravity().Scale(body->GetMassMatrix().m_w));
	body->SetForce(force);
	body->SetTorque(ndVectorGlue::m_zero);
}

void ndBodyNotifyGlue::OnTransformCallback(const ndMatrixGlue& matrix)
{
}

void ndBodyNotifyGlue::OnTransform(ndInt32 threadIndex, const ndMatrix& matrix)
{
	OnTransformCallback(ndMatrixGlue(matrix));
}

void ndBodyNotifyGlue::OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
{
	OnApplyExternalForce(timestep);
}

