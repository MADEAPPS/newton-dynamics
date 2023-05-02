/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndModelStdafx.h"
#include "ndModelNotify.h"

ndModelNotify::ndModelNotify(ndBodyKinematic* const parentBody, ndVector gravity)
	:ndBodyNotify(gravity)
	,m_parentBody(parentBody)
{
	//static ndModelNotifyFileLoadSave registerClass;
}

ndModelNotify::~ndModelNotify()
{
}

void ndModelNotify::OnApplyExternalForce(ndInt32, ndFloat32)
{
	ndBodyKinematic* const body = GetBody()->GetAsBodyKinematic();
	ndAssert(body);
	if (body->GetInvMass() > 0.0f)
	{
		ndVector massMatrix(body->GetMassMatrix());
		ndVector force(GetGravity().Scale(massMatrix.m_w));
		body->SetForce(force);
		body->SetTorque(ndVector::m_zero);
	}
}

bool ndModelNotify::CheckInWorld(const ndMatrix& matrix) const
{
	return matrix.m_posit.m_y > -100.0f;
}

void ndModelNotify::CalculateMatrix(const ndMatrix& matrix, ndQuaternion& rot, ndVector& posit) const
{
	const ndBody* const body = GetBody();
	if (!m_parentBody)
	{
		rot = body->GetRotation();
		posit = matrix.m_posit;
		return;
	}
	const ndMatrix parentMatrix(m_parentBody->GetMatrix());
	const ndMatrix localMatrix(matrix * parentMatrix.OrthoInverse());
	rot = localMatrix;
	posit = localMatrix.m_posit;
}

void ndModelNotify::OnTransform(ndInt32, const ndMatrix&)
{
	//ndAssert(0);
}

