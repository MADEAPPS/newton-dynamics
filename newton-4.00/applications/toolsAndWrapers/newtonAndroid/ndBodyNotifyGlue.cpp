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

class ndBodyNotifyInternal : public ndBodyNotify
{
	public:
	ndBodyNotifyInternal(ndBodyNotifyGlue* const parent)
		:ndBodyNotify(ndVector::m_zero)
		,m_notifyParent(parent)
	{
	}

	virtual ~ndBodyNotifyInternal()
	{
	}

	virtual void* GetUserData() const
	{
		return m_notifyParent;
	}

	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix)
	{
		m_notifyParent->OnTransform((ndMatrixGlue&)matrix);
	}

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		m_notifyParent->OnApplyExternalForce(timestep);
	}

	ndBodyNotifyGlue* m_notifyParent;
};

ndBodyNotifyGlue::ndBodyNotifyGlue()
	:m_world(nullptr)
	,m_notify(new ndBodyNotifyInternal(this))
{
}

ndBodyNotifyGlue::~ndBodyNotifyGlue()
{
	// note: m_notify is managed by the newton sdk, do not delete it
}
	
void ndBodyNotifyGlue::SetGravity(const ndVectorGlue& gravity)
{
	m_notify->SetGravity(gravity);
}
	
void ndBodyNotifyGlue::OnApplyExternalForce(ndFloat32 timestep)
{
	ndAssert(m_notify->GetBody()->GetAsBodyKinematic());
	ndBodyKinematic* const body = (ndBodyKinematic*)m_notify->GetBody();
	ndVector force(m_notify->GetGravity().Scale(body->GetMassMatrix().m_w));
	body->SetForce(force);
	body->SetTorque(ndVector::m_zero);
}

void ndBodyNotifyGlue::OnTransform(const ndMatrixGlue& matrix)
{
	ndAssert(0);
}


