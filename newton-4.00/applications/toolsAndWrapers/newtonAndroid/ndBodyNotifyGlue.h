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

#ifndef _ND_BODY_NOTIFY_GLUE_H_
#define _ND_BODY_NOTIFY_GLUE_H_

#include "ndMatrixGlue.h"
#include "ndBodyNotify.h"

class ndBodyNotifyGlue : public ndBodyNotify
{
	public:
	ndBodyNotifyGlue()
		:ndBodyNotify(ndVectorGlue::m_zero)
	{
	}

	virtual ~ndBodyNotifyGlue()
	{
	}

	void SetGravity(const ndVectorGlue& gravity)
	{
		ndBodyNotify::SetGravity(gravity);
	}
	
	virtual void OnApplyExternalForce(ndFloat32 timestep)
	{
		ndAssert(GetBody()->GetAsBodyKinematic());
		ndBodyKinematic* const body = (ndBodyKinematic*)GetBody();
		ndVector force(GetGravity().Scale(body->GetMassMatrix().m_w));
		body->SetForce(force);
		body->SetTorque(ndVectorGlue::m_zero);
	}

	// callback to Java code
	virtual void OnTransformCallback(const ndMatrixGlue& matrix)
	{
	}

	// called from newton cpp core to interact with the app
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix)
	{
		OnTransformCallback(ndMatrixGlue(matrix));
	}

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		OnApplyExternalForce(timestep);
	}
};


#endif 

