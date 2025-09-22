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

#ifndef __ND_MODEL_BODY_NOTIFY_H__
#define __ND_MODEL_BODY_NOTIFY_H__

#include "ndModelStdafx.h"

class ndModelBodyNotify: public ndBodyNotify
{
	public:
	D_CLASS_REFLECTION(ndModelBodyNotify, ndBodyNotify)

	ndModelBodyNotify(const ndModelBodyNotify& src);
	ndModelBodyNotify(ndBodyKinematic* const parentBody = nullptr, ndVector gravity = ndVector (ndFloat32 (0.0f), ndFloat32(-10.0f), ndFloat32(0.0f), ndFloat32(0.0f)));
	virtual ~ndModelBodyNotify();

	ndBodyKinematic* GetParentBody() const;
	void SetParentBody(ndBodyKinematic* const kinematicBody);

	ndBodyNotify* Clone() const override
	{
		return new ndModelBodyNotify(*this);
	}

	//virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix) override;
	virtual void OnTransform(ndFloat32 timestep, const ndMatrix& matrix) override;
	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep) override;

	bool CheckInWorld(const ndMatrix& matrix) const;
	void CalculateMatrix(const ndMatrix& matrix, ndQuaternion& rot, ndVector& posit) const;

	ndBodyKinematic* m_parentBody;
};

#endif
