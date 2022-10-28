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

class ndWorldGlue;

class ndBodyNotifyGlue : public ndBodyNotify
{
	public:
	ndBodyNotifyGlue();
	virtual ~ndBodyNotifyGlue();

	void SetGravity(const ndVectorGlue& gravity);
	virtual void OnApplyExternalForce(ndFloat32 timestep);

	// callback to Java code
	virtual void OnTransformCallback(const ndMatrixGlue& matrix);

	// called from newton cpp core to interact with the app
	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	private:
	ndWorldGlue* m_world;
	friend class ndWorldGlue;
	friend class ndRigidBodyGlue;
};


#endif 

