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
class ndBodyNotifyInternal;

class ndBodyNotifyGlue
{
	public:
	ndBodyNotifyGlue();
	virtual ~ndBodyNotifyGlue();

	void SetGravity(const ndVectorGlue& gravity);

	virtual void OnTransform(const ndMatrixGlue& matrix);
	virtual void OnApplyExternalForce(ndFloat32 timestep);

	private:
	ndWorldGlue* m_world;
	ndBodyNotifyInternal* m_notify;
	friend class ndWorldGlue;
	friend class ndRigidBodyGlue;
};

#endif 

