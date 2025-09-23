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


#ifndef __DEMO_CAMERA_NODE_FOLLOW_H__
#define __DEMO_CAMERA_NODE_FOLLOW_H__

#include "ndSandboxStdafx.h"
#include "ndDemoCameraNode.h"

class ndDemoCameraNodeFollow: public ndDemoCameraNode
{
	public:
	ndDemoCameraNodeFollow(ndRender* const owner, const ndVector& pivot, ndFloat32 distance);

	void TickUpdate(ndFloat32 timestep);
	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position) override;

	ndVector m_pivot;
	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndFloat32 m_yawRate;
	ndFloat32 m_pitchRate;
	ndFloat32 m_mousePosX;
	ndFloat32 m_mousePosY;
};

#endif 
