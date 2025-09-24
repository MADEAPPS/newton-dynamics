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


#ifndef __DEMO_CAMERA_NODE_FLYBY_H__
#define __DEMO_CAMERA_NODE_FLYBY_H__

#include "ndSandboxStdafx.h"
#include "ndDemoCameraNode.h"

class ndDemoCameraNodeFlyby: public ndDemoCameraNode
{
	public:
	ndDemoCameraNodeFlyby(ndRender* const owner);

	void TickUpdate(ndFloat32 timestep);
	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position) override;

	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndFloat32 m_yawRate;
	ndFloat32 m_pitchRate;
	ndFloat32 m_mousePosX;
	ndFloat32 m_mousePosY;
	ndFloat32 m_frontSpeed;
	ndFloat32 m_sidewaysSpeed;
};

#endif 
