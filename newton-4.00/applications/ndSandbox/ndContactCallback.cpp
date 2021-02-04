/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndContactCallback.h"

ndContactCallback::ndContactCallback()
	:ndContactNotify()
{
}

void ndContactCallback::OnBodyAdded(ndBodyKinematic* const body) const
{
}

void ndContactCallback::OnBodyRemoved(ndBodyKinematic* const body) const
{
}

ndMaterial ndContactCallback::GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	return ndMaterial();
}

bool ndContactCallback::OnAaabbOverlap(const ndContact* const contactJoint, dFloat32 timestep)
{
	return true;
}

void ndContactCallback::OnContactCallback(dInt32 threadIndex, const ndContact* const contactJoint, dFloat32 timestep)
{
}
