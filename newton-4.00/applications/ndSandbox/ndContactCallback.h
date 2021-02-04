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
#ifndef __CONTACT_CALLBACK_H__
#define __CONTACT_CALLBACK_H__

#include "ndSandboxStdafx.h"

class ndContactCallback: public ndContactNotify
{
	public: 
	ndContactCallback();
	virtual void OnBodyAdded(ndBodyKinematic* const body) const;
	virtual void OnBodyRemoved(ndBodyKinematic* const body) const;
	virtual ndMaterial GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const;
	virtual bool OnAaabbOverlap(const ndContact* const contactJoint, dFloat32 timestep);
	virtual void OnContactCallback(dInt32 threadIndex, const ndContact* const contactJoint, dFloat32 timestep);
};


#endif