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


#ifndef __DEMO_CAMERA_NODE_H__
#define __DEMO_CAMERA_NODE_H__

#include "ndSandboxStdafx.h"

class ndDemoCameraNode;
class ndDemoCameraPickBodyJoint : public ndJointKinematicController
{
	public:
	ndDemoCameraPickBodyJoint(ndBodyKinematic* const childBody, ndBodyKinematic* const worldBody, 
		const ndVector& attachmentPointInGlobalSpace, ndDemoCameraNode* const camera)
		:ndJointKinematicController(childBody, worldBody, attachmentPointInGlobalSpace)
		,m_owner(camera)
	{
	}

	~ndDemoCameraPickBodyJoint()
	{
		if (m_owner)
		{
			ndAssert(0);
			//m_owner->ResetPickBody();
		}
	}

	ndDemoCameraNode* m_owner;
};

class ndDemoCameraNode: public ndRenderSceneNode
{
	public:
	ndDemoCameraNode(ndRender* const owner);

	virtual void TickUpdate(ndFloat32 timestep) = 0;
	
	protected:
	void ResetPickBody();
	void UpdatePickBody();

	ndVector m_pickedBodyTargetPosition;
	ndSharedPtr<ndJointBilateralConstraint> m_pickJoint;

	ndFloat32 m_pickedBodyParam;
	bool m_pickingMode;
	bool m_prevMouseState;
};

#endif 
