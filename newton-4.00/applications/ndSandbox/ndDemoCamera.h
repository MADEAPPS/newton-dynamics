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


#ifndef __DEMO_CAMERA_H__
#define __DEMO_CAMERA_H__

#include "ndSandboxStdafx.h"

class ndDemoCamera: public ndRenderSceneCamera
{
	class ndDemoCameraPickBodyJoint : public ndJointKinematicController
	{
		public:
		ndDemoCameraPickBodyJoint(ndBodyKinematic* const childBody, ndBodyKinematic* const worldBody, const ndVector& attachmentPointInGlobalSpace, ndDemoCamera* const camera)
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

		ndDemoCamera* m_owner;
	};

	public:
	ndDemoCamera(ndRender* const owner);

	void TickUpdate(ndFloat32 timestep);
	void ResetPickBody();
	void UpdatePickBody(bool mousePickState, const ndVector& p0, const ndVector& p1);

	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position) override;

	ndVector m_pickedBodyTargetPosition;
	ndSharedPtr<ndJointBilateralConstraint> m_pickJoint;

	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndFloat32 m_yawRate;
	ndFloat32 m_pitchRate;
	ndFloat32 m_mousePosX;
	ndFloat32 m_mousePosY;
	ndFloat32 m_frontSpeed;
	ndFloat32 m_sidewaysSpeed;
	ndFloat32 m_pickedBodyParam;
	bool m_pickingMode;
	bool m_prevMouseState;
};

#endif 
