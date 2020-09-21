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


// RenderPrimitive.h: interface for the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __DEMO_CAMERA_LISTENER_H__
#define __DEMO_CAMERA_LISTENER_H__

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoCamera;
class ndDemoCameraPickBodyJoint;
class ndDemoCameraManager
{
	public:
	ndDemoCameraManager(ndDemoEntityManager* const scene);
	~ndDemoCameraManager();

	ndDemoCamera* GetCamera() const 
	{
		return m_camera;
	}

	void SetCameraMatrix (ndDemoEntityManager* const scene, const dQuaternion& rotation, const dVector& position);

	void SetCameraMouseLock (bool loockState);

	void RenderPickedTarget () const;
	void InterpolateMatrices (ndDemoEntityManager* const scene, dFloat timeStepFraction);

	virtual void FixUpdate(const NewtonWorld* const world, dFloat timestep);

	void ResetPickBody();

	private:
	virtual void OnBodyDestroy (NewtonBody* const body);
	void UpdatePickBody (ndDemoEntityManager* const scene, bool mouseState, const dVector& camPos0, const dVector& camPos1, dFloat timestep); 
	
	ndDemoCamera* m_camera;
	int m_mousePosX;
	int m_mousePosY;
	dFloat m_yaw;
	dFloat m_pitch;
	dFloat m_yawRate;
	dFloat m_pitchRate;
	dFloat m_frontSpeed;
	dFloat m_sidewaysSpeed;
	dFloat m_pickedBodyParam;

	bool m_prevMouseState;	
	bool m_mouseLockState;	
	dVector m_pickedBodyTargetPosition;
	dVector m_pickedBodyLocalAtachmentPoint;
	dVector m_pickedBodyLocalAtachmentNormal;
	NewtonBody* m_targetPicked;
	ndDemoCameraPickBodyJoint* m_pickJoint;
	NewtonBodyDestructor m_bodyDestructor;
	friend class ndDemoCamera;
};

#endif 

