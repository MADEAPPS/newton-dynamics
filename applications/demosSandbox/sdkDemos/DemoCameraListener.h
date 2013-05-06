/* Copyright (c) <2009> <Newton Game Dynamics>
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

#include <toolbox_stdafx.h>
#include "DemoEntity.h"


class DemoCameraListener: public DemoListenerBase
{
	public:
	DemoCameraListener(DemoEntityManager* const scene);
	~DemoCameraListener();

	DemoCamera* GetCamera() const 
	{
		return m_camera;
	}

	void SetCameraMatrix (DemoEntityManager* const scene, const dQuaternion& rotation, const dVector& position) const
	{
		m_camera->SetMatrix(*scene, rotation, position);
		m_camera->SetMatrix(*scene, rotation, position);
	}

	void RenderPickedTarget () const;
	void InterpolateMatrices (DemoEntityManager* const scene, dFloat timeStepFraction);

	private:
	virtual void PreUpdate (const NewtonWorld* const world, dFloat timestep);
	virtual void PostUpdate (const NewtonWorld* const world, dFloat timestep);

	void UpdatePickBody (DemoEntityManager* const scene, float timestep); 

	static void OnPickedBodyDestroyedNotify (const NewtonBody* body);
	static void OnPickedBodyApplyForce (const NewtonBody* body, dFloat timestep, int threadIndex);
	
	DemoCamera* m_camera;

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
	static dVector m_pickedBodyDisplacement;
	static dVector m_pickedBodyLocalAtachmentPoint;
	static dVector m_pickedBodyLocalAtachmentNormal;
	static NewtonBody* m_targetPicked;
	static NewtonBodyDestructor m_bodyDestructor;
	static NewtonApplyForceAndTorque m_chainPickBodyForceCallback; 

	friend DemoCamera;
};

#endif 

