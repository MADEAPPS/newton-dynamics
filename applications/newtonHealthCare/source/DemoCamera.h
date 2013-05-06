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

#ifndef __DEMO_CAMERA_H__
#define __DEMO_CAMERA_H__

#include <toolbox_stdafx.h>
#include "DemoEntity.h"


class DemoCamera: public DemoEntity
{
	public:
	DemoCamera();
	~DemoCamera();

	void RenderPickedTarget () const;
	
	void SetMatrix (DemoEntityManager& world, const dQuaternion& rotation, const dVector& position);
	void SetViewMatrix (int width, int height);

	virtual void Render(dFloat timeStep) const;
	virtual void UpdateInputs (float timestep, const NewtonDemos* const mainWin, DemoEntityManager& world);

	static void PhysicsApplyPickForce (const NewtonBody* body, dFloat timestep, int threadIndex);

	private:
	float m_yawRate;
	float m_pitchRate;
	float m_sidewaysSpeed;
	float m_cameraFrontSpeed;
	float m_cameraYaw;
	float m_cameraPitch;
	int m_mousePosX;
	int m_mousePosY;
	dFloat m_fov;
	dFloat m_frontPlane;
	dFloat m_backPlane;
	bool m_prevMouseState;

	NewtonBody* m_targetPicked;
	static dFloat m_pickedBodyParam;
	static dVector m_pickedBodyDisplacement;
	static dVector m_pickedBodyLocalAtachmentPoint;
	static dVector m_pickedBodyLocalAtachmentNormal;
	static NewtonApplyForceAndTorque m_chainPickBodyForceCallback; 
};



class ThirdPersonCamera: public DemoCamera
{
	public:
	ThirdPersonCamera(DemoEntity* const targetEntity);
	void UpdateInputs (float timestep, const NewtonDemos* const mainWin, DemoEntityManager& world); 

	dFloat m_camYaw; 
	dFloat m_camPitch; 
	dFloat m_camDist; 
	DemoEntity* m_targetEntity;
	bool m_toggleFreeCamera;
};


#endif 

