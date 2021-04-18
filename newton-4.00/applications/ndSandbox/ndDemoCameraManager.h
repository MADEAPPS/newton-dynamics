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

#ifndef __DEMO_CAMERA_LISTENER_H__
#define __DEMO_CAMERA_LISTENER_H__

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoCamera;
class ndDemoCameraPickBodyJoint;

class ndDemoCameraManager: public dClassAlloc
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

	void SetPickMode(bool mode) {m_pickingMode = mode;}

	void RenderPickedTarget () const;
	void InterpolateMatrices (ndDemoEntityManager* const scene, dFloat32 timeStepFraction);

	//virtual void FixUpdate(ndDemoEntityManager* const scene, dFloat32 timestep);
	void FixUpdate(ndDemoEntityManager* const scene, dFloat32 timestep);

	void ResetPickBody();

	private:
	//virtual void OnBodyDestroy (NewtonBody* const body);
	void UpdatePickBody (ndDemoEntityManager* const scene, bool mouseState, const dVector& camPos0, const dVector& camPos1, dFloat32 timestep); 

	dVector m_pickedBodyTargetPosition;
	dVector m_pickedBodyLocalAtachmentPoint;
	dVector m_pickedBodyLocalAtachmentNormal;

	ndDemoCamera* m_camera;
	ndBodyKinematic* m_targetPicked;
	ndDemoCameraPickBodyJoint* m_pickJoint;
	dFloat32 m_mousePosX;
	dFloat32 m_mousePosY;
	dFloat32 m_yaw;
	dFloat32 m_pitch;
	dFloat32 m_yawRate;
	dFloat32 m_pitchRate;
	dFloat32 m_frontSpeed;
	dFloat32 m_sidewaysSpeed;
	dFloat32 m_pickedBodyParam;
	
	bool m_prevMouseState;	
	bool m_mouseLockState;
	bool m_pickingMode;

	friend class ndDemoCamera;
};

#endif 

