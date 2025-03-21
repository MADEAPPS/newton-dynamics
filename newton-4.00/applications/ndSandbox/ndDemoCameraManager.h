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

#ifndef __DEMO_CAMERA_LISTENER_H__
#define __DEMO_CAMERA_LISTENER_H__

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoCamera;
class ndDemoCameraPickBodyJoint;

class ndDemoCameraManager: public ndClassAlloc
{
	public:
	ndDemoCameraManager(ndDemoEntityManager* const scene);
	~ndDemoCameraManager();

	ndDemoCamera* GetCamera()
	{
		return *m_camera;
	}

	void SetCameraMatrix (const ndQuaternion& rotation, const ndVector& position);

	void SetCameraMouseLock (bool loockState);

	void SetPickMode(bool mode) {m_pickingMode = mode;}

	void RenderPickedTarget () const;
	void InterpolateMatrices (ndDemoEntityManager* const scene, ndFloat32 timeStepFraction);

	//virtual void FixUpdate(ndDemoEntityManager* const scene, ndFloat32 timestep);
	void FixUpdate(ndDemoEntityManager* const scene, ndFloat32 timestep);

	void ResetPickBody();

	private:
	//virtual void OnBodyDestroy (NewtonBody* const body);
	void UpdatePickBody (ndDemoEntityManager* const scene, bool mouseState, const ndVector& camPos0, const ndVector& camPos1, ndFloat32 timestep); 

	ndVector m_pickedBodyTargetPosition;
	ndVector m_pickedBodyLocalAtachmentPoint;
	ndVector m_pickedBodyLocalAtachmentNormal;

	//ndDemoCamera* m_camera;
	ndSharedPtr<ndDemoCamera> m_camera;
	ndSharedPtr<ndJointBilateralConstraint> m_pickJoint;
	ndFloat32 m_mousePosX;
	ndFloat32 m_mousePosY;
	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndFloat32 m_yawRate;
	ndFloat32 m_pitchRate;
	ndFloat32 m_frontSpeed;
	ndFloat32 m_sidewaysSpeed;
	ndFloat32 m_pickedBodyParam;
	
	bool m_prevMouseState;	
	bool m_mouseLockState;
	bool m_pickingMode;

	friend class ndDemoCamera;
};

#endif 

