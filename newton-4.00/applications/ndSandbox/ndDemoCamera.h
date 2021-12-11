/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoCamera: public ndDemoEntity
{
	public:
	ndDemoCamera();
	~ndDemoCamera();

	dFloat32 GetYawAngle() const;
	dFloat32 GetPichAngle() const;

	const ndMatrix& GetViewMatrix() const;
	const ndMatrix& GetProjectionMatrix() const;

	void SetMatrix (const ndQuaternion& rotation, const ndVector& position);
	void SetViewMatrix (dInt32 width, dInt32 height);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	ndVector ScreenToWorld (const ndVector& screenPoint) const;
	ndVector WorldToScreen (const ndVector& worldPoint) const;

	
	private:
	ndMatrix CreateLookAtMatrix(const ndVector& eye, const ndVector& center, const ndVector& normUp);
	ndMatrix CreateMatrixFromFrustum(dFloat32 Left, dFloat32 Right, dFloat32 Bottom, dFloat32 Top, dFloat32 ZNear, dFloat32 ZFar);
	ndMatrix CreatePerspectiveMatrix(dFloat32 FOV, dFloat32 Aspect, dFloat32 ZNear, dFloat32 ZFar);

	ndMatrix m_viewMatrix;
	ndMatrix m_projectionMatrix;

	dFloat32 m_fov;
	dFloat32 m_backPlane;
	dFloat32 m_frontPlane;
	dFloat32 m_cameraYaw;
	dFloat32 m_cameraPitch;

	GLint m_viewport[4]; 
	friend class ndDemoEntity;
};



#endif 
