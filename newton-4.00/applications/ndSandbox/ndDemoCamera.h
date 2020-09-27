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

	const dMatrix& GetViewMatrix() const;
	const dMatrix& GetProjectionMatrix() const;

	void SetMatrix (ndDemoEntityManager& world, const dQuaternion& rotation, const dVector& position);
	void SetViewMatrix (int width, int height);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

	dVector ScreenToWorld (const dVector& screenPoint) const;
	dVector WorldToScreen (const dVector& worldPoint) const;
	
	private:
	dMatrix CreateLookAtMatrix(const dVector& eye, const dVector& center, const dVector& normUp);
	dMatrix CreateMatrixFromFrustum(dFloat32 Left, dFloat32 Right, dFloat32 Bottom, dFloat32 Top, dFloat32 ZNear, dFloat32 ZFar);
	dMatrix CreatePerspectiveMatrix(dFloat32 FOV, dFloat32 Aspect, dFloat32 ZNear, dFloat32 ZFar);


	dMatrix m_viewMatrix;
	dMatrix m_projectionMatrix;

	dFloat32 m_fov;
	dFloat32 m_backPlane;
	dFloat32 m_frontPlane;
	dFloat32 m_cameraYaw;
	dFloat32 m_cameraPitch;

	GLint m_viewport[4]; 
	//GLdouble m_modelViewMatrix[16];
	//GLdouble m_projectionViewMatrix[16];
	friend class ndDemoEntity;
};



#endif 
