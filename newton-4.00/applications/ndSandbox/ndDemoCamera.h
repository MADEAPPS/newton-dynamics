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

	ndFloat32 GetYawAngle() const;
	ndFloat32 GetPichAngle() const;

	const ndMatrix& GetViewMatrix() const;
	const ndMatrix& GetInvViewMatrix() const;
	const ndMatrix& GetProjectionMatrix() const;

	const ndMatrix& GetWorlToGlMatrix() const;

	void SetMatrix (const ndQuaternion& rotation, const ndVector& position);
	void SetViewMatrix (ndInt32 width, ndInt32 height);

	virtual void Render(ndFloat32 timeStep, ndDemoEntityManager* const scene, const ndMatrix& matrix) const;

	ndVector ScreenToWorld (const ndVector& screenPoint) const;
	ndVector WorldToScreen (const ndVector& worldPoint) const;
	
	private:
	ndMatrix CreateLookAtMatrix(const ndVector& eye, const ndVector& center, const ndVector& normUp);
	ndMatrix CreateMatrixFromFrustum(ndFloat32 Left, ndFloat32 Right, ndFloat32 Bottom, ndFloat32 Top, ndFloat32 ZNear, ndFloat32 ZFar);
	ndMatrix CreatePerspectiveMatrix(ndFloat32 FOV, ndFloat32 Aspect, ndFloat32 ZNear, ndFloat32 ZFar);

	ndMatrix m_viewMatrix;
	ndMatrix m_invViewMatrix;
	ndMatrix m_projectionMatrix;
	ndMatrix m_invProjectionMatrix;
	ndVector m_frustum[8];

	ndFloat32 m_fov;
	ndFloat32 m_backPlane;
	ndFloat32 m_frontPlane;
	ndFloat32 m_cameraYaw;
	ndFloat32 m_cameraPitch;

	static ndMatrix m_worldToOpenGl;

	ndInt32 m_viewport[4]; 
	friend class ndDemoEntity;
	friend class ndShadowMapRenderPass;
};

inline ndFloat32 ndDemoCamera::GetYawAngle() const
{
	return m_cameraYaw;
}

inline ndFloat32 ndDemoCamera::GetPichAngle() const
{
	return m_cameraPitch;
}

inline const ndMatrix& ndDemoCamera::GetProjectionMatrix() const
{
	return m_projectionMatrix;
}

inline const ndMatrix& ndDemoCamera::GetViewMatrix() const
{
	return m_viewMatrix;
}

inline const ndMatrix& ndDemoCamera::GetInvViewMatrix() const
{
	return m_invViewMatrix;
}

inline const ndMatrix& ndDemoCamera::GetWorlToGlMatrix() const
{
	return m_worldToOpenGl;
}

#endif 
