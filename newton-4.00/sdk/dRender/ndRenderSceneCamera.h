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
#ifndef __ND_RENDER_SCENE_CAMERA_H__
#define __ND_RENDER_SCENE_CAMERA_H__

#include "ndRenderStdafx.h"
#include "ndRenderSceneNode.h"

class ndRenderSceneCamera : public ndRenderSceneNode
{
	public:
	ndRenderSceneCamera(ndRender* const owner);

	virtual ndRenderSceneCamera* GetAsCamera() override;
	virtual const ndRenderSceneCamera* GetAsCamera() const override;

	virtual void SetMatrix(const ndQuaternion& rotation, const ndVector& position) override;

	void SetViewMatrix(ndInt32 width, ndInt32 height);
	ndMatrix CreatePerspectiveMatrix(ndFloat32 fov, ndFloat32 aspect, ndFloat32 front, ndFloat32 back) const;
	ndMatrix CreateLookAtMatrix(const ndVector& eyepoint, const ndVector& eyepointTarget, const ndVector& normUp) const;
	ndMatrix CreateMatrixFromFrustum(ndFloat32 left, ndFloat32 right, ndFloat32 bottom, ndFloat32 top, ndFloat32 front, ndFloat32 back) const;

	ndVector ScreenToWorld(const ndVector& screenPoint) const;

	ndMatrix m_viewMatrix;
	ndMatrix m_invViewMatrix;
	ndMatrix m_projectionMatrix;
	ndMatrix m_invProjectionMatrix;
	ndMatrix m_invViewRrojectionMatrix;
	ndVector m_frustum[8];

	ndFloat32 m_fov;
	ndFloat32 m_backPlane;
	ndFloat32 m_frontPlane;
	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndInt32 m_viewport[4];
	static ndMatrix m_worldToOpenGl;

	friend class ndRenderPassShadowsImplement;
};

#endif