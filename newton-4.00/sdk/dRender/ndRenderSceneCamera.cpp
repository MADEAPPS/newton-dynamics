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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderSceneCamera.h"

#define D_RENDER_CAMERA_ANGLE	60.0f

ndMatrix ndRenderSceneCamera::m_worldToOpenGl(
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f)));

ndRenderSceneCamera::ndRenderSceneCamera(ndRender* const owner)
	:ndRenderSceneNode(ndGetIdentityMatrix())
	,m_viewMatrix(ndGetIdentityMatrix())
	,m_invViewMatrix(ndGetIdentityMatrix())
	,m_projectionMatrix(ndGetIdentityMatrix())
	,m_invProjectionMatrix(ndGetIdentityMatrix())
	,m_fov(D_RENDER_CAMERA_ANGLE* ndDegreeToRad)
	,m_backPlane(ndFloat32(2000.0f))
	,m_frontPlane(ndFloat32(0.1f))
	,m_yaw(ndFloat32(0.0f))
	,m_pitch(ndFloat32(0.0f))
{
	m_owner = owner;
}

ndRenderSceneCamera* ndRenderSceneCamera::GetAsCamera()
{
	return this;
}

const ndRenderSceneCamera* ndRenderSceneCamera::GetAsCamera() const
{
	return this;
}

ndMatrix ndRenderSceneCamera::CreateMatrixFromFrustum(ndFloat32 left, ndFloat32 right, ndFloat32 bottom, ndFloat32 top, ndFloat32 front, ndFloat32 back) const
{
	ndMatrix projectionMatrix(ndGetIdentityMatrix());

	projectionMatrix[0][0] = ndFloat32(2.0f) * front / (right - left);
	projectionMatrix[0][1] = ndFloat32(0.0f);
	projectionMatrix[0][2] = ndFloat32(0.0f);
	projectionMatrix[0][3] = ndFloat32(0.0f);

	projectionMatrix[1][0] = ndFloat32(0.0f);
	projectionMatrix[1][1] = ndFloat32(2.0f) * front / (top - bottom);
	projectionMatrix[1][2] = ndFloat32(0.0f);
	projectionMatrix[1][3] = ndFloat32(0.0f);

	projectionMatrix[2][0] = (right + left) / (right - left);
	projectionMatrix[2][1] = (top + bottom) / (top - bottom);
	projectionMatrix[2][2] = -(back + front) / (back - front);
	projectionMatrix[2][3] = ndFloat32(-1.0f);

	projectionMatrix[3][0] = ndFloat32(0.0f);
	projectionMatrix[3][1] = ndFloat32(0.0f);
	projectionMatrix[3][2] = ndFloat32(-2.0f) * back * front / (back - front);
	projectionMatrix[3][3] = ndFloat32(0.0f);

	return projectionMatrix;
}

ndMatrix ndRenderSceneCamera::CreatePerspectiveMatrix(ndFloat32 fov, ndFloat32 aspect, ndFloat32 front, ndFloat32 back) const
{
	fov = ndClamp(fov, ndFloat32(0.0f), ndPi);
	ndFloat32 y = front * ndTan(fov * ndFloat32(0.5f));
	ndFloat32 x = y * aspect;
	ndMatrix projectionMatrix(CreateMatrixFromFrustum(-x, x, -y, y, front, back));
	return m_worldToOpenGl * projectionMatrix;
}

ndMatrix ndRenderSceneCamera::CreateLookAtMatrix(const ndVector& eyepoint, const ndVector& eyepointTarget, const ndVector& normUp) const
{
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_front = ((eyepointTarget - eyepoint) & ndVector::m_triplexMask).Normalize();
	matrix.m_right = (matrix.m_front.CrossProduct(normUp) & ndVector::m_triplexMask).Normalize();
	matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	matrix.m_posit = (eyepoint & ndVector::m_triplexMask) | ndVector::m_wOne;
	return matrix;
}

void ndRenderSceneCamera::SetViewMatrix(ndInt32 width, ndInt32 height)
{
	m_viewport[0] = 0;
	m_viewport[1] = 0;
	m_viewport[2] = width;
	m_viewport[3] = height;

	// set the model view matrix 
	const ndMatrix matrix(CalculateGlobalMatrix());
	const ndVector pointOfInterest(matrix.m_posit + matrix.m_front);
	m_viewMatrix = CreateLookAtMatrix(matrix.m_posit, pointOfInterest, matrix.m_up);
	m_invViewMatrix = m_viewMatrix.OrthoInverse();
	
	// calculate projection matrix
	m_projectionMatrix = CreatePerspectiveMatrix(m_fov, ndFloat32(width) / ndFloat32(height), m_frontPlane, m_backPlane);
	m_invProjectionMatrix = m_projectionMatrix.Inverse4x4();
	m_invViewRrojectionMatrix = m_invViewMatrix * m_projectionMatrix;
	
	auto CalculateFrustumPoint = [this](const ndVector& pointInClickSpace, ndFloat32 zdist)
	{
		ndVector point(pointInClickSpace.Scale(zdist));
		point.m_w = zdist;
		point = m_invProjectionMatrix.TransformVector1x4(point);
		return point;
	};
	
	for (ndInt32 j = 0; j < 2; ++j)
	{
		for (ndInt32 i = 0; i < 2; ++i)
		{
			const ndVector sp(ndFloat32(i * 2 - 1), ndFloat32(j * 2 - 1), ndFloat32(-1.0f), ndFloat32(1.0f));
			m_frustum[2 * j + i] = CalculateFrustumPoint(sp, m_frontPlane);
		}
	}
	
	for (ndInt32 j = 0; j < 2; ++j)
	{
		for (ndInt32 i = 0; i < 2; ++i)
		{
			const ndVector sp(ndFloat32(i * 2 - 1), ndFloat32(j * 2 - 1), ndFloat32(1.0f), ndFloat32(1.0f));
			m_frustum[4 + 2 * j + i] = CalculateFrustumPoint(sp, m_backPlane);
		}
	}
}

void ndRenderSceneCamera::SetMatrix(const ndQuaternion& rotation, const ndVector& position)
{
	ndRenderSceneNode::SetMatrix(rotation, position);
}

ndVector ndRenderSceneCamera::ScreenToWorld(const ndVector& screenPoint) const
{
	ndVector sp(screenPoint);
	sp.m_y = (ndFloat32)m_viewport[3] - sp.m_y;

	sp.m_x = ndFloat32(2.0f) * (sp.m_x - (ndFloat32)m_viewport[0]) / (ndFloat32)m_viewport[2] - ndFloat32(1.0f);
	sp.m_y = ndFloat32(2.0f) * (sp.m_y - (ndFloat32)m_viewport[1]) / (ndFloat32)m_viewport[3] - ndFloat32(1.0f);
	sp.m_z = ndFloat32(2.0f) * sp.m_z - ndFloat32(1.0f);
	sp.m_w = ndFloat32(1.0f);

	//sp = viewPoint * ViewMatrx * projeMatrix;
	//ndVector viewPoint(m_viewMatrix.OrthoInverse().TransformVector1x4(m_invProjectionMatrix.TransformVector1x4(sp)));
	ndVector viewPoint(m_viewMatrix.TransformVector1x4(m_invProjectionMatrix.TransformVector1x4(sp)));
	if (viewPoint.m_w != ndFloat32 (0.0f))
	{
		viewPoint = viewPoint.Scale(1.0f / viewPoint.m_w);
	}
	return viewPoint;
}
