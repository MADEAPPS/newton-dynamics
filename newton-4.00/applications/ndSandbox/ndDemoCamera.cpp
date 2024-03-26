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

#include "ndSandboxStdafx.h"
#include "ndDemoCamera.h"

#define MOUSE_PICK_DAMP			10.0f
#define MOUSE_PICK_STIFFNESS	100.0f

#define D_CAMERA_ANGLE			60.0f

ndMatrix ndDemoCamera::m_worldToOpenGl(
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32( 0.0f), ndFloat32(1.0f)));


ndDemoCamera::ndDemoCamera()
	:ndDemoEntity (ndGetIdentityMatrix(), nullptr) 
	,m_viewMatrix(ndGetIdentityMatrix())
	,m_projectionMatrix(ndGetIdentityMatrix())
	,m_invProjectionMatrix(ndGetIdentityMatrix())
	,m_fov(D_CAMERA_ANGLE * ndDegreeToRad)
	,m_backPlane(ndFloat32(2000.0f))
	,m_frontPlane (ndFloat32(0.1f))
	,m_cameraYaw(ndFloat32(0.0f))
	,m_cameraPitch(ndFloat32(0.0f))
{
}
	
ndDemoCamera::~ndDemoCamera()
{
}

void ndDemoCamera::Render(ndFloat32, ndDemoEntityManager* const, const ndMatrix&) const
{
}

ndFloat32 ndDemoCamera::GetYawAngle() const
{
	return m_cameraYaw;
}

ndFloat32 ndDemoCamera::GetPichAngle() const
{
	return m_cameraPitch;
}

const ndMatrix& ndDemoCamera::GetProjectionMatrix() const
{
	return m_projectionMatrix;
}

const ndMatrix& ndDemoCamera::GetViewMatrix() const
{
	return m_viewMatrix;
}

ndMatrix ndDemoCamera::CreateMatrixFromFrustum(ndFloat32 left, ndFloat32 right, ndFloat32 bottom, ndFloat32 top, ndFloat32 front, ndFloat32 back)
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

ndMatrix ndDemoCamera::CreateLookAtMatrix(const ndVector& eyepoint, const ndVector& eyepointTarget, const ndVector& normUp)
{
	//ndMatrix result(ndGetIdentityMatrix());
	//ndVector zAxis(eyepoint - eyepointTarget);
	//zAxis = zAxis & ndVector::m_triplexMask;
	//zAxis = zAxis.Normalize();
	//
	//ndVector xAxis(normUp.CrossProduct(zAxis) & ndVector::m_triplexMask);
	//xAxis = xAxis.Normalize();
	//ndVector YAxis (zAxis.CrossProduct(xAxis) & ndVector::m_triplexMask);
	//
	//result[0] = xAxis;
	//result[1] = YAxis;
	//result[2] = zAxis;
	//result[3] = ndVector::m_wOne;
	//result = result.Transpose3x3();
	//
	//ndVector negEye (eyepoint);
	//negEye = negEye.Scale(ndFloat32 (-1.0f));
	//negEye[3] = ndFloat32 (1.0f);
	//result[3] = result.TransformVector(negEye);

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_front = ((eyepoint - eyepointTarget) & ndVector::m_triplexMask).Normalize();
	matrix.m_right = (matrix.m_front.CrossProduct(normUp) & ndVector::m_triplexMask).Normalize();
	matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	matrix.m_posit = (eyepoint & ndVector::m_triplexMask) | ndVector::m_wOne;
	return m_worldToOpenGl * matrix;
}

ndMatrix ndDemoCamera::CreatePerspectiveMatrix(ndFloat32 fov, ndFloat32 aspect, ndFloat32 front, ndFloat32 back)
{
	fov = ndClamp (fov, ndFloat32 (0.0f), ndPi);
	ndFloat32 y = front * ndTan(fov * ndFloat32(0.5f));
	ndFloat32 x = y * aspect;
	ndMatrix projectionMatrix (CreateMatrixFromFrustum(-x, x, -y, y, front, back));
	return projectionMatrix;
}

void ndDemoCamera::SetViewMatrix(ndInt32 width, ndInt32 height)
{
	// set the view port for this render section
	m_viewport[0] = 0;
	m_viewport[1] = 0;
	m_viewport[2] = width;
	m_viewport[3] = height;

	// calculate projection matrix
	m_projectionMatrix = CreatePerspectiveMatrix(m_fov, GLfloat(width) / GLfloat(height), m_frontPlane, m_backPlane);
	m_invProjectionMatrix = m_projectionMatrix.Inverse4x4();

	// set the model view matrix 
	const ndVector pointOfInterest(m_matrix.m_posit + m_matrix.m_front);
	//m_viewMatrix = CreateLookAtMatrix(m_matrix.m_posit, pointOfInterest, m_matrix.m_up);
	m_viewMatrix____ = CreateLookAtMatrix(m_matrix.m_posit, pointOfInterest, m_matrix.m_up);
	m_invViewMatrix = m_viewMatrix____.OrthoInverse();

	m_viewMatrix = m_invViewMatrix;

	//auto Test_XXXX1 = [this](ndVector xxxx)
	//{
	//	ndVector xxxx1(m_invViewMatrix.TransformVector1x4(xxxx));
	//	xxxx1 = m_projectionMatrix.TransformVector1x4(xxxx1);
	//	xxxx1.m_x /= xxxx1.m_w;
	//	xxxx1.m_y /= xxxx1.m_w;
	//	xxxx1.m_z /= xxxx1.m_w;
	//	return xxxx1;
	//};
	//auto Test_XXXX0 = [this](ndVector xxxx)
	//{
	//	ndVector xxxx1(xxxx.Scale (xxxx.m_w));
	//	xxxx1.m_w = xxxx.m_w;
	//	xxxx1 = m_invProjectionMatrix.TransformVector1x4(xxxx1);
	//	xxxx1 = m_viewMatrix____.TransformVector1x4(xxxx1);
	//	return xxxx1;
	//};
	//
	//ndVector xxxx1(Test_XXXX1(ndVector(m_frontPlane, m_frontPlane * 0.577f, m_frontPlane * 1.026f, 1.0f)));
	//ndVector xxxx2(Test_XXXX1(ndVector(m_backPlane, m_backPlane * 0.577f, m_backPlane * 1.026f, 1.0f)));
	//
	//ndVector xxxx3(Test_XXXX0(xxxx1));
	//ndVector xxxx4(Test_XXXX0(xxxx2));
	//ndVector xxxx5(Test_XXXX0(xxxx1));

	auto CalcuateFrustumPoint = [this](const ndVector& pointInClickSpace, ndFloat32 zdist)
	{
		ndVector point(pointInClickSpace.Scale (zdist));
		point.m_w = zdist;
		point = m_invProjectionMatrix.TransformVector1x4(point);
		//point = m_viewMatrix____.TransformVector1x4(point);
		return point;
	};

	for (ndInt32 j = 0; j < 2; ++j)
	{
		for (ndInt32 i = 0; i < 2; ++i)
		{
			const ndVector sp(ndFloat32(i * 2 - 1), ndFloat32(j * 2 - 1), ndFloat32(-1.0f), ndFloat32(1.0f));
			const ndVector viewPoint(m_invProjectionMatrix.TransformVector1x4(sp));
			m_frutrum[4 * 0 + 2 * j + i] = viewPoint.Scale(1.0f / viewPoint.m_w);

			m_frustum[2 * j + i] = CalcuateFrustumPoint(sp, m_frontPlane);
		}
	}

	for (ndInt32 j = 0; j < 2; ++j)
	{
		for (ndInt32 i = 0; i < 2; ++i)
		{
			const ndVector sp(ndFloat32(i * 2 - 1), ndFloat32(j * 2 - 1), ndFloat32(1.0f), ndFloat32(1.0f));
			const ndVector viewPoint(m_invProjectionMatrix.TransformVector1x4(sp));
			m_frutrum[4 * 1 + 2 * j + i] = viewPoint.Scale(1.0f / viewPoint.m_w);

			m_frustum[4 + 2 * j + i] = CalcuateFrustumPoint(sp, m_backPlane);
		}
	}
}

ndVector ndDemoCamera::ScreenToWorld (const ndVector& screenPoint) const
{
	ndVector sp(screenPoint);
	sp.m_y = (ndFloat32)m_viewport[3] - sp.m_y;

	sp.m_x = ndFloat32(2.0f) * (sp.m_x - (ndFloat32)m_viewport[0]) / (ndFloat32)m_viewport[2] - ndFloat32(1.0f);
	sp.m_y = ndFloat32(2.0f) * (sp.m_y - (ndFloat32)m_viewport[1]) / (ndFloat32)m_viewport[3] - ndFloat32(1.0f);
	sp.m_z = ndFloat32(2.0f) * sp.m_z  - ndFloat32(1.0f);
	sp.m_w = ndFloat32(1.0f);
	
	//sp = viewPoint * ViewMatrx * projeMatrix;
	ndVector viewPoint(m_viewMatrix.OrthoInverse().TransformVector1x4(m_invProjectionMatrix.TransformVector1x4(sp)));
	if (viewPoint.m_w != 0.0)
	{
		viewPoint = viewPoint.Scale(1.0f / viewPoint.m_w);
	}
	return viewPoint;
}

//ndVector ndDemoCamera::WorldToScreen (const ndVector& worldPoint) const
ndVector ndDemoCamera::WorldToScreen(const ndVector&) const
{
ndAssert (0);

/*
	ndInt32 win[4]; 
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	//get a point on the display array of the windows
	GLUI_Master.get_viewport_area(&win[0], &win[1], &win[2], &win[3]);
	viewport[0] = GLint (win[0]);
	viewport[1] = GLint (win[1]);
	viewport[2] = GLint (win[2]);
	viewport[3] = GLint (win[3]);


	//Where the 16 doubles of the matrix are to be stored
	GLdouble modelview[16]; 

	//Retrieve the matrix
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview); 

	GLdouble projection[16]; 
	glGetDoublev(GL_PROJECTION_MATRIX, projection);

	GLdouble winX;
	GLdouble winY;
	GLdouble winZ; //The coordinates to pass in

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	GLdouble objx = world.m_x;
	GLdouble objy = world.m_y;
	GLdouble objz = world.m_z;

	// use the real GL view port
	glGetIntegerv(GL_VIEWPORT, viewport); 
	gluProject (objx, objy, objz, modelview, projection, viewport, &winX, &winY, &winZ);

	winY = (ndFloat32)viewport[3] - winY; 

	return ndVector (ndFloat32(winX), ndFloat32 (winY), ndFloat32(winZ), 0.0f);
*/
	return ndVector::m_zero;
}

void ndDemoCamera::SetMatrix (const ndQuaternion& rotation, const ndVector& position)
{
	ndMatrix matrix (ndCalculateMatrix(rotation, position));
	m_cameraPitch = ndAsin (matrix.m_front.m_y);
	m_cameraYaw = ndAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);

	ndDemoEntity::SetMatrix (rotation, position);
}



