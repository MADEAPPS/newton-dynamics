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

#include "ndSandboxStdafx.h"
#include "ndDemoCamera.h"

#define MOUSE_PICK_DAMP			10.0f
#define MOUSE_PICK_STIFFNESS	100.0f

#define D_CAMERA_ANGLE			60.0f

ndDemoCamera::ndDemoCamera()
	:ndDemoEntity (dGetIdentityMatrix(), nullptr) 
	,m_viewMatrix(dGetIdentityMatrix())
	,m_projectionMatrix(dGetIdentityMatrix())
	,m_fov(D_CAMERA_ANGLE * ndDegreeToRad)
	,m_backPlane(2000.0f)
	,m_frontPlane (0.01f)
	,m_cameraYaw(0.0f)
	,m_cameraPitch(0.0f)
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

ndMatrix ndDemoCamera::CreateMatrixFromFrustum(ndFloat32 Left, ndFloat32 Right, ndFloat32 Bottom, ndFloat32 Top, ndFloat32 ZNear, ndFloat32 ZFar)
{
	ndMatrix Result(dGetIdentityMatrix());

	Result[0][0] = 2 * ZNear / (Right - Left);
	Result[0][1] = 0;
	Result[0][2] = 0;
	Result[0][3] = 0;

	Result[1][0] = 0;
	Result[1][1] = 2 * ZNear / (Top - Bottom);
	Result[1][2] = 0;
	Result[1][3] = 0;

	Result[2][0] = (Right + Left) / (Right - Left);
	Result[2][1] = (Top + Bottom) / (Top - Bottom);
	Result[2][2] = -(ZFar + ZNear) / (ZFar - ZNear);
	Result[2][3] = -1;

	Result[3][0] = 0;
	Result[3][1] = 0;
	Result[3][2] = -2 * ZFar * ZNear / (ZFar - ZNear);
	Result[3][3] = 0;

	return Result;
}

ndMatrix ndDemoCamera::CreateLookAtMatrix(const ndVector& eye, const ndVector& center, const ndVector& normUp)
{
	ndMatrix Result(dGetIdentityMatrix());
	
	ndVector ZAxis (center - eye);
	ZAxis = ZAxis & ndVector::m_triplexMask;

	ZAxis = ZAxis.Normalize();
	ndVector XAxis (ZAxis.CrossProduct(normUp));

	XAxis = XAxis.Normalize();
	ndVector YAxis (XAxis.CrossProduct(ZAxis));

	Result[0] = XAxis;
	Result[1] = YAxis;
	Result[2] = ZAxis;

	Result[2] = Result[2].Scale(-1.0f);

	Result[3] = ndVector::m_wOne;

	Result = Result.Transpose();

	ndVector negEye (eye);
	negEye = negEye.Scale(-1.0f);
	negEye[3] = 1.0f;

	negEye = Result.TransformVector(negEye);

	Result[3] = negEye;

	return Result;
}

ndMatrix ndDemoCamera::CreatePerspectiveMatrix(ndFloat32 fov, ndFloat32 Aspect, ndFloat32 ZNear, ndFloat32 ZFar)
{
	fov = dClamp (fov, ndFloat32 (0.0f), ndPi);
	//y = ZNear * (ndFloat32)ndTan((fov * cPIdiv180) * 0.5f);
	ndFloat32 y = ZNear * ndTan(fov * 0.5f);
	ndFloat32 x = y * Aspect;
	ndMatrix Result (CreateMatrixFromFrustum(-x, x, -y, y, ZNear, ZFar));
	return Result;
}

void ndDemoCamera::SetViewMatrix(ndInt32 width, ndInt32 height)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint)width, (GLint)height);
	glGetIntegerv(GL_VIEWPORT, m_viewport);

	// calculate projection matrix
	m_projectionMatrix = CreatePerspectiveMatrix(m_fov, GLfloat(width) / GLfloat(height), m_frontPlane, m_backPlane);

	// set the model view matrix 
	ndVector pointOfInterest(m_matrix.m_posit + m_matrix.m_front);
	m_viewMatrix = CreateLookAtMatrix(
		ndVector(m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z, 1.0f),
		ndVector(pointOfInterest.m_x, pointOfInterest.m_y, pointOfInterest.m_z, 1.0f),
		ndVector(m_matrix.m_up.m_x, m_matrix.m_up.m_y, m_matrix.m_up.m_z, 0.0f));
}

ndVector ndDemoCamera::ScreenToWorld (const ndVector& screenPoint) const
{
	GLdouble winX = screenPoint.m_x; //Store the x cord;
	GLdouble winY = screenPoint.m_y; //Store the y cord
	GLdouble winZ = screenPoint.m_z; //Store the Z cord

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	winY = (ndFloat32)m_viewport[3] - winY; 

	GLdouble objx;
	GLdouble objy;
	GLdouble objz;
	GLdouble modelViewMatrix[16];
	GLdouble projectionViewMatrix[16];
	for (ndInt32 i = 0; i < 4; i++) 
	{
		for (ndInt32 j = 0; j < 4; j++)
		{
			modelViewMatrix[i * 4 + j] = m_viewMatrix[i][j];
			projectionViewMatrix[i * 4 + j] = m_projectionMatrix[i][j];
		}
	}

	gluUnProject (winX, winY, winZ, modelViewMatrix, projectionViewMatrix, (GLint*)&m_viewport, &objx, &objy, &objz);
	return ndVector (ndFloat32(objx), ndFloat32(objy), ndFloat32(objz), ndFloat32 (1.0f));
}

//ndVector ndDemoCamera::WorldToScreen (const ndVector& worldPoint) const
ndVector ndDemoCamera::WorldToScreen(const ndVector&) const
{
dAssert (0);

/*
	ndInt32 win[4]; 
	GLint viewport[4]; 

	//Retrieves the viewport and stores it in the variable
	// get a point on the display arai of the windows
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
	ndMatrix matrix (rotation, position);
	m_cameraPitch = ndAsin (matrix.m_front.m_y);
	m_cameraYaw = ndAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);

	ndDemoEntity::SetMatrix (rotation, position);
}



