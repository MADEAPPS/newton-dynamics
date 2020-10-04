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


#include "ndSandboxStdafx.h"
#include "ndDemoCamera.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MOUSE_PICK_DAMP			10.0f
#define MOUSE_PICK_STIFFNESS	100.0f

#define D_CAMERA_ANGLE			60.0f

ndDemoCamera::ndDemoCamera()
	:ndDemoEntity (dGetIdentityMatrix(), nullptr) 
	,m_viewMatrix(dGetIdentityMatrix())
	,m_projectionMatrix(dGetIdentityMatrix())
	,m_fov(D_CAMERA_ANGLE * dDegreeToRad)
	,m_backPlane(2000.0f)
	,m_frontPlane (0.01f)
	,m_cameraYaw(0.0f)
	,m_cameraPitch(0.0f)
{
}
	
ndDemoCamera::~ndDemoCamera()
{
}

void ndDemoCamera::Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const
{
}

dFloat32 ndDemoCamera::GetYawAngle() const
{
	return m_cameraYaw;
}

dFloat32 ndDemoCamera::GetPichAngle() const
{
	return m_cameraPitch;
}

const dMatrix& ndDemoCamera::GetProjectionMatrix() const
{
	return m_projectionMatrix;
}

const dMatrix& ndDemoCamera::GetViewMatrix() const
{
	return m_viewMatrix;
}

dMatrix ndDemoCamera::CreateMatrixFromFrustum(dFloat32 Left, dFloat32 Right, dFloat32 Bottom, dFloat32 Top, dFloat32 ZNear, dFloat32 ZFar)
{
	dMatrix Result(dGetIdentityMatrix());

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

dMatrix ndDemoCamera::CreateLookAtMatrix(const dVector& eye, const dVector& center, const dVector& normUp)
{
	//dVector XAxis, YAxis, ZAxis, negEye;
	dMatrix Result(dGetIdentityMatrix());
	
	dVector ZAxis (center - eye);
	ZAxis = ZAxis & dVector::m_triplexMask;

	//NormalizeVector(ZAxis);
	ZAxis = ZAxis.Normalize();

	dVector XAxis (ZAxis.CrossProduct(normUp));

	//NormalizeVector(XAxis);
	XAxis = XAxis.Normalize();

	dVector YAxis (XAxis.CrossProduct(ZAxis));

	Result[0] = XAxis;
	Result[1] = YAxis;
	Result[2] = ZAxis;

	Result[2] = Result[2].Scale(-1.0f);

	Result[3] = dVector::m_wOne;

	Result = Result.Transpose();

	dVector negEye (eye);
	negEye = negEye.Scale(-1.0f);
	negEye[3] = 1.0f;

	negEye = Result.TransformVector(negEye);

	Result[3] = negEye;

	return Result;
}

dMatrix ndDemoCamera::CreatePerspectiveMatrix(dFloat32 fov, dFloat32 Aspect, dFloat32 ZNear, dFloat32 ZFar)
{
	fov = dClamp (fov, 0.0f, dPi);
	//y = ZNear * (dFloat32)dTan((fov * cPIdiv180) * 0.5f);
	dFloat32 y = ZNear * dTan(fov * 0.5f);
	dFloat32 x = y * Aspect;
	dMatrix Result (CreateMatrixFromFrustum(-x, x, -y, y, ZNear, ZFar));
	return Result;
}

void ndDemoCamera::SetViewMatrix(dInt32 width, dInt32 height)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint)width, (GLint)height);
	glGetIntegerv(GL_VIEWPORT, m_viewport);

	// calculate projection matrix
	m_projectionMatrix = CreatePerspectiveMatrix(m_fov, GLfloat(width) / GLfloat(height), m_frontPlane, m_backPlane);

	// set the model view matrix 
	dVector pointOfInterest(m_matrix.m_posit + m_matrix.m_front);
	m_viewMatrix = CreateLookAtMatrix(
		dVector(m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z, 1.0f),
		dVector(pointOfInterest.m_x, pointOfInterest.m_y, pointOfInterest.m_z, 1.0f),
		dVector(m_matrix.m_up.m_x, m_matrix.m_up.m_y, m_matrix.m_up.m_z, 0.0f));
}

dVector ndDemoCamera::ScreenToWorld (const dVector& screenPoint) const
{
	GLdouble winX = screenPoint.m_x; //Store the x cord;
	GLdouble winY = screenPoint.m_y; //Store the y cord
	GLdouble winZ = screenPoint.m_z; //Store the Z cord

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	winY = (dFloat32)m_viewport[3] - winY; 

	GLdouble objx;
	GLdouble objy;
	GLdouble objz;
	GLdouble modelViewMatrix[16];
	GLdouble projectionViewMatrix[16];
	for (dInt32 i = 0; i < 4; i++) 
	{
		for (int j = 0; j < 4; j++)
		{
			modelViewMatrix[i * 4 + j] = m_viewMatrix[i][j];
			projectionViewMatrix[i * 4 + j] = m_projectionMatrix[i][j];
		}
	}

	gluUnProject (winX, winY, winZ, modelViewMatrix, projectionViewMatrix, (GLint*)&m_viewport, &objx, &objy, &objz);
	return dVector (dFloat32(objx), dFloat32(objy), dFloat32(objz), dFloat32 (1.0f));
}

dVector ndDemoCamera::WorldToScreen (const dVector& worldPoint) const
{
dAssert (0);

/*
	int win[4]; 
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

	winY = (dFloat32)viewport[3] - winY; 

	return dVector (dFloat32(winX), dFloat32 (winY), dFloat32(winZ), 0.0f);
*/
	return dVector::m_zero;
}



void ndDemoCamera::SetMatrix (ndDemoEntityManager& scene, const dQuaternion& rotation, const dVector& position)
{
	dMatrix matrix (rotation, position);
	m_cameraPitch = dAsin (matrix.m_front.m_y);
	m_cameraYaw = dAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);

	ndDemoEntity::SetMatrix (scene, rotation, position);
}



