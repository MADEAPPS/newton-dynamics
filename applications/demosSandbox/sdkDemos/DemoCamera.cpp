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



// RenderPrimitive.cpp: implementation of the RenderPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include "toolbox_stdafx.h"
#include "DemoCamera.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MOUSE_PICK_DAMP			 10.0f
#define MOUSE_PICK_STIFFNESS	 100.0f


DemoCamera::DemoCamera()
	:DemoEntity (dGetIdentityMatrix(), NULL) 
	,m_fov (60.0f * dDegreeToRad)
	,m_backPlane(2000.0f)
	,m_frontPlane (0.01f)
	,m_cameraYaw(0.0f)
	,m_cameraPitch(0.0f)
{
}

	
DemoCamera::~DemoCamera()
{
}

void DemoCamera::Render(dFloat timeStep, DemoEntityManager* const scene) const
{
}

dFloat DemoCamera::GetYawAngle() const
{
	return m_cameraYaw;
}

dFloat DemoCamera::GetPichAngle() const
{
	return m_cameraPitch;
}

void DemoCamera::SetViewMatrix(int width, int height)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint) width, (GLint) height);

	// set the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//m_backPlane = 10000.0f;
	gluPerspective(m_fov * 180.0f / dPi, GLfloat (width) /GLfloat(height), m_frontPlane, m_backPlane);

	// set the model view matrix 
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	dVector pointOfInterest (m_matrix.m_posit + m_matrix.m_front);
	gluLookAt(m_matrix.m_posit.m_x, m_matrix.m_posit.m_y, m_matrix.m_posit.m_z, 
		      pointOfInterest.m_x, pointOfInterest.m_y, pointOfInterest.m_z, 
			  m_matrix.m_up.m_x, m_matrix.m_up.m_y, m_matrix.m_up.m_z);	


	glGetIntegerv(GL_VIEWPORT, m_viewport); 
	glGetDoublev(GL_MODELVIEW_MATRIX, m_modelViewMatrix); 
	glGetDoublev(GL_PROJECTION_MATRIX, m_projectionViewMatrix); 
}

dVector DemoCamera::ScreenToWorld (const dVector& screenPoint) const
{
	GLdouble winX = screenPoint.m_x; //Store the x cord;
	GLdouble winY = screenPoint.m_y; //Store the y cord
	GLdouble winZ = screenPoint.m_z; //Store the Z cord

	//Now windows coordinates start with (0,0) being at the top left 
	//whereas OpenGL cords start lower left so we have to do this to convert it: 
	//Remember we got viewport value before 
	winY = (dFloat)m_viewport[3] - winY; 

	GLdouble objx;
	GLdouble objy;
	GLdouble objz;
	gluUnProject (winX, winY, winZ, m_modelViewMatrix, m_projectionViewMatrix, (GLint*)&m_viewport, &objx, &objy, &objz);

	return dVector (dFloat(objx), dFloat(objy), dFloat(objz));
}

dVector DemoCamera::WorldToScreen (const dVector& worldPoint) const
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

	winY = (dFloat)viewport[3] - winY; 

	return dVector (dFloat(winX), dFloat (winY), dFloat(winZ), 0.0f);
*/
	return dVector (0.0f);
}



void DemoCamera::SetMatrix (DemoEntityManager& scene, const dQuaternion& rotation, const dVector& position)
{
	dMatrix matrix (rotation, position);
	m_cameraPitch = dAsin (matrix.m_front.m_y);
	m_cameraYaw = dAtan2 (-matrix.m_front.m_z, matrix.m_front.m_x);

	DemoEntity::SetMatrix (scene, rotation, position);
}



