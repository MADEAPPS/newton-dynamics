/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef AFX_DEBUG_DISPLAY_H___H
#define AFX_DEBUG_DISPLAY_H___H

#include "toolbox_stdafx.h"
#include <dVector.h>
#include <dMatrix.h>


enum DEBUG_DRAW_MODE
{
	m_lines,
	m_solid,
};

class dJointDebugDisplay: public dCustomJoint::dDebugDisplay
{
	public:
	dJointDebugDisplay(const dMatrix& cameraMatrix)
		:dCustomJoint::dDebugDisplay(cameraMatrix)
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glColor3f(1.0f, 1.0f, 1.0f);

		GLint viewport[4];
		glGetIntegerv (GL_VIEWPORT, viewport);
		m_width = viewport[2]; 
		m_height = viewport[3]; 
		glBegin(GL_LINES);
	}

	~dJointDebugDisplay()
	{
		glEnd();
		glEnable(GL_LIGHTING);
		glEnable(GL_TEXTURE_2D);
	}

	void SetColor(const dVector& color)
	{
		glColor3f(GLfloat (color.m_x), GLfloat (color.m_y), GLfloat (color.m_z));
	}

	void DrawLine(const dVector& p0, const dVector& p1)
	{
		glVertex3f(GLfloat (p0.m_x), GLfloat (p0.m_y), GLfloat (p0.m_z));
		glVertex3f(GLfloat (p1.m_x), GLfloat (p1.m_y), GLfloat (p1.m_z));
	}

	virtual void SetOrthRendering() 
	{
		glEnd();
		glPushMatrix();
		glMatrixMode(GL_PROJECTION);

		glPushMatrix();
		glLoadIdentity();
		glOrtho(0.0f, m_width, m_height, 0.0f, 0.0f, 1.0f);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		float x = 0.0f;
		float y = 0.0f;
		glTranslated(x, y, 0);

		glBegin(GL_LINES);
	}

	virtual void ResetOrthRendering() 
	{
		glEnd();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
		glLoadIdentity();

		glBegin(GL_LINES);
	}
};


int DebugDisplayOn();
void SetDebugDisplayMode(int state);

void RenderAABB (NewtonWorld* const world);
void RenderCenterOfMass (NewtonWorld* const world);
void RenderBodyFrame (NewtonWorld* const world);
void RenderRayCastHit(NewtonWorld* const world);
void RenderNormalForces (NewtonWorld* const world);
void RenderContactPoints (NewtonWorld* const world); 
void RenderJointsDebugInfo (NewtonWorld* const world, dJointDebugDisplay* const jointDebug);

void DebugShowSoftBodySpecialCollision (void* userData, int vertexCount, const dFloat* const faceVertec, int faceId);

void DebugRenderWorldCollision (const NewtonWorld* const world, DEBUG_DRAW_MODE mode);

void DebugDrawPoint (const dVector& p0, dFloat size);
void DebugDrawLine (const dVector& p0, const dVector& p1);
void DebugDrawCollision (const NewtonCollision* const collision, const dMatrix& matrix, DEBUG_DRAW_MODE mode);

void ClearDebugDisplay(NewtonWorld* const world);
void ShowMeshCollidingFaces (const NewtonBody* const staticCollisionBody, const NewtonBody* const body, int faceID, int vertexCount, const dFloat* const vertex, int vertexstrideInBytes);

#endif
