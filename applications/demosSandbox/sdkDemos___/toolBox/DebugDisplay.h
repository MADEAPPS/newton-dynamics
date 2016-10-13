/* Copyright (c) <2009> <Newton Game Dynamics>
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

#include <toolbox_stdafx.h>
#include <dVector.h>
#include <dMatrix.h>


enum DEBUG_DRAW_MODE
{
	m_lines,
	m_solid,
};

int DebugDisplayOn();
void SetDebugDisplayMode(int state);

void RenderAABB (NewtonWorld* const world);
void RenderCenterOfMass (NewtonWorld* const world);
void RenderNormalForces (NewtonWorld* const world);
void RenderContactPoints (NewtonWorld* const world); 
void RenderJointsDebugInfo (NewtonWorld* const world, dFloat size);

void DebugShowSoftBodySpecialCollision (void* userData, int vertexCount, const dFloat* const faceVertec, int faceId);

void DebugRenderWorldCollision (const NewtonWorld* const world, DEBUG_DRAW_MODE mode);

void DebugDrawPoint (const dVector& p0, dFloat size);
void DebugDrawLine (const dVector& p0, const dVector& p1);
void DebugDrawCollision (const NewtonCollision* const collision, const dMatrix& matrix, DEBUG_DRAW_MODE mode);

void ClearDebugDisplay(NewtonWorld* const world);
void ShowMeshCollidingFaces (const NewtonBody* const staticCollisionBody, const NewtonBody* const body, int faceID, int vertexCount, const dFloat* const vertex, int vertexstrideInBytes);

#endif
