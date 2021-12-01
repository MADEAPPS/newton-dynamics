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


#ifndef __DEBUG_DISPLAY_H__
#define __DEBUG_DISPLAY_H__

#include "ndSandboxStdafx.h"
#include <ndVector.h>
#include <ndMatrix.h>

class ndDemoEntityManager;

#if 0
void RenderBodyFrame (NewtonWorld* const world);
void RenderRayCastHit(NewtonWorld* const world);
void RenderNormalForces (NewtonWorld* const world);

void RenderListenersDebugInfo (NewtonWorld* const world, dJointDebugDisplay* const jointDebug);

void DebugShowSoftBodySpecialCollision (void* userData, dInt32 vertexCount, const dFloat32* const faceVertec, dInt32 faceId);


void DebugDrawPoint (const dVector& p0, dFloat32 size);
void DebugDrawLine (const dVector& p0, const dVector& p1);
void DebugDrawCollision (const NewtonCollision* const collision, const dMatrix& matrix, dDebugDisplayMode mode);

void ClearDebugDisplay(NewtonWorld* const world);
void ShowMeshCollidingFaces (const NewtonBody* const staticCollisionBody, const NewtonBody* const body, dInt32 faceID, dInt32 vertexCount, const dFloat32* const vertex, dInt32 vertexstrideInBytes);
#endif


void RenderParticles(ndDemoEntityManager* const scene);
void RenderWorldScene(ndDemoEntityManager* const scene);
void RenderBodiesAABB(ndDemoEntityManager* const scene);
void RenderBodyFrame(ndDemoEntityManager* const scene);
void RenderCenterOfMass(ndDemoEntityManager* const scene);
void RenderContactPoints(ndDemoEntityManager* const scene);
void RenderJointsDebugInfo(ndDemoEntityManager* const scene);
void RenderModelsDebugInfo(ndDemoEntityManager* const scene);

#endif

