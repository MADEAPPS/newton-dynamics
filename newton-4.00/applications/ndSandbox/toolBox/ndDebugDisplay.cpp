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

// dGeometry.cpp: implementation of the dGeometry class.
//
//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsUtils.h"

#if 0
static int g_debugMode = 0;


int DebugDisplayOn()
{
	return g_debugMode;
}

void SetDebugDisplayMode(int state)
{
	g_debugMode = state;
}

#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	static int g_debugDisplayCount;
	static dVector g_debugDisplayCallback[1024 * 4];
#endif

static void RenderBodyContactsAndTangentDiretions (NewtonBody* const body, dFloat32 length)
{
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
		if (NewtonJointIsActive (joint)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point(0.0f);
				dVector normal(0.0f);	
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

				dVector tangentDir0(0.0f);
				dVector tangentDir1(0.0f);
				NewtonMaterialGetContactTangentDirections(material, body, &tangentDir0[0], &tangentDir1[0]);

				// if we are display debug info we need to block other threads from writing the data at the same time
				dVector p1 (point + normal.Scale (length));
				dVector p0 (point);
				glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
				glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
			}
		}
	}
}

static void RenderBodyContactsForces (NewtonBody* const body, dFloat32 scale)
{
	dFloat32 mass;
	dFloat32 Ixx;
	dFloat32 Iyy;
	dFloat32 Izz;
	NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);	

	//draw normal forces in term of acceleration.
	//this  mean that two bodies with same shape but different mass will display the same force
	if (mass > 0.0f) {
		scale = scale/mass;
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
			if (NewtonJointIsActive (joint)) {
				for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
					dVector point(0.0f);
					dVector normal(0.0f);	
					dVector tangnetDir0(0.0f);
					dVector tangnetDir1(0.0f);
					dVector contactForce(0.0f);	
					NewtonMaterial* const material = NewtonContactGetMaterial (contact);

					NewtonMaterialGetContactForce(material, body, &contactForce.m_x);
					NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);
					dVector normalforce (normal.Scale (contactForce.DotProduct3(normal)));
					dVector p0 (point);
					dVector p1 (point + normalforce.Scale (scale));
					glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
					glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));


					// these are the components of the tangents forces at the contact point, the can be display at the contact position point.
					NewtonMaterialGetContactTangentDirections(material, body, &tangnetDir0[0], &tangnetDir1[0]);
					dVector tangentForce1 (tangnetDir0.Scale ((contactForce.DotProduct3(tangnetDir0)) * scale));
					dVector tangentForce2 (tangnetDir1.Scale ((contactForce.DotProduct3(tangnetDir1)) * scale));

					p1 = point + tangentForce1.Scale (scale);
					glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
					glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

					p1 = point + tangentForce2.Scale (scale);
					glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
					glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

				}
			}
		}
	}
}

//void NewtonMaterialGetContactForce(const NewtonMaterial* const materialHandle, NewtonBody* const body, dFloat32* const forcePtr)

void RenderAABB (NewtonWorld* const world)
{
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.0f, 1.0f);

	glBegin(GL_LINES);
//glVertex3f (-20.3125000f, 3.54991579f, 34.3441200f);
//glVertex3f (-19.6875000f, 3.54257250f, 35.2211456f);

	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		dMatrix matrix;
		dVector p0(0.0f); 
		dVector p1(0.0f); 
		
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		NewtonBodyGetMatrix (body, &matrix[0][0]);
		NewtonCollisionCalculateAABB (collision, &matrix[0][0], &p0[0], &p1[0]);
//		CalculateAABB (collision, matrix, p0, p1);

		glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

		glVertex3f(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
		glVertex3f(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

		glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
		glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	}
	glEnd();
}

void RenderCenterOfMass (NewtonWorld* const world)
{
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.0f, 1.0f);

	glBegin(GL_LINES);
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		dMatrix matrix;
		dVector com(0.0f); 
		
		NewtonBodyGetCentreOfMass (body, &com[0]);
		NewtonBodyGetMatrix (body, &matrix[0][0]);

		dVector o (matrix.TransformVector (com));

		dVector x (o + matrix.RotateVector (dVector (1.0f, 0.0f, 0.0f, 0.0f)));
		glColor3f (1.0f, 0.0f, 0.0f);
		glVertex3f(GLfloat(o.m_x), GLfloat(o.m_y), GLfloat(o.m_z));
		glVertex3f(GLfloat(x.m_x), GLfloat(x.m_y), GLfloat(x.m_z));

 		dVector y (o + matrix.RotateVector (dVector (0.0f, 1.0f, 0.0f, 0.0f)));
		glColor3f (0.0f, 1.0f, 0.0f);
		glVertex3f(GLfloat(o.m_x), GLfloat(o.m_y), GLfloat(o.m_z));
		glVertex3f(GLfloat(y.m_x), GLfloat(y.m_y), GLfloat(y.m_z));

		dVector z (o + matrix.RotateVector (dVector (0.0f, 0.0f, 1.0f, 0.0f)));
		glColor3f (0.0f, 0.0f, 1.0f);
		glVertex3f(GLfloat(o.m_x), GLfloat(o.m_y), GLfloat(o.m_z));
		glVertex3f(GLfloat(z.m_x), GLfloat(z.m_y), GLfloat(z.m_z));
	}
	glEnd();
}

void RenderBodyFrame(NewtonWorld* const world)
{
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.0f, 1.0f);

	glBegin(GL_LINES);
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		dMatrix matrix;

		NewtonBodyGetMatrix(body, &matrix[0][0]);

		dVector o(matrix.m_posit);

		dVector x(o + matrix.RotateVector(dVector(1.1f, 0.0f, 0.0f, 0.0f)));
		glColor3f(0.5f, 0.0f, 0.0f);
		glVertex3f(GLfloat(o.m_x), GLfloat(o.m_y), GLfloat(o.m_z));
		glVertex3f(GLfloat(x.m_x), GLfloat(x.m_y), GLfloat(x.m_z));

		dVector y(o + matrix.RotateVector(dVector(0.0f, 1.1f, 0.0f, 0.0f)));
		glColor3f(0.0f, 0.5f, 0.0f);
		glVertex3f(GLfloat(o.m_x), GLfloat(o.m_y), GLfloat(o.m_z));
		glVertex3f(GLfloat(y.m_x), GLfloat(y.m_y), GLfloat(y.m_z));

		dVector z(o + matrix.RotateVector(dVector(0.0f, 0.0f, 1.1f, 0.0f)));
		glColor3f(0.0f, 0.0f, 0.5f);
		glVertex3f(GLfloat(o.m_x), GLfloat(o.m_y), GLfloat(o.m_z));
		glVertex3f(GLfloat(z.m_x), GLfloat(z.m_y), GLfloat(z.m_z));
	}
	glEnd();
}

void RenderContactPoints (NewtonWorld* const world)
{
	glDisable(GL_TEXTURE_2D);

	glPointSize(8.0f);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
			if (NewtonJointIsActive (joint)) {
				for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
					dVector point(0.0f);
					dVector normal(0.0f);	
					NewtonMaterial* const material = NewtonContactGetMaterial (contact);
					NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

					// if we are display debug info we need to block other threads from writing the data at the same time
					glVertex3f (GLfloat(point.m_x), GLfloat(point.m_y), GLfloat(point.m_z));
				}
			}
		}
	}
	glEnd();
	glPointSize(1.0f);
}

void RenderRayCastHit(NewtonWorld* const world)
{
	dVector point;
	dVector normal;
	if (GetLastHit (point, normal)) 
	{
		glDisable(GL_TEXTURE_2D);

		glPointSize(8.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_POINTS);
		// if we are display debug info we need to block other threads from writing the data at the same time
		glVertex3f(GLfloat(point.m_x), GLfloat(point.m_y), GLfloat(point.m_z));
		glEnd();
		glPointSize(1.0f);
	}
}

void RenderNormalForces (NewtonWorld* const world)
{
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.5f, 1.0f);
	glBegin(GL_LINES);

	dFloat32 length = 1.0f / 200.0f;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		RenderBodyContactsForces (body, length);
	}
	glEnd();
}


void DebugShowSoftBodySpecialCollision (void* userData, int vertexCount, const dFloat32* const faceVertex, int clusterIndex)
{
	static dVector color[] = {dVector(1.0f, 0.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 0.0f, 0.0f), dVector(0.0f, 0.0f, 1.0f, 0.0f), 
							  dVector(1.0f, 1.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 1.0f, 0.0f), dVector(1.0f, 0.0f, 1.0f, 0.0f),
							  dVector(0.5f, 0.5f, 0.0f, 0.0f), dVector(0.0f, 0.5f, 0.5f, 0.0f), dVector(0.5f, 0.0f, 0.5f, 0.0f)};
		
	int index = clusterIndex % (sizeof (color) / sizeof (color[0]));
	glColor3f(GLfloat(color[index].m_x), GLfloat(color[index].m_y), GLfloat(color[index].m_z));
	DebugShowGeometryCollision (userData, vertexCount, faceVertex, clusterIndex);
}

void DebugDrawPoint (const dVector& p, dFloat32 size)
{
	glPointSize(GLfloat(size));
	glBegin(GL_POINTS);
	glVertex3f(GLfloat(p.m_x), GLfloat(p.m_y), GLfloat(p.m_z));
	glEnd();
	glPointSize(1.0f);
}

void DebugDrawLine (const dVector& p0, const dVector& p1)
{
	glBegin(GL_LINES);
	glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
	glEnd();
}

void DebugDrawCollision (const NewtonCollision* const collision, const dMatrix& matrix, dDebugDisplayMode mode)
{
//	glBegin(GL_LINES);
	NewtonCollisionForEachPolygonDo (collision, &matrix[0][0], DebugShowGeometryCollision, (void*)mode);
//	glEnd();
}


void ClearDebugDisplay( NewtonWorld* const world )
{
	g_debugDisplayCount = 0;
}

void ShowMeshCollidingFaces (const NewtonBody* const staticCollisionBody, const NewtonBody* const body, int faceID, int vertexCount, const dFloat32* const vertex, int vertexstrideInBytes)
{
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	if (g_debugMode) {
		if ((g_debugDisplayCount + vertexCount * 2) < int (sizeof (g_debugDisplayCallback) / sizeof(g_debugDisplayCallback[0]))) {
			// we are coping data to and array of memory, another call back may be doing the same thing
			// here fore we need to avoid race conditions
			NewtonWorldCriticalSectionLock (NewtonBodyGetWorld (staticCollisionBody), 0);

			int stride = vertexstrideInBytes / sizeof (dFloat32);
			dVector l0 (vertex[(vertexCount-1) * stride + 0], vertex[(vertexCount-1) * stride + 1], vertex[(vertexCount-1) * stride + 2], 0.0f);
			for (int j = 0; j < vertexCount; j ++) {
				dVector l1 (vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2], 0.0f);
				g_debugDisplayCallback[g_debugDisplayCount + 0] = l0;
				g_debugDisplayCallback[g_debugDisplayCount + 1] = l1;
				g_debugDisplayCount  += 2;
				l0 = l1;
			}
			// unlock the critical section
			NewtonWorldCriticalSectionUnlock (NewtonBodyGetWorld (staticCollisionBody));
		}
	}
#endif
}

void RenderJointsDebugInfo (NewtonWorld* const world, dJointDebugDisplay* const jointDebug)
{
	// this will go over the joint list twice, 
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
			dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
			if (customJoint) {
				customJoint->Debug(jointDebug);
			}
		}
	}
	
}

void RenderListenersDebugInfo (NewtonWorld* const world, dJointDebugDisplay* const jointDebug)
{
	NewtonWorldListenerDebug(world, jointDebug);
}

#endif
