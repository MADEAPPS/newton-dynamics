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
#include "toolbox_stdafx.h"
#include "DebugDisplay.h"
#include "PhysicsUtils.h"

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

static void RenderBodyContactsAndTangentDiretions (NewtonBody* const body, dFloat length)
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

static void RenderBodyContactsForces (NewtonBody* const body, dFloat scale)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
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

//void NewtonMaterialGetContactForce(const NewtonMaterial* const materialHandle, NewtonBody* const body, dFloat* const forcePtr)

void RenderAABB (NewtonWorld* const world)
{
	glDisable (GL_LIGHTING);
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
	glDisable (GL_LIGHTING);
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
	glDisable(GL_LIGHTING);
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
	glDisable (GL_LIGHTING);
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
	if (GetLastHit (point, normal)) { 

		glDisable(GL_LIGHTING);
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
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.5f, 1.0f);
	glBegin(GL_LINES);

	dFloat length = 1.0f / 50.0f;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		RenderBodyContactsForces (body, length);
	}
	glEnd();
}



void DebugShowGeometryCollision (void* userData, int vertexCount, const dFloat* const faceVertec, int id)
{
	//DEBUG_DRAW_MODE mode = (DEBUG_DRAW_MODE) ((int)userData); //NOTE error: cast from ‘void*’ to ‘int’ loses precision
	DEBUG_DRAW_MODE mode = (DEBUG_DRAW_MODE) ((intptr_t)userData);
	
	if (mode == m_lines) {
		int index = vertexCount - 1;
		dVector p0 (faceVertec[index * 3 + 0], faceVertec[index * 3 + 1], faceVertec[index * 3 + 2]);
		for (int i = 0; i < vertexCount; i ++) {
			dVector p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
			glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
			glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
			p0 = p1;
		}
	} else {
		dVector p0 (faceVertec[0 * 3 + 0], faceVertec[0 * 3 + 1], faceVertec[0 * 3 + 2]);
		dVector p1 (faceVertec[1 * 3 + 0], faceVertec[1 * 3 + 1], faceVertec[1 * 3 + 2]);
		dVector p2 (faceVertec[2 * 3 + 0], faceVertec[2 * 3 + 1], faceVertec[2 * 3 + 2]);

		dVector normal ((p1 - p0).CrossProduct(p2 - p0));
		normal = normal.Scale (1.0f / dSqrt (normal.DotProduct3(normal)));
		glNormal3f(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));
		for (int i = 2; i < vertexCount; i ++) {
			p2 = dVector (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
			glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
			glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
			glVertex3f(GLfloat(p2.m_x), GLfloat(p2.m_y), GLfloat(p2.m_z));
			p1 = p2;
		}
	}
}

void DebugShowSoftBodySpecialCollision (void* userData, int vertexCount, const dFloat* const faceVertec, int clusterIndex)
{
	static dVector color[] = {dVector(1.0f, 0.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 0.0f, 0.0f), dVector(0.0f, 0.0f, 1.0f, 0.0f), 
							  dVector(1.0f, 1.0f, 0.0f, 0.0f), dVector(0.0f, 1.0f, 1.0f, 0.0f), dVector(1.0f, 0.0f, 1.0f, 0.0f),
							  dVector(0.5f, 0.5f, 0.0f, 0.0f), dVector(0.0f, 0.5f, 0.5f, 0.0f), dVector(0.5f, 0.0f, 0.5f, 0.0f)};
		
	int index = clusterIndex % (sizeof (color) / sizeof (color[0]));
	glColor3f(GLfloat(color[index].m_x), GLfloat(color[index].m_y), GLfloat(color[index].m_z));
	DebugShowGeometryCollision (userData, vertexCount, faceVertec, clusterIndex);
}


static void DebugShowBodyCollision (const NewtonBody* const body, DEBUG_DRAW_MODE mode)
{
	switch (NewtonBodyGetType(body)) 
	{
		case NEWTON_DYNAMIC_BODY:
		{
			int sleepState = NewtonBodyGetSleepState(body);
			if (sleepState == 1) {
				// indicate when body is sleeping 
				glColor3f(0.42f, 0.73f, 0.98f);
			} else {
				// body is active
				glColor3f(1.0f, 1.0f, 1.0f);
			}
			break;
		}

		case NEWTON_KINEMATIC_BODY:
			glColor3f(1.0f, 1.0f, 0.0f);
			break;
	}
	dMatrix matrix;
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonCollisionForEachPolygonDo (NewtonBodyGetCollision(body), &matrix[0][0], DebugShowGeometryCollision, (void*) mode);
}


void DebugRenderWorldCollision (const NewtonWorld* const world, DEBUG_DRAW_MODE mode)
{
	glDisable(GL_TEXTURE_2D);
	if (mode == m_lines) {
		glDisable (GL_LIGHTING);
		glBegin(GL_LINES);
	} else {
		glBegin(GL_TRIANGLES);
	}
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {

		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
		if (mass == 0.0f) {
//			continue;
		}

		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		int collisionType = NewtonCollisionGetType (collision);
		switch (collisionType) 
		{
			//SERIALIZE_ID_SPHERE:
			//SERIALIZE_ID_CAPSULE:
			//SERIALIZE_ID_CHAMFERCYLINDER:
			//SERIALIZE_ID_TAPEREDCAPSULE:
			//SERIALIZE_ID_CYLINDER:
			//SERIALIZE_ID_TAPEREDCYLINDER:
			//SERIALIZE_ID_BOX:
			//SERIALIZE_ID_CONE:
			//SERIALIZE_ID_CONVEXHULL:
			//SERIALIZE_ID_NULL:
			//SERIALIZE_ID_COMPOUND:
			//SERIALIZE_ID_CLOTH_PATCH:
			//SERIALIZE_ID_DEFORMABLE_SOLID:
//			case SERIALIZE_ID_TREE:
//			case SERIALIZE_ID_SCENE:
//			case SERIALIZE_ID_USERMESH:
//			case SERIALIZE_ID_HEIGHTFIELD:
//			case SERIALIZE_ID_COMPOUND_BREAKABLE:
			case 1000:
				break;
			default: 
				DebugShowBodyCollision (body, mode);
		}
	}
	glEnd();

	glDisable (GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 0.0f);
	for (int i = 0; i < g_debugDisplayCount; i += 2) {
		glVertex3f (GLfloat(g_debugDisplayCallback[i].m_x), GLfloat(g_debugDisplayCallback[i].m_y), GLfloat(g_debugDisplayCallback[i].m_z));
		glVertex3f (GLfloat(g_debugDisplayCallback[i+1].m_x), GLfloat(g_debugDisplayCallback[i+1].m_y), GLfloat(g_debugDisplayCallback[i+1].m_z));
	}
	glEnd();
}

void DebugDrawPoint (const dVector& p, dFloat size)
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

void DebugDrawCollision (const NewtonCollision* const collision, const dMatrix& matrix, DEBUG_DRAW_MODE mode)
{
//	glBegin(GL_LINES);
	NewtonCollisionForEachPolygonDo (collision, &matrix[0][0], DebugShowGeometryCollision, (void*)mode);
//	glEnd();
}


void ClearDebugDisplay( NewtonWorld* const world )
{
	g_debugDisplayCount = 0;
}

void ShowMeshCollidingFaces (const NewtonBody* const staticCollisionBody, const NewtonBody* const body, int faceID, int vertexCount, const dFloat* const vertex, int vertexstrideInBytes)
{
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	if (g_debugMode) {
		if ((g_debugDisplayCount + vertexCount * 2) < int (sizeof (g_debugDisplayCallback) / sizeof(g_debugDisplayCallback[0]))) {
			// we are coping data to and array of memory, another call back may be doing the same thing
			// here fore we need to avoid race conditions
			NewtonWorldCriticalSectionLock (NewtonBodyGetWorld (staticCollisionBody), 0);

			int stride = vertexstrideInBytes / sizeof (dFloat);
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