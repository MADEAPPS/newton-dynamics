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

// dGeometry.cpp: implementation of the dGeometry class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include <DebugDisplay.h>


static int debugMode = 0;

int DebugDisplayOn()
{
	return debugMode;
}

void SetDebugDisplayMode(int state)
{
	debugMode = state;
}

#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	static int debugDisplayCount;
	static dVector debugDisplayCallback[1024 * 4];
#endif

static void RenderBodyContactsAndTangentDiretions (NewtonBody* const body, float length)
{
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
		for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
			dVector point;
			dVector normal;	
			NewtonMaterial* const material = NewtonContactGetMaterial (contact);
			NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

			dVector tangnetDir0;
			dVector tangnetDir1;
			NewtonMaterialGetContactTangentDirections(material, body, &tangnetDir0[0], &tangnetDir1[0]);

			// if we are display debug info we need to block other threads from writing the data at the same time
			dVector p1 (point + normal.Scale (length));
			//dVector p0 (point - normal.Scale (length));
			dVector p0 (point);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);
		}
	}
}

static void RenderBodyContactsForces (NewtonBody* const body, float scale)
{

	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);	

	//draw normal forces in term of acceleration.
	//this  mean that two bodies with same shape but different mass will display the same force
	if (mass > 0.0f) {
		scale = scale/mass;
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point;
				dVector normal;	
				dVector tangnetDir0;
				dVector tangnetDir1;
				dVector contactForce;	
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);

				NewtonMaterialGetContactForce(material, body, &contactForce.m_x);
				NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

				// these are the components of the tangents forces at the contact point, the can be display at the contact position point.
				//NewtonMaterialGetContactTangentDirections(material, body, &tangnetDir0[0], &tangnetDir1[0]);
				//dVector tangentForce1 (tangnetDir0.Scale ((contactForce % tangnetDir0) * scale));
				//dVector tangentForce2 (tangnetDir1.Scale ((contactForce % tangnetDir1) * scale));

				dVector normalforce (normal.Scale (contactForce % normal));

				dVector p0 (point);
				dVector p1 (point + normalforce.Scale (scale));
				glVertex3f (p0.m_x, p0.m_y, p0.m_z);
				glVertex3f (p1.m_x, p1.m_y, p1.m_z);

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
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		dVector p0; 
		dVector p1; 
		dMatrix matrix;
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		NewtonBodyGetMatrix (body, &matrix[0][0]);
		NewtonCollisionCalculateAABB (collision, &matrix[0][0], &p0[0], &p1[0]);

		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p1.m_x, p0.m_y, p0.m_z);

		glVertex3f (p0.m_x, p1.m_y, p0.m_z);
		glVertex3f (p1.m_x, p1.m_y, p0.m_z);

		glVertex3f (p0.m_x, p1.m_y, p1.m_z);
		glVertex3f (p1.m_x, p1.m_y, p1.m_z);

		glVertex3f (p0.m_x, p0.m_y, p1.m_z);
		glVertex3f (p1.m_x, p0.m_y, p1.m_z);


		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p0.m_x, p1.m_y, p0.m_z);

		glVertex3f (p1.m_x, p0.m_y, p0.m_z);
		glVertex3f (p1.m_x, p1.m_y, p0.m_z);

		glVertex3f (p0.m_x, p0.m_y, p1.m_z);
		glVertex3f (p0.m_x, p1.m_y, p1.m_z);

		glVertex3f (p1.m_x, p0.m_y, p1.m_z);
		glVertex3f (p1.m_x, p1.m_y, p1.m_z);


		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p0.m_x, p0.m_y, p1.m_z);

		glVertex3f (p1.m_x, p0.m_y, p0.m_z);
		glVertex3f (p1.m_x, p0.m_y, p1.m_z);

		glVertex3f (p0.m_x, p1.m_y, p0.m_z);
		glVertex3f (p0.m_x, p1.m_y, p1.m_z);

		glVertex3f (p1.m_x, p1.m_y, p0.m_z);
		glVertex3f (p1.m_x, p1.m_y, p1.m_z);
	}
	glEnd();
}

void RenderContactPoints (NewtonWorld* const world)
{
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.5f, 1.0f);

	glBegin(GL_LINES);
	float length = 0.5f;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		RenderBodyContactsAndTangentDiretions (body, length);
/*
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point;
				dVector normal;	
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

				dVector tangnetDir0;
				dVector tangnetDir1;
				NewtonMaterialGetContactTangentDirections(material, body, &tangnetDir0[0], &tangnetDir1[0]);

				// if we are display debug info we need to block other threads from writing the data at the same time
				dVector p0 (point + normal.Scale (length));
				dVector p1 (point - normal.Scale (length));
				glVertex3f (p0.m_x, p0.m_y, p0.m_z);
				glVertex3f (p1.m_x, p1.m_y, p1.m_z);
			}
		}
*/
	}
	glEnd();

	glPointSize(8.0f);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point;
				dVector normal;	
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

				// if we are display debug info we need to block other threads from writing the data at the same time
				glVertex3f (point.m_x, point.m_y, point.m_z);

			}
		}
	}
	glEnd();
	glPointSize(1.0f);
}


void RenderNormalForces (NewtonWorld* const world)
{
	glDisable (GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.0f, 0.5f, 1.0f);
	glBegin(GL_LINES);


	float length = 0.25f;
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		RenderBodyContactsForces (body, length);
	}
	glEnd();

	glPointSize(8.0f);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point;
				dVector normal;	
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);

				// if we are display debug info we need to block other threads from writing the data at the same time
				glVertex3f (point.m_x, point.m_y, point.m_z);

			}
		}
	}
	glEnd();
	glPointSize(1.0f);
}


void DebugShowGeometryCollision (void* userData, int vertexCount, const dFloat* faceVertec, int id)
{
	//DEBUG_DRAW_MODE mode = (DEBUG_DRAW_MODE) ((int)userData); //NOTE error: cast from ‘void*’ to ‘int’ loses precision
	DEBUG_DRAW_MODE mode = (DEBUG_DRAW_MODE) ((intptr_t)userData);
	
	if (mode == m_lines) {
		int i = vertexCount - 1;
		dVector p0 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		for (int i = 0; i < vertexCount; i ++) {
			dVector p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);
			p0 = p1;
		}
	} else {
		dVector p0 (faceVertec[0 * 3 + 0], faceVertec[0 * 3 + 1], faceVertec[0 * 3 + 2]);
		dVector p1 (faceVertec[1 * 3 + 0], faceVertec[1 * 3 + 1], faceVertec[1 * 3 + 2]);
		dVector p2 (faceVertec[2 * 3 + 0], faceVertec[2 * 3 + 1], faceVertec[2 * 3 + 2]);

		dVector normal ((p1 - p0) * (p2 - p0));
		normal = normal.Scale (1.0f / dSqrt (normal % normal));
		glNormal3f(normal.m_x, normal.m_y, normal.m_z);
		for (int i = 2; i < vertexCount; i ++) {
			dVector p2 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
			glVertex3f (p0.m_x, p0.m_y, p0.m_z);
			glVertex3f (p1.m_x, p1.m_y, p1.m_z);
			glVertex3f (p2.m_x, p2.m_y, p2.m_z);
			p1 = p2;
		}
	}
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

		case NEWTON_DEFORMABLE_BODY:
			glColor3f (0.0f, 1.0f, 1.0f);
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
		DebugShowBodyCollision (body, mode);
	}
	glEnd();

	glDisable (GL_LIGHTING);
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 0.0f);
	for (int i = 0; i < debugDisplayCount; i += 2) {
		glVertex3f (debugDisplayCallback[i].m_x, debugDisplayCallback[i].m_y, debugDisplayCallback[i].m_z);
		glVertex3f (debugDisplayCallback[i+1].m_x, debugDisplayCallback[i+1].m_y, debugDisplayCallback[i+1].m_z);
	}
	glEnd();
}

void DebugDrawPoint (const dVector& p, dFloat size)
{
	glPointSize(size);
	glBegin(GL_POINTS);
	glVertex3f(p.m_x, p.m_y, p.m_z);
	glEnd();
	glPointSize(1.0f);
}

void DebugDrawLine (const dVector& p0, const dVector& p1)
{
	glBegin(GL_LINES);
	glVertex3f (p0.m_x, p0.m_y, p0.m_z);
	glVertex3f (p1.m_x, p1.m_y, p1.m_z);
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
	debugDisplayCount = 0;
}

void ShowMeshCollidingFaces (const NewtonBody* const staticCollisionBody, const NewtonBody* const body, int faceID, int vertexCount, const dFloat* const vertex, int vertexstrideInBytes)
{
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	if (debugMode) {
		if ((debugDisplayCount + vertexCount * 2) < int (sizeof (debugDisplayCallback) / sizeof(debugDisplayCallback[0]))) {
			// we are coping data to and array of memory, another call back may be doing the same thing
			// here fore we need to avoid race conditions
			NewtonWorldCriticalSectionLock (NewtonBodyGetWorld (staticCollisionBody), 0);

			int stride = vertexstrideInBytes / sizeof (dFloat);
			dVector l0 (vertex[(vertexCount-1) * stride + 0], vertex[(vertexCount-1) * stride + 1], vertex[(vertexCount-1) * stride + 2], 0.0f);
			for (int j = 0; j < vertexCount; j ++) {
				dVector l1 (vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2], 0.0f);
				debugDisplayCallback[debugDisplayCount + 0] = l0;
				debugDisplayCallback[debugDisplayCount + 1] = l1;
				debugDisplayCount  += 2;
				l0 = l1;
			}
	

			// unlock the critical section
			NewtonWorldCriticalSectionUnlock (NewtonBodyGetWorld (staticCollisionBody));
		}

	}
#endif
}
