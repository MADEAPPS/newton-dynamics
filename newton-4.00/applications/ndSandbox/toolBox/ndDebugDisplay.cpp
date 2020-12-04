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
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"

#if 0

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
			for (dInt32 j = 0; j < vertexCount; j ++) {
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

void RenderListenersDebugInfo (NewtonWorld* const world, dJointDebugDisplay* const jointDebug)
{
	NewtonWorldListenerDebug(world, jointDebug);
}

#endif


static ndMeshVector CalculatePoint(const dMatrix& matrix, const dVector& center, dFloat32 x, dFloat32 y, dFloat32 w)
{
	dVector point(center.m_x + x, center.m_y + y, center.m_z, center.m_w);
	point = matrix.TransformVector1x4(point.Scale(w));
	return ndMeshVector(GLfloat(point.m_x), GLfloat(point.m_y), GLfloat(point.m_z));
}

static void DrawBox(const dVector& p0, const dVector& p1, ndMeshVector box[12][2])
{
	//ndMeshVector box[12][2];
	box[0][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[0][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));

	box[1][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
	box[1][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

	box[2][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
	box[2][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[3][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
	box[3][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

	box[4][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[4][1] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

	box[5][0] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[5][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

	box[6][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
	box[6][1] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[7][0] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
	box[7][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[8][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[8][1] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

	box[9][0] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[9][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

	box[10][0] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
	box[10][1] = ndMeshVector(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[11][0] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
	box[11][1] = ndMeshVector(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	glDrawArrays(GL_LINES, 0, 24);
}

void RenderBodiesAABB(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	const ndBodyList& bodyList = world->GetBodyList();

	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());

	ndMeshVector4 color;
	color.m_x = 0.0f;
	color.m_y = 0.0f;
	color.m_z = 1.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	ndMeshVector box[12][2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof (ndMeshVector), box);

	for (ndBodyList::dListNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		dVector p0;
		dVector p1;
		ndBodyKinematic* const body = bodyNode->GetInfo();
		body->GetAABB(p0, p1);
		DrawBox(p0, p1, box);
	}
	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderWorldScene(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());

	ndMeshVector4 color;
	color.m_x = 1.0f;
	color.m_y = 1.0f;
	color.m_z = 0.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);
	glEnableClientState(GL_VERTEX_ARRAY);

	class ndDrawScene: public ndSceneTreeNotiFy
	{
		public: 
		ndDrawScene()
			:ndSceneTreeNotiFy()
		{
			glVertexPointer(3, GL_FLOAT, sizeof(ndMeshVector), m_box);
		}

		virtual void OnDebugNode(const ndSceneNode* const node)
		{
			dVector p0;
			dVector p1;
			node->GetAABB(p0, p1);
			DrawBox(p0, p1, m_box);
		}

		ndMeshVector m_box[12][2];
	};

	ndDrawScene drawBroaphase;
	world->DebugScene(&drawBroaphase);

	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderContactPoints(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
	dMatrix invViewProjectionMatrix(camera->GetProjectionMatrix().Inverse4x4() * camera->GetViewMatrix().Inverse());

	ndMeshVector4 color;
	color.m_x = 1.0f;
	color.m_y = 0.0f;
	color.m_z = 0.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	dFloat32 pizelSize = 8.0f / viewport[2];

	ndMeshVector pointBuffer[4];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof (ndMeshVector), pointBuffer);
	const ndContactList& contactList = world->GetContactList();
	for (ndContactList::dListNode* contactNode = contactList.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	{
		const ndContact* const contact = &contactNode->GetInfo();
		if (contact->IsActive())
		{
			const ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::dListNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
			{
				const ndContactPoint& contactPoint = contactPointsNode->GetInfo();
				dVector point(viewProjectionMatrix.TransformVector1x4(contactPoint.m_point));
				dFloat32 zDist = point.m_w;
				point = point.Scale(1.0f / zDist);

				pointBuffer[0] = CalculatePoint(invViewProjectionMatrix, point, -pizelSize, pizelSize, zDist);
				pointBuffer[1] = CalculatePoint(invViewProjectionMatrix, point, -pizelSize, -pizelSize, zDist);
				pointBuffer[2] = CalculatePoint(invViewProjectionMatrix, point, pizelSize, pizelSize, zDist);
				pointBuffer[3] = CalculatePoint(invViewProjectionMatrix, point, pizelSize, -pizelSize, zDist);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			}
		}
	}

	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderCenterOfMass(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
	
	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	ndMeshVector line[2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(ndMeshVector), line);

	const ndBodyList& bodyList = world->GetBodyList();
	for (ndBodyList::dListNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();

		dMatrix matrix(body->GetMatrix());
		dVector com(body->GetCentreOfMass());
		
		dVector o(matrix.TransformVector(com));
		line[0].m_x = o.m_x;
		line[0].m_y = o.m_y;
		line[0].m_z = o.m_z;

		dVector x(o + matrix.RotateVector(dVector(0.5f, 0.0f, 0.0f, 0.0f)));
		line[1].m_x = x.m_x;
		line[1].m_y = x.m_y;
		line[1].m_z = x.m_z;
		dVector color(1.0f, 0.0f, 0.0f, 0.0f);
		glUniform4fv(shadeColorLocation, 1, &color.m_x);
		glDrawArrays(GL_LINES, 0, 2);

		x = o + matrix.RotateVector(dVector(0.0f, 0.5f, 0.0f, 0.0f));
		line[1].m_x = x.m_x;
		line[1].m_y = x.m_y;
		line[1].m_z = x.m_z;
		color = dVector(0.0f, 1.0f, 0.0f, 0.0f);
		glUniform4fv(shadeColorLocation, 1, &color.m_x);
		glDrawArrays(GL_LINES, 0, 2);

		x = o + matrix.RotateVector(dVector(0.0f, 0.0f, 0.5f, 0.0f));
		line[1].m_x = x.m_x;
		line[1].m_y = x.m_y;
		line[1].m_z = x.m_z;
		color = dVector (0.0f, 0.0f, 1.0f, 0.0f);
		glUniform4fv(shadeColorLocation, 1, &color.m_x);
		glDrawArrays(GL_LINES, 0, 2);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
	glUseProgram(0);
}

void RenderParticles(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
	dMatrix invViewProjectionMatrix(camera->GetProjectionMatrix().Inverse4x4() * camera->GetViewMatrix().Inverse());

	ndMeshVector4 color;
	color.m_x = 50.0f / 255.0f;
	color.m_y = 100.0f / 255.0f;
	color.m_z = 200.0f / 255.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	dFloat32 pizelSize = 8.0f / viewport[2];

	ndMeshVector pointBuffer[4];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(ndMeshVector), pointBuffer);

	const ndBodyParticleSetList& particles = world->GetParticleList();
	for (ndBodyParticleSetList::dListNode* particleNode = particles.GetFirst(); particleNode; particleNode = particleNode->GetNext())
	{
		ndBodyParticleSet* const particle = particleNode->GetInfo();
		const dArray<dVector>& positions = particle->GetPositions();
		for (dInt32 i = 0; i < positions.GetCount(); i ++)
		{
			dVector particlePosit(positions[i]);
			particlePosit.m_w = 1.0f;
			dVector point(viewProjectionMatrix.TransformVector1x4(particlePosit));
			dFloat32 zDist = point.m_w;
			point = point.Scale(1.0f / zDist);

			pointBuffer[0] = CalculatePoint(invViewProjectionMatrix, point, -pizelSize, pizelSize, zDist);
			pointBuffer[1] = CalculatePoint(invViewProjectionMatrix, point, -pizelSize, -pizelSize, zDist);
			pointBuffer[2] = CalculatePoint(invViewProjectionMatrix, point, pizelSize, pizelSize, zDist);
			pointBuffer[3] = CalculatePoint(invViewProjectionMatrix, point, pizelSize, -pizelSize, zDist);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		}
	}

	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void RenderJointsDebugInfo(ndDemoEntityManager* const scene)
{
	class ndJoindDebug : public ndConstraintDebugCallback
	{
		public:
		ndJoindDebug(ndDemoEntityManager* const scene)
		{
			ndDemoCamera* const camera = scene->GetCamera();
			dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
			m_shader = scene->GetShaderCache().m_wireFrame;

			glUseProgram(m_shader);

			m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
			m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
			glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(ndMeshVector), m_line);
		}

		~ndJoindDebug()
		{
			glDisableClientState(GL_VERTEX_ARRAY);
			glUseProgram(0);
		}

		void DrawLine(const dVector& p0, const dVector& p1, const dVector& color)
		{
			m_line[0].m_x = p0.m_x;
			m_line[0].m_y = p0.m_y;
			m_line[0].m_z = p0.m_z;
			m_line[1].m_x = p1.m_x;
			m_line[1].m_y = p1.m_y;
			m_line[1].m_z = p1.m_z;
			glUniform4fv(m_shadeColorLocation, 1, &color.m_x);
			glDrawArrays(GL_LINES, 0, 2);
		}

		GLuint m_shader;
		dInt32 m_shadeColorLocation;
		dInt32 m_projectionViewModelMatrixLocation;

		ndMeshVector m_line[2];
	};

	ndJoindDebug debugJoint(scene);
	ndWorld* const workd = scene->GetWorld();
	const ndJointList& jointList = workd->GetJointList();
	for (ndJointList::dListNode* jointNode = jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
	{
		ndJointBilateralConstraint* const joint = jointNode->GetInfo();
		joint->DebugJoint(debugJoint);
	}
}

void RenderModelsDebugInfo(ndDemoEntityManager* const scene)
{
	class ndJoindDebug : public ndConstraintDebugCallback
	{
		public:
		ndJoindDebug(ndDemoEntityManager* const scene)
		{
			ndDemoCamera* const camera = scene->GetCamera();
			dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
			m_shader = scene->GetShaderCache().m_wireFrame;

			glUseProgram(m_shader);

			m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
			m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
			glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(ndMeshVector), m_line);
		}

		~ndJoindDebug()
		{
			glDisableClientState(GL_VERTEX_ARRAY);
			glUseProgram(0);
		}

		void DrawLine(const dVector& p0, const dVector& p1, const dVector& color)
		{
			m_line[0].m_x = p0.m_x;
			m_line[0].m_y = p0.m_y;
			m_line[0].m_z = p0.m_z;
			m_line[1].m_x = p1.m_x;
			m_line[1].m_y = p1.m_y;
			m_line[1].m_z = p1.m_z;
			glUniform4fv(m_shadeColorLocation, 1, &color.m_x);
			glDrawArrays(GL_LINES, 0, 2);
		}

		GLuint m_shader;
		dInt32 m_shadeColorLocation;
		dInt32 m_projectionViewModelMatrixLocation;

		ndMeshVector m_line[2];
	};

	ndJoindDebug debugJoint(scene);
	ndWorld* const workd = scene->GetWorld();
	//const ndJointList& jointList = workd->GetJointList();
	const ndModelList& modelList = workd->GetModelList();
	for (ndModelList::dListNode* jointNode = modelList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
	{
		ndModel* const model = jointNode->GetInfo();
		model->Debug(debugJoint);
	}
}