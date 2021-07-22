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

static glVector3 CalculatePoint(const dMatrix& matrix, const dVector& center, dFloat32 x, dFloat32 y, dFloat32 w)
{
	dVector point(center.m_x + x, center.m_y + y, center.m_z, center.m_w);
	point = matrix.TransformVector1x4(point.Scale(w));
	return glVector3(GLfloat(point.m_x), GLfloat(point.m_y), GLfloat(point.m_z));
}

static void DrawBox(const dVector& p0, const dVector& p1, glVector3 box[12][2])
{
	//ndMeshVector box[12][2];
	box[0][0] = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[0][1] = glVector3(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));

	box[1][0] = glVector3(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
	box[1][1] = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

	box[2][0] = glVector3(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
	box[2][1] = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[3][0] = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
	box[3][1] = glVector3(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

	box[4][0] = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[4][1] = glVector3(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

	box[5][0] = glVector3(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[5][1] = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));

	box[6][0] = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
	box[6][1] = glVector3(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[7][0] = glVector3(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));
	box[7][1] = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[8][0] = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[8][1] = glVector3(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

	box[9][0] = glVector3(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
	box[9][1] = glVector3(GLfloat(p1.m_x), GLfloat(p0.m_y), GLfloat(p1.m_z));

	box[10][0] = glVector3(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
	box[10][1] = glVector3(GLfloat(p0.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	box[11][0] = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p0.m_z));
	box[11][1] = glVector3(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

	glDrawArrays(GL_LINES, 0, 24);
}

void RenderBodiesAABB(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	const ndBodyList& bodyList = world->GetBodyList();

	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());

	glVector4 color;
	color.m_x = 0.0f;
	color.m_y = 0.0f;
	color.m_z = 1.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	glVector3 box[12][2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof (glVector3), box);

	for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
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
	const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());

	glVector4 color(dVector(1.0f, 1.0f, 0.0f, 1.0f));

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color[0]);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);
	glEnableClientState(GL_VERTEX_ARRAY);

	class ndDrawScene: public ndSceneTreeNotiFy
	{
		public: 
		ndDrawScene()
			:ndSceneTreeNotiFy()
		{
			glVertexPointer(3, GL_FLOAT, sizeof(glVector3), m_box);
		}

		virtual void OnDebugNode(const ndSceneNode* const node)
		{
			dVector p0;
			dVector p1;
			node->GetAabb(p0, p1);
			DrawBox(p0, p1, m_box);
		}

		glVector3 m_box[12][2];
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
	const dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
	const dMatrix invViewProjectionMatrix(camera->GetProjectionMatrix().Inverse4x4() * camera->GetViewMatrix().Inverse());

	glVector4 color(dVector(1.0f, 0.0f, 0.0f, 1.0f));

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	glUniform4fv(shadeColorLocation, 1, &color[0]);
	const glMatrix viewProjMatrix(viewProjectionMatrix);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjMatrix[0][0]);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	dFloat32 pizelSize = 8.0f / viewport[2];

	glVector3 pointBuffer[4];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof (glVector3), pointBuffer);
	const ndContactList& contactList = world->GetContactList();
	for (ndContactList::dNode* contactNode = contactList.GetFirst(); contactNode; contactNode = contactNode->GetNext())
	{
		const ndContact* const contact = &contactNode->GetInfo();
		if (contact->IsActive())
		{
			const ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::dNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
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

void RenderBodyFrame(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	glVector3 line[2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(glVector3), line);

	const ndBodyList& bodyList = world->GetBodyList();
	for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();

		dMatrix matrix(body->GetMatrix());
		dVector o(matrix.m_posit);
		line[0].m_x = GLfloat(o.m_x);
		line[0].m_y = GLfloat(o.m_y);
		line[0].m_z = GLfloat(o.m_z);

		dVector x(o + matrix.RotateVector(dVector(0.5f, 0.0f, 0.0f, 0.0f)));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		glVector4 color (dVector (1.0f, 0.0f, 0.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);

		x = o + matrix.RotateVector(dVector(0.0f, 0.5f, 0.0f, 0.0f));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		color = glVector4(dVector(0.0f, 1.0f, 0.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);

		x = o + matrix.RotateVector(dVector(0.0f, 0.0f, 0.5f, 0.0f));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		color = glVector4(dVector(0.0f, 0.0f, 1.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
	glUseProgram(0);
}
void RenderCenterOfMass(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
	
	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	glVector3 line[2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(glVector3), line);

	const ndBodyList& bodyList = world->GetBodyList();
	for (ndBodyList::dNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo();

		dMatrix matrix(body->GetMatrix());
		dVector com(body->GetCentreOfMass());
		
		dVector o(matrix.TransformVector(com));
		line[0].m_x = GLfloat(o.m_x);
		line[0].m_y = GLfloat(o.m_y);
		line[0].m_z = GLfloat(o.m_z);

		dVector x(o + matrix.RotateVector(dVector(0.5f, 0.0f, 0.0f, 0.0f)));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		glVector4 color (dVector (1.0f, 0.0f, 0.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);

		x = o + matrix.RotateVector(dVector(0.0f, 0.5f, 0.0f, 0.0f));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		color = glVector4(dVector(0.0f, 1.0f, 0.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);

		x = o + matrix.RotateVector(dVector(0.0f, 0.0f, 0.5f, 0.0f));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		color = glVector4(dVector (0.0f, 0.0f, 1.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
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
	const dMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
	const dMatrix invViewProjectionMatrix(camera->GetProjectionMatrix().Inverse4x4() * camera->GetViewMatrix().Inverse());

	glVector4 color;
	color.m_x = 50.0f / 255.0f;
	color.m_y = 100.0f / 255.0f;
	color.m_z = 200.0f / 255.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	dInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	dInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	const glMatrix viewProjMatrix(viewProjectionMatrix);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjMatrix[0][0]);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	dFloat32 pizelSize = 8.0f / viewport[2];

	glVector3 pointBuffer[4];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(glVector3), pointBuffer);

	const ndBodyParticleSetList& particles = world->GetParticleList();
	for (ndBodyParticleSetList::dNode* particleNode = particles.GetFirst(); particleNode; particleNode = particleNode->GetNext())
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
			const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
			m_shader = scene->GetShaderCache().m_wireFrame;

			glUseProgram(m_shader);

			m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
			m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
			glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(glVector3), m_line);
		}

		~ndJoindDebug()
		{
			glDisableClientState(GL_VERTEX_ARRAY);
			glUseProgram(0);
		}

		void DrawLine(const dVector& p0, const dVector& p1, const dVector& color)
		{
			m_line[0].m_x = GLfloat(p0.m_x);
			m_line[0].m_y = GLfloat(p0.m_y);
			m_line[0].m_z = GLfloat(p0.m_z);
			m_line[1].m_x = GLfloat(p1.m_x);
			m_line[1].m_y = GLfloat(p1.m_y);
			m_line[1].m_z = GLfloat(p1.m_z);

			glVector4 c(color);
			glUniform4fv(m_shadeColorLocation, 1, &c[0]);
			glDrawArrays(GL_LINES, 0, 2);
		}

		GLuint m_shader;
		dInt32 m_shadeColorLocation;
		dInt32 m_projectionViewModelMatrixLocation;

		glVector3 m_line[2];
	};

	ndJoindDebug debugJoint(scene);
	ndWorld* const workd = scene->GetWorld();
	const ndJointList& jointList = workd->GetJointList();
	for (ndJointList::dNode* jointNode = jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
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
			const glMatrix viewProjectionMatrix(camera->GetViewMatrix() * camera->GetProjectionMatrix());
			m_shader = scene->GetShaderCache().m_wireFrame;

			glUseProgram(m_shader);

			m_shadeColorLocation = glGetUniformLocation(m_shader, "shadeColor");
			m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
			glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

			glEnableClientState(GL_VERTEX_ARRAY);
			glVertexPointer(3, GL_FLOAT, sizeof(glVector3), m_line);
		}

		~ndJoindDebug()
		{
			glDisableClientState(GL_VERTEX_ARRAY);
			glUseProgram(0);
		}
		
		void DrawLine(const dVector& p0, const dVector& p1, const dVector& color)
		{
			m_line[0].m_x = GLfloat(p0.m_x);
			m_line[0].m_y = GLfloat(p0.m_y);
			m_line[0].m_z = GLfloat(p0.m_z);
			m_line[1].m_x = GLfloat(p1.m_x);
			m_line[1].m_y = GLfloat(p1.m_y);
			m_line[1].m_z = GLfloat(p1.m_z);
			glVector4 c(color);

			glUniform4fv(m_shadeColorLocation, 1, &c[0]);
			glDrawArrays(GL_LINES, 0, 2);
		}

		GLuint m_shader;
		dInt32 m_shadeColorLocation;
		dInt32 m_projectionViewModelMatrixLocation;

		glVector3 m_line[2];
	};

	ndJoindDebug debugJoint(scene);
	ndWorld* const workd = scene->GetWorld();
	const ndModelList& modelList = workd->GetModelList();
	for (ndModelList::dNode* jointNode = modelList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
	{
		ndModel* const model = jointNode->GetInfo();
		model->Debug(debugJoint);
	}
}