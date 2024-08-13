/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndOpenGlUtil.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"

#if 0
#ifndef D_USE_GEOMETRY_SHADERS 
static glVector3 CalculatePoint(const ndMatrix& matrix, const ndVector& center, ndFloat32 x, ndFloat32 y, ndFloat32 w)
{
	ndVector point(center.m_x + x, center.m_y + y, center.m_z, center.m_w);
	point = matrix.TransformVector1x4(point.Scale(w));
	return glVector3(GLfloat(point.m_x), GLfloat(point.m_y), GLfloat(point.m_z));
}
#endif

static void DrawBox(const ndVector& p0, const ndVector& p1, glVector3 box[12][2])
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
	
	GLuint shader = scene->GetShaderCache().m_wireFrame;
	
	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix viewProjectionMatrix(camera->GetInvViewProjectionMatrix());
	
	glVector4 color;
	color.m_x = 0.0f;
	color.m_y = 0.0f;
	color.m_z = 1.0f;
	color.m_w = 1.0f;
	
	glUseProgram(shader);
	
	ndInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	ndInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	
	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);
	
	glVector3 box[12][2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof (glVector3), box);
	
	const ndBodyListView& bodyList = world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndVector p0;
		ndVector p1;
		ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
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
	const glMatrix viewProjectionMatrix(camera->GetInvViewProjectionMatrix());

	glVector4 color(ndVector(1.0f, 1.0f, 0.0f, 1.0f));

	glUseProgram(shader);

	ndInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	ndInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

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

		virtual void OnDebugNode(const ndBvhNode* const node)
		{
			ndVector p0;
			ndVector p1;
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

void RenderPolygon(ndDemoEntityManager* const scene, const ndVector* const points, ndInt32 count, const ndVector& color)
{
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix viewProjectionMatrix(camera->GetInvViewProjectionMatrix());

	glUseProgram(shader);

	ndInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	ndInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);

	glVector3 line[64];

	glVector4 colorgl(color);
	glUniform4fv(shadeColorLocation, 1, &colorgl[0]);

	ndInt32 i0 = count - 1;
	for (ndInt32 i = 0; i < count; ++i)
	{
		line[i * 2 + 0].m_x = GLfloat(points[i0].m_x);
		line[i * 2 + 0].m_y = GLfloat(points[i0].m_y);
		line[i * 2 + 0].m_z = GLfloat(points[i0].m_z);
		line[i * 2 + 1].m_x = GLfloat(points[i].m_x);
		line[i * 2 + 1].m_y = GLfloat(points[i].m_y);
		line[i * 2 + 1].m_z = GLfloat(points[i].m_z);
		i0 = i;
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(glVector3), line);

	glDrawArrays(GL_LINES, 0, count * 2);
	glDisableClientState(GL_VERTEX_ARRAY);
	glUseProgram(0);
}

void RenderBodyFrame(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;
	
	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix viewProjectionMatrix(camera->GetInvViewProjectionMatrix());
	
	glUseProgram(shader);
	
	ndInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	ndInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjectionMatrix[0][0]);
	
	glVector3 line[2];
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, sizeof(glVector3), line);
	
	const ndBodyListView& bodyList = world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
	
		ndMatrix matrix(body->GetMatrix());
		ndVector o(matrix.m_posit);
		line[0].m_x = GLfloat(o.m_x);
		line[0].m_y = GLfloat(o.m_y);
		line[0].m_z = GLfloat(o.m_z);
	
		ndVector x(o + matrix.RotateVector(ndVector(0.5f, 0.0f, 0.0f, 0.0f)));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		glVector4 color (ndVector (1.0f, 0.0f, 0.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);
	
		x = o + matrix.RotateVector(ndVector(0.0f, 0.5f, 0.0f, 0.0f));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		color = glVector4(ndVector(0.0f, 1.0f, 0.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);
	
		x = o + matrix.RotateVector(ndVector(0.0f, 0.0f, 0.5f, 0.0f));
		line[1].m_x = GLfloat(x.m_x);
		line[1].m_y = GLfloat(x.m_y);
		line[1].m_z = GLfloat(x.m_z);
		color = glVector4(ndVector(0.0f, 0.0f, 1.0f, 0.0f));
		glUniform4fv(shadeColorLocation, 1, &color[0]);
		glDrawArrays(GL_LINES, 0, 2);
	}
	
	glDisableClientState(GL_VERTEX_ARRAY);
	glUseProgram(0);
}

#ifdef D_USE_GEOMETRY_SHADERS 
void RenderParticles(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_spriteSpheres;

	ndDemoCamera* const camera = scene->GetCamera();
	const ndMatrix invViewMatrix(camera->GetInvViewMatrix());
	const ndMatrix projectionMatrix(camera->GetProjectionMatrix());

	glVector4 color;
	color.m_x = 50.0f / 255.0f;
	color.m_y = 100.0f / 255.0f;
	color.m_z = 200.0f / 255.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	ndInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	ndInt32 viewModelMatrixLocation = glGetUniformLocation(shader, "viewModelMatrix");

	ndInt32 projectionMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");

	ndInt32 quadLocation = glGetUniformLocation(shader, "quadSize");
	ndInt32 uvSizeLocation = glGetUniformLocation(shader, "uvSize");
	ndInt32 radiusLocation = glGetUniformLocation(shader, "spriteRadius");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);

	const glMatrix glViewMatrix(invViewMatrix);
	glUniformMatrix4fv(viewModelMatrixLocation, 1, false, &glViewMatrix[0][0]);

	const glMatrix glProjectionMatrix(projectionMatrix);
	glUniformMatrix4fv(projectionMatrixLocation, 1, false, &glProjectionMatrix[0][0]);
	
	const ndBodyList& particles = world->GetParticleList();
	for (ndBodyList::ndNode* particleNode = particles.GetFirst(); particleNode; particleNode = particleNode->GetNext())
	{
		ndBodyParticleSet* const particle = particleNode->GetInfo()->GetAsBodyParticleSet();
		const ndArray<ndVector>& positions = particle->GetPositions();
		if (positions.GetCount())
		{
			glEnableClientState(GL_VERTEX_ARRAY);
			//glVertexPointer(3, GL_FLOAT, sizeof(glVector3), &pointBuffer[0]);
			glVertexPointer(4, GL_FLOAT, 0, &positions[0]);

			ndFloat32 radius = 1.0f * particle->GetParticleRadius();

			glVector4 quadUV[] =
			{
				ndVector(-1.0f, -1.0f, 0.0f, 0.0f),
				ndVector(1.0f, -1.0f, 0.0f, 0.0f),
				ndVector(-1.0f, 1.0f, 0.0f, 0.0f),
				ndVector(1.0f, 1.0f, 0.0f, 0.0f),
			};

			glVector4 quad[] =
			{
				ndVector(-radius, -radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(radius, -radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(-radius,  radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(radius,  radius, ndFloat32(0.0f), ndFloat32(0.0f)),
			};

			glVector4 spriteRadius(radius, radius, radius, 0.0f);
			glUniform4fv(radiusLocation, 1, &spriteRadius.m_x);
			glUniform4fv(quadLocation, 4, &quad[0].m_x);
			glUniform4fv(uvSizeLocation, 4, &quadUV[0][0]);
			glDrawArrays(GL_POINTS, 0, positions.GetCount());
		}
	}
	
	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);
}

#else

void RenderParticles(ndDemoEntityManager* const scene)
{
	ndWorld* const world = scene->GetWorld();
	GLuint shader = scene->GetShaderCache().m_wireFrame;

	ndDemoCamera* const camera = scene->GetCamera();
	const ndMatrix invViewMatrix(camera->GetViewMatrix());
	const ndMatrix projectionMatrix(camera->GetProjectionMatrix());
	const ndMatrix viewProjectionMatrix(invViewMatrix * projectionMatrix);

	glVector4 color;
	color.m_x = 50.0f / 255.0f;
	color.m_y = 100.0f / 255.0f;
	color.m_z = 200.0f / 255.0f;
	color.m_w = 1.0f;

	glUseProgram(shader);

	ndInt32 shadeColorLocation = glGetUniformLocation(shader, "shadeColor");
	ndInt32 projectionViewModelMatrixLocation = glGetUniformLocation(shader, "projectionViewModelMatrix");

	glUniform4fv(shadeColorLocation, 1, &color.m_x);
	const glMatrix viewProjMatrix(viewProjectionMatrix);
	glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, false, &viewProjMatrix[0][0]);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	static ndArray<glVector3> pointBuffer;

	const ndBodyParticleSetList& particles = world->GetParticleList();
	for (ndBodyParticleSetList::ndNode* particleNode = particles.GetFirst(); particleNode; particleNode = particleNode->GetNext())
	{
		ndBodyParticleSet* const particle = particleNode->GetInfo();
		const ndArray<ndVector>& positions = particle->GetPositions();

		pointBuffer.SetCount(6 * positions.GetCount());
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, &pointBuffer[0]);

		{
			//D_TRACKTIME();
			ndFloat32 radius = particle->GetParticleRadius();

			//radius *= 16.0f;
			//radius *= 0.7f;
			radius *= 0.5f;
			ndVector quad[] =
			{
				ndVector(-radius,  radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(-radius, -radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(radius,  radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(radius,  radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(-radius, -radius, ndFloat32(0.0f), ndFloat32(0.0f)),
				ndVector(radius, -radius, ndFloat32(0.0f), ndFloat32(0.0f)),
			};

			for (ndInt32 i = 0; i < positions.GetCount(); ++i)
			{
				const ndVector p(invViewMatrix.TransformVector(positions[i]));

				ndInt32 j = i * 6;
				pointBuffer[j + 0] = invViewMatrix.UntransformVector(p + quad[0]);
				pointBuffer[j + 1] = invViewMatrix.UntransformVector(p + quad[1]);
				pointBuffer[j + 2] = invViewMatrix.UntransformVector(p + quad[2]);
				pointBuffer[j + 3] = invViewMatrix.UntransformVector(p + quad[3]);
				pointBuffer[j + 4] = invViewMatrix.UntransformVector(p + quad[4]);
				pointBuffer[j + 5] = invViewMatrix.UntransformVector(p + quad[5]);
			}
		}
		glDrawArrays(GL_TRIANGLES, 0, pointBuffer.GetCount());
	}

	glUseProgram(0);
	glDisableClientState(GL_VERTEX_ARRAY);
}
#endif

#endif

ndDebugDisplay::ndDebugDisplay()
	:m_normalForces()
	,m_contactsPonts()
	,m_jointDebugInfo()
	,m_modelsDebugInfo()
	,m_shapesDebugInfo()
{
}

ndDebugDisplay::~ndDebugDisplay()
{
}

void ndDebugDisplay::Init(ndDemoEntityManager* const scene)
{
	m_centerOfMass.Init(scene);
	m_normalForces.Init(scene);
	m_contactsPonts.Init(scene);
	m_jointDebugInfo.Init(scene);
	m_modelsDebugInfo.Init(scene);
	m_shapesDebugInfo.Init(scene);
}

void ndDebugDisplay::Cleanup()
{
	m_centerOfMass.CleanUp();
	m_normalForces.CleanUp();
	m_contactsPonts.CleanUp();
	m_jointDebugInfo.CleanUp();
	m_modelsDebugInfo.CleanUp();
	m_shapesDebugInfo.CleanUp();
}

void ndDebugDisplay::UpdateCenterOfMass(ndDemoEntityManager* const scene)
{
	m_centerOfMass.UpdateBuffers(scene);
}

void ndDebugDisplay::UpdateContactPoints(ndDemoEntityManager* const scene)
{
	m_contactsPonts.UpdateBuffers(scene);
}

void ndDebugDisplay::UpdateNormalForces(ndDemoEntityManager* const scene)
{
	m_normalForces.UpdateBuffers(scene);
}

void ndDebugDisplay::UpdateJointsDebugInfo(ndDemoEntityManager* const scene)
{
	m_jointDebugInfo.UpdateBuffers(scene);
}

void ndDebugDisplay::UpdateModelsDebugInfo(ndDemoEntityManager* const scene)
{
	m_modelsDebugInfo.UpdateBuffers(scene);
}

void ndDebugDisplay::UpdateDebugShapes(ndDemoEntityManager* const scene, ndInt32 collisionDisplayMode)
{
	m_shapesDebugInfo.m_debugMode = collisionDisplayMode;
	m_shapesDebugInfo.UpdateBuffers(scene);
}

void ndDebugDisplay::RenderCenterOfMass(ndDemoEntityManager* const scene)
{
	m_centerOfMass.Render(scene);
}

void ndDebugDisplay::RenderContactPoints(ndDemoEntityManager* const scene)
{
	m_contactsPonts.Render(scene);
}

void ndDebugDisplay::RenderNormalForces(ndDemoEntityManager* const scene, ndFloat32 scale)
{
	m_normalForces.m_scale = scale;
	m_normalForces.Render(scene);
}

void ndDebugDisplay::RenderJointsDebugInfo(ndDemoEntityManager* const scene)
{
	m_jointDebugInfo.Render(scene);
}

void ndDebugDisplay::RenderModelsDebugInfo(ndDemoEntityManager* const scene)
{
	m_modelsDebugInfo.Render(scene);
}

void ndDebugDisplay::RenderDebugShapes(ndDemoEntityManager* const scene, ndInt32 collisionDisplayMode)
{
	m_shapesDebugInfo.m_debugMode = collisionDisplayMode;
	m_shapesDebugInfo.Render(scene);
}

// ***************************************************************************
//
// ***************************************************************************
ndDebugDisplay::ndDebugPass::ndDebugPass()
	:m_lock()
	,m_points()
	,m_shader(0)
	,m_vertexBuffer(0)
	,m_vertextArrayBuffer(0)
	,m_frameTick0(0)
	,m_frameTick1(1)
	,m_projectionViewModelMatrixLocation(0)
{
	m_points.SetCount(0);
}

ndDebugDisplay::ndDebugPass::~ndDebugPass()
{
}

void ndDebugDisplay::ndDebugPass::Init(ndDemoEntityManager* const scene)
{
	m_frameTick0 = 0;
	m_frameTick1 = 1;

	glGenBuffers(1, &m_vertexBuffer);
	glGenVertexArrays(1, &m_vertextArrayBuffer);

	glBindVertexArray(m_vertextArrayBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ndColorPoint), (void*)OFFSETOF(ndColorPoint, m_point));

	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ndColorPoint), (void*)OFFSETOF(ndColorPoint, m_color));

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	m_shader = scene->GetShaderCache().m_colorPoint;
	glUseProgram(m_shader);
	m_projectionViewModelMatrixLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
	glUseProgram(0);
}

void ndDebugDisplay::ndDebugPass::CleanUp()
{
	if (m_vertextArrayBuffer)
	{
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
	if (m_vertexBuffer)
	{
		glDeleteBuffers(1, &m_vertexBuffer);
	}

	m_vertexBuffer = 0;
	m_vertextArrayBuffer = 0;
}

void ndDebugDisplay::ndDebugPass::LoadBufferData(ndArray<ndColorPoint>& data)
{
	ndScopeSpinLock lock(m_lock);

	if (m_frameTick1 != m_frameTick0)
	{
		if (data.GetCount())
		{
			glBindVertexArray(m_vertextArrayBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
			glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(data.GetCount() * sizeof(ndColorPoint)), &data[0].m_point.m_x, GL_DYNAMIC_DRAW);

			//const glVector3* const xxxx = (glVector3*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);
			//glUnmapBuffer(GL_ARRAY_BUFFER);

			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
		}
	}
	m_frameTick0 = m_frameTick1;
}

void ndDebugDisplay::ndDebugPass::RenderBuffer(ndDemoEntityManager* const scene, GLenum mode, GLint pointCount, GLuint vertexArrayBuffer, GLuint vertexBuffer)
{
	ndDemoCamera* const camera = scene->GetCamera();
	const glMatrix invViewProjectionMatrix(camera->GetInvViewProjectionMatrix());
	
	glUseProgram(m_shader);
	glUniformMatrix4fv(m_projectionViewModelMatrixLocation, 1, false, &invViewProjectionMatrix[0][0]);
	
	glBindVertexArray(vertexArrayBuffer);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	
	glDrawArrays(mode, 0, pointCount);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

//***************************************************************************
//
//***************************************************************************
void ndDebugDisplay::ndCenterOfMass::UpdateBuffers(ndDemoEntityManager* const scene)
{
	ndScopeSpinLock lock(m_lock);

	m_points.SetCount(0);
	ndWorld* const world = scene->GetWorld();

	ndColorPoint red;
	ndColorPoint green;
	ndColorPoint blue;
	red.m_color = glVector3(GLfloat(1.0f), GLfloat(0.0f), GLfloat(0.0f));
	green.m_color = glVector3(GLfloat(0.0f), GLfloat(1.0f), GLfloat(0.0f));
	blue.m_color = glVector3(GLfloat(0.0f), GLfloat(0.0f), GLfloat(1.0f));

	ndColorPoint colorPoint;
	ndFloat32 scale = ndFloat32(0.25f);
	const ndBodyListView& bodyList = world->GetBodyList();
	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		const ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();

		const ndMatrix matrix(body->GetMatrix());
		const ndVector com(body->GetCentreOfMass());
		const ndVector o(matrix.TransformVector(com));

		red.m_point = o;
		m_points.PushBack(red);
		red.m_point = o + matrix.RotateVector(ndVector(scale, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)));
		m_points.PushBack(red);

		green.m_point = o;
		m_points.PushBack(green);
		green.m_point = o + matrix.RotateVector(ndVector(ndFloat32(0.0f), scale, ndFloat32(0.0f), ndFloat32(0.0f)));
		m_points.PushBack(green);

		blue.m_point = o;
		m_points.PushBack(blue);
		blue.m_point = o + matrix.RotateVector(ndVector(ndFloat32(0.0f), ndFloat32(0.0f), scale, ndFloat32(0.0f)));
		m_points.PushBack(blue);
	}

	m_frameTick1++;
}

void ndDebugDisplay::ndCenterOfMass::Render(ndDemoEntityManager* const scene)
{
	LoadBufferData(m_points);
	if (m_points.GetCount())
	{
		//glLineWidth(GLfloat(1.0f));
		RenderBuffer(scene, GL_LINES, ndInt32 (m_points.GetCount()), m_vertextArrayBuffer, m_vertexBuffer);
		//glLineWidth(GLfloat(1.0f));
	}
}

// ***************************************************************************
//
// ***************************************************************************
void ndDebugDisplay::ndContactPoints::UpdateBuffers(ndDemoEntityManager* const scene)
{
	ndScopeSpinLock lock(m_lock);

	m_points.SetCount(0);
	ndWorld* const world = scene->GetWorld();
	const ndContactArray& contactList = world->GetContactList();

	glVector3 color(GLfloat(1.0f), GLfloat(0.0f), GLfloat(0.0f));
	for (ndInt32 i = 0; i < contactList.GetCount(); ++i)
	{
		const ndContact* const contact = contactList[i];
		if (contact->IsActive())
		{
			const ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
			{
				const ndContactPoint& contactPoint = contactPointsNode->GetInfo();

				ndColorPoint colorPoint;
				colorPoint.m_point = contactPoint.m_point;
				//colorPoint.m_point.m_y += 1.0f;
				colorPoint.m_color = color;
				m_points.PushBack(colorPoint);
			}
		}
	}
	m_frameTick1++;
}

void ndDebugDisplay::ndContactPoints::Render(ndDemoEntityManager* const scene)
{
	LoadBufferData(m_points);
	if (m_points.GetCount())
	{
		glPointSize(GLfloat(4.0f));
		RenderBuffer(scene, GL_POINTS, ndInt32 (m_points.GetCount()), m_vertextArrayBuffer, m_vertexBuffer);
		glPointSize(GLfloat(1.0f));
	}
}

// ***************************************************************************
//
// ***************************************************************************
void ndDebugDisplay::ndNormalForces::UpdateBuffers(ndDemoEntityManager* const scene)
{
	ndScopeSpinLock lock(m_lock);
	
	m_points.SetCount(0);
	
	ndWorld* const world = scene->GetWorld();
	const ndContactArray& contactList = world->GetContactList();
	for (ndInt32 i = 0; i < contactList.GetCount(); ++i)
	{
		const ndContact* const contact = contactList[i];
		if (contact->IsActive())
		{
			glVector3 color(GLfloat(1.0f), GLfloat(1.0f), GLfloat(0.0f));
			const ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
			{
				const ndContactMaterial& contactPoint = contactPointsNode->GetInfo();
				const ndVector origin(contactPoint.m_point);
				const ndVector normal(contactPoint.m_normal);
				const ndVector dest(origin + normal.Scale(contactPoint.m_normal_Force.m_force * m_scale));
	
				ndColorPoint colorPoint;
				colorPoint.m_point = origin;
				colorPoint.m_color = color;
				m_points.PushBack(colorPoint);
	
				colorPoint.m_point = dest;
				colorPoint.m_color = color;
				m_points.PushBack(colorPoint);
			}
		}
	}
	m_frameTick1++;
}

void ndDebugDisplay::ndNormalForces::Render(ndDemoEntityManager* const scene)
{
	LoadBufferData(m_points);
	if (m_points.GetCount())
	{
		ndFloat32 thickness = 1.0f;
		glLineWidth(GLfloat(thickness));
		RenderBuffer(scene, GL_LINES, ndInt32(m_points.GetCount()), m_vertextArrayBuffer, m_vertexBuffer);
		glLineWidth(GLfloat(1.0f));
	}
}

//***************************************************************************
//
//***************************************************************************
void ndDebugDisplay::ndModelsDebugInfo::Init(ndDemoEntityManager* const scene)
{
	ndDebugPass::Init(scene);
	
	m_scale = ndFloat32(0.5f);
	m_lineThickness = ndFloat32(1.0f);
	m_pointThickness = ndFloat32(1.0f);
	
	glGenBuffers(1, &m_lineBuffer);
	glGenVertexArrays(1, &m_lineArrayBuffer);
	
	glBindVertexArray(m_lineArrayBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_lineBuffer);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ndColorPoint), (void*)OFFSETOF(ndColorPoint, m_point));
	
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ndColorPoint), (void*)OFFSETOF(ndColorPoint, m_color));
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void ndDebugDisplay::ndModelsDebugInfo::CleanUp()
{
	ndDebugPass::CleanUp();
	
	if (m_lineBuffer)
	{
		glDeleteBuffers(1, &m_lineBuffer);
	}
	if (m_lineArrayBuffer)
	{
		glDeleteVertexArrays(1, &m_lineArrayBuffer);
	}
	m_lineBuffer = 0;
	m_lineArrayBuffer = 0;
}

void ndDebugDisplay::ndModelsDebugInfo::UpdateBuffers(ndDemoEntityManager* const scene)
{
	ndScopeSpinLock lock(m_lock);
	
	class ndJoindDebug : public ndConstraintDebugCallback
	{
		public:
		ndJoindDebug(ndModelsDebugInfo* const me)
			:ndConstraintDebugCallback()
			,m_me(me)
			,m_lineThikness(ndFloat32(0.0f))
			,m_pointThikness(ndFloat32(0.0f))
		{
			m_me->m_points.SetCount(0);
			m_me->m_lines.SetCount(0);
			SetScale(m_me->m_scale);
		}
	
		~ndJoindDebug()
		{
			m_me->m_lineThickness = m_lineThikness;
			m_me->m_pointThickness = m_pointThikness;
		}
	
		void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness)
		{
			ndColorPoint colorPoint;
			colorPoint.m_point = point;
			colorPoint.m_color = color;
			m_me->m_points.PushBack(colorPoint);
			m_pointThikness = ndMax(thickness, m_pointThikness);
		}
	
		void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness)
		{
			ndColorPoint colorPoint;
			colorPoint.m_point = p0;
			colorPoint.m_color = color;
			m_me->m_lines.PushBack(colorPoint);
			
			colorPoint.m_point = p1;
			colorPoint.m_color = color;
			m_me->m_lines.PushBack(colorPoint);
			
			m_lineThikness = ndMax(thickness, m_lineThikness);
		}
	
		ndModelsDebugInfo* m_me;
		ndFloat32 m_lineThikness;
		ndFloat32 m_pointThikness;
	};
	
	ndJoindDebug debugJoint(this);
	
	ndWorld* const world = scene->GetWorld();
	const ndModelList& modelList = world->GetModelList();
	for (ndModelList::ndNode* modelNode = modelList.GetFirst(); modelNode; modelNode = modelNode->GetNext())
	{ 
		ndModel* const model = *modelNode->GetInfo();
		ndSharedPtr<ndModelNotify>& notify = model->GetNotifyCallback();
		if (*notify)
		{
			notify->Debug(debugJoint);
		}
	}
	m_frameTick1++;
}

void ndDebugDisplay::ndModelsDebugInfo::LoadBufferData()
{
	ndScopeSpinLock lock(m_lock);
	
	if (m_frameTick1 != m_frameTick0)
	{
		if (m_lines.GetCount())
		{
			glBindVertexArray(m_lineArrayBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, m_lineBuffer);
			glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m_lines.GetCount() * sizeof(ndColorPoint)), &m_lines[0].m_point.m_x, GL_DYNAMIC_DRAW);
		
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
		}
	
		if (m_points.GetCount())
		{
			glBindVertexArray(m_vertextArrayBuffer);
			glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
			glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(m_points.GetCount() * sizeof(ndColorPoint)), &m_points[0].m_point.m_x, GL_DYNAMIC_DRAW);
	
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			glBindVertexArray(0);
		}
	}
	m_frameTick0 = m_frameTick1;
}

void ndDebugDisplay::ndModelsDebugInfo::Render(ndDemoEntityManager* const scene)
{
	LoadBufferData();
	if (m_points.GetCount())
	{
		glPointSize(GLfloat(m_pointThickness));
		RenderBuffer(scene, GL_POINTS, ndInt32(m_points.GetCount()), m_vertextArrayBuffer, m_vertexBuffer);
		glPointSize(GLfloat(1.0f));
	}
	
	if (m_lines.GetCount())
	{
		glLineWidth(GLfloat(m_lineThickness));
		RenderBuffer(scene, GL_LINES, ndInt32(m_lines.GetCount()), m_lineArrayBuffer, m_lineBuffer);
		glLineWidth(GLfloat(1.0f));
	}
}

//***************************************************************************
//
//***************************************************************************
void ndDebugDisplay::ndJointDebugInfo::UpdateBuffers(ndDemoEntityManager* const scene)
{
	ndScopeSpinLock lock(m_lock);

	class ndJoindDebug : public ndConstraintDebugCallback
	{
		public:
		ndJoindDebug(ndModelsDebugInfo* const me)
			:ndConstraintDebugCallback()
			,m_me(me)
			,m_lineThikness(ndFloat32(0.0f))
			,m_pointThikness(ndFloat32(0.0f))
		{
			m_me->m_points.SetCount(0);
			m_me->m_lines.SetCount(0);
			SetScale(m_me->m_scale);
		}

		~ndJoindDebug()
		{
			m_me->m_lineThickness = m_lineThikness;
			m_me->m_pointThickness = m_pointThikness;
		}

		void DrawPoint(const ndVector& point, const ndVector& color, ndFloat32 thickness)
		{
			ndColorPoint colorPoint;
			colorPoint.m_point = point;
			colorPoint.m_color = color;
			m_me->m_points.PushBack(colorPoint);
			m_pointThikness = ndMax(thickness, m_pointThikness);
		}

		void DrawLine(const ndVector& p0, const ndVector& p1, const ndVector& color, ndFloat32 thickness)
		{
			ndColorPoint colorPoint;
			colorPoint.m_point = p0;
			colorPoint.m_color = color;
			m_me->m_lines.PushBack(colorPoint);

			colorPoint.m_point = p1;
			colorPoint.m_color = color;
			m_me->m_lines.PushBack(colorPoint);

			m_lineThikness = ndMax(thickness, m_lineThikness);
		}

		ndModelsDebugInfo* m_me;
		ndFloat32 m_lineThikness;
		ndFloat32 m_pointThikness;
	};

	ndJoindDebug debugJoint(this);
	debugJoint.SetScale(ndFloat32 (0.2f));

	ndWorld* const world = scene->GetWorld();
	const ndJointList& jointList = world->GetJointList();
	for (ndJointList::ndNode* jointNode = jointList.GetFirst(); jointNode; jointNode = jointNode->GetNext())
	{
		ndJointBilateralConstraint* const joint = *jointNode->GetInfo();
		joint->DebugJoint(debugJoint);
	}
	m_frameTick1++;
}

//***************************************************************************
//
//***************************************************************************
void ndDebugDisplay::ndShapesDebugInfo::Init(ndDemoEntityManager* const)
{
}

void ndDebugDisplay::ndShapesDebugInfo::CleanUp()
{
	m_meshCache.RemoveAll();
}

void ndDebugDisplay::ndShapesDebugInfo::UpdateBuffers(ndDemoEntityManager* const)
{
	ndScopeSpinLock lock(m_lock);
	m_frameTick1++;
}

void ndDebugDisplay::ndShapesDebugInfo::LoadBufferData(ndDemoEntityManager* const scene)
{
	ndScopeSpinLock lock(m_lock);

	if (m_frameTick1 != m_frameTick0)
	{
		ndWorld* const world = scene->GetWorld();
		const ndBodyListView& bodyList = world->GetBodyList();

		for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
		{
			ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
			const ndShapeInstance& shapeInstance = body->GetCollisionShape();
			ndShape* const key = (ndShape*)shapeInstance.GetShape();
			if (!(key->GetAsShapeNull() || key->GetAsShapeStaticProceduralMesh()))
			{
				ndDebugMeshCache::ndNode* shapeNode = m_meshCache.Find(key);
				if (!shapeNode)
				{
					ndShapeInstance shape(body->GetCollisionShape());
					shape.SetScale(ndVector(1.0f));
					shape.SetLocalMatrix(ndGetIdentityMatrix());

					ndDebugMesh debugMesh;
					debugMesh.m_flatShaded = new ndFlatShadedDebugMesh(scene->GetShaderCache(), &shape);
					debugMesh.m_zBufferShaded = new ndZbufferDebugMesh(scene->GetShaderCache(), &shape);
					debugMesh.m_wireFrameShareEdge = new ndWireFrameDebugMesh(scene->GetShaderCache(), &shape);
					if (shape.GetShape()->GetAsShapeStaticBVH())
					{
						debugMesh.m_wireFrameOpenEdge = new ndWireFrameDebugMesh(scene->GetShaderCache(), &shape, ndShapeDebugNotify::ndEdgeType::m_open);
						debugMesh.m_wireFrameOpenEdge->SetColor(ndVector(1.0f, 0.0f, 1.0f, 1.0f));
					}
					shapeNode = m_meshCache.Insert(debugMesh, key);
				}
			}
		}
	}
	m_frameTick0 = m_frameTick1;
}

void ndDebugDisplay::ndShapesDebugInfo::Render(ndDemoEntityManager* const scene)
{
	LoadBufferData(scene);
	if (!m_meshCache.GetCount())
	{
		return;
	}

	ndWorld* const world = scene->GetWorld();
	const ndBodyListView& bodyList = world->GetBodyList();

	const ndVector awakeColor(1.0f, 1.0f, 1.0f, 1.0f);
	const ndVector sleepColor(0.42f, 0.73f, 0.98f, 1.0f);

	if (m_debugMode == 3)
	{
		for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
		{
			ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
			const ndShapeInstance& shapeInstance = body->GetCollisionShape();
			ndShape* const key = (ndShape*)shapeInstance.GetShape();

			const ndMatrix matrix(shapeInstance.GetScaledTransform(body->GetMatrix()));
			
			ndDebugMeshCache::ndNode* const shapeNode = m_meshCache.Find(key);
			if (shapeNode)
			{
				ndZbufferDebugMesh* const zbufferMesh = *shapeNode->GetInfo().m_zBufferShaded;
				zbufferMesh->Render(scene, matrix);
			}
		}
	}

	for (ndBodyListView::ndNode* bodyNode = bodyList.GetFirst(); bodyNode; bodyNode = bodyNode->GetNext())
	{
		ndBodyKinematic* const body = bodyNode->GetInfo()->GetAsBodyKinematic();
		const ndShapeInstance& shapeInstance = body->GetCollisionShape();
		ndShape* const key = (ndShape*)shapeInstance.GetShape();

		const ndMatrix matrix(shapeInstance.GetScaledTransform(body->GetMatrix()));
		const ndVector color((body->GetSleepState() == 1) ? sleepColor : awakeColor);

		if (m_debugMode >= 2)
		{
			ndDebugMeshCache::ndNode* const shapeNode = m_meshCache.Find(key);
			if (shapeNode)
			{
				ndWireFrameDebugMesh* const sharedEdgeMesh = *shapeNode->GetInfo().m_wireFrameShareEdge;
				sharedEdgeMesh->SetColor(color);
				sharedEdgeMesh->Render(scene, matrix);
			}
		}
		else
		{
			ndDebugMeshCache::ndNode* const shapeNode = m_meshCache.Find(key);
			if (shapeNode)
			{
				ndFlatShadedDebugMesh* const shadedMesh = *shapeNode->GetInfo().m_flatShaded;
				shadedMesh->SetColor(color);
				shadedMesh->Render(scene, matrix);
			}
		}
	}

	//RenderParticles(this);
}