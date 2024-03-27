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
#include "ndDemoEntity.h"
#include "ndShadowsMapRenderPass.h"

#define ND_MAX_FAR_PLANE	ndFloat32 (100.0f)
#define ND_MIN_NEAR_PLANE	ndFloat32 (0.25f)

ndShadowMapRenderPass::ndShadowMapRenderPass()
	:ndRenderPass()
	,m_width(0)
	,m_height(0)
	,m_shaderProgram(0)
	,m_shadowMapTexture(0)
	,m_frameBufferObject(0)
	,m_modelProjectionMatrixLocation(0)
{
}

ndShadowMapRenderPass::~ndShadowMapRenderPass()
{
	if (m_shadowMapTexture != 0)
	{
		glDeleteTextures(1, &m_shadowMapTexture);
	}

	if (m_frameBufferObject != 0) 
	{
		glDeleteFramebuffers(1, &m_frameBufferObject);
	}
}

void ndShadowMapRenderPass::Init(ndDemoEntityManager* const manager, ndInt32 arguments, ...)
{
	ndRenderPass::Init(manager, 0);

	va_list args;
	va_start(args, arguments);
	m_shaderProgram = va_arg(args, GLuint);
	va_end(args);

	m_width = ND_SHADOW_MAP_RESOLUTION;
	m_height = ND_SHADOW_MAP_RESOLUTION;

	//ndInt32 glMajorVersion = 0;
	//ndInt32 glMinorVersion = 0;
	//glGetIntegerv(GL_MAJOR_VERSION, &glMajorVersion);
	//glGetIntegerv(GL_MINOR_VERSION, &glMinorVersion);
	//ndAssert ((glMajorVersion >= 4) && (glMinorVersion >= 5));

	// make a frame buffer for rendering to texture
	glGenFramebuffers(1, &m_frameBufferObject);

	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferObject);
	// Disable writes to the color buffer
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	// Create the depth buffer textures
	glGenTextures(1, &m_shadowMapTexture);
	glBindTexture(GL_TEXTURE_2D, m_shadowMapTexture);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_width * 2, m_height * 2, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_shadowMapTexture, 0);
	
	m_modelProjectionMatrixLocation = GLuint(glGetUniformLocation(m_shaderProgram, "viewModelProjectionMatrix"));

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

	//m_viewPorts[0].m_origin_x = 0;
	//m_viewPorts[0].m_origin_y = m_height;
	//
	//m_viewPorts[1].m_origin_x = m_width;
	//m_viewPorts[1].m_origin_y = m_height;
	//
	//m_viewPorts[2].m_origin_x = 0;
	//m_viewPorts[2].m_origin_y = 0;
	//
	//m_viewPorts[3].m_origin_x = m_width;
	//m_viewPorts[3].m_origin_y = 0;

	m_viewPortTiles[0] = ndVector(ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[1] = ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[2] = ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[3] = ndVector(ndFloat32(0.5f), ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f));

	m_lightProjectToTextureSpace = ndGetIdentityMatrix();
	m_lightProjectToTextureSpace[0][0] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace[1][1] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace[2][2] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace.m_posit = ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(1.0f));
}

ndMatrix ndShadowMapRenderPass::CreateLightMatrix() const
{
	ndMatrix lighMatrix(ndGetIdentityMatrix());
	//const ndVector normUp(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector zAxis(m_manager->GetDirectionsLight());
	//lighMatrix.m_right = zAxis;
	//lighMatrix.m_front = normUp.CrossProduct(lighMatrix.m_right).Normalize();
	//lighMatrix.m_up = lighMatrix.m_right.CrossProduct(lighMatrix.m_front);
	//return lighMatrix.Transpose4X4();

lighMatrix = ndPitchMatrix(-90.0f * ndDegreeToRad);
	
	return lighMatrix;
}

void ndShadowMapRenderPass::UpdateCascadeSplits()
{
	const ndDemoCamera* const camera = m_manager->GetCamera();
	const ndFloat32 farPlane = ndMin(camera->m_backPlane, ND_MAX_FAR_PLANE);
	const ndFloat32 nearPlane = ndMax(camera->m_frontPlane, ND_MIN_NEAR_PLANE);
	const ndFloat32 ratio = farPlane / nearPlane;
	
	ndFloat32 splitePlanes[5];
	splitePlanes[0] = nearPlane;
	ndFloat32 split_weight = ndFloat32(0.75f);
	for (ndInt32 i = 1; i < 5; i++)
	{
		ndFloat32 si = ndFloat32(i) / ndFloat32(4);
		splitePlanes[i] = split_weight * (nearPlane * ndPow(ratio, si)) + (ndFloat32(1.0f) - split_weight) * (nearPlane + (farPlane - nearPlane) * si);
	}
	m_nearFrustumPlanes = ndVector(splitePlanes[0], splitePlanes[1], splitePlanes[2], splitePlanes[3]);
	m_farFrustumPlanes = ndVector(splitePlanes[1], splitePlanes[2], splitePlanes[3], splitePlanes[4]);
	m_farFrustumPlanes = m_farFrustumPlanes.Scale(ndFloat32(1.001f));
	m_farFrustumPlanes.m_w = farPlane;
}

ndMatrix ndShadowMapRenderPass::CalculateOrthoMatrix(const ndMatrix& camInvLight, ndInt32 sliceIndex) const
{
	const ndDemoCamera* const camera = m_manager->GetCamera();
	
	ndVector pMin(ndFloat32(1.0e10f));
	ndVector pMax(ndFloat32(-1.0e10f));

	const ndVector p0(ndVector::m_zero);
	for (ndInt32 i = 0; i < 4; ++i)
	{
		//const ndVector p0(camera->m_frutrum[i + 0]);
		const ndVector p1(camera->m_frutrum[i + 4]);
		const ndVector p10(p1 - p0);
	
		ndFloat32 z0 = -m_nearFrustumPlanes[sliceIndex];
		ndFloat32 step0 = (z0 - p0.m_z) / p10.m_z;
		const ndVector lighFrutum0(p0.m_x + p10.m_x * step0, p0.m_y + p10.m_y * step0, z0, ndFloat32(1.0f));
		const ndVector q0(camInvLight.TransformVector(lighFrutum0));
	
		ndFloat32 z1 = -m_farFrustumPlanes[sliceIndex];
		ndFloat32 step1 = (z1 - p0.m_z) / p10.m_z;
		const ndVector lighFrutum1(p0.m_x + p10.m_x * step1, p0.m_y + p10.m_y * step1, z1, ndFloat32(1.0f));
		const ndVector q1(camInvLight.TransformVector(lighFrutum1));
		
		pMin = pMin.GetMin(q0.GetMin(q1));
		pMax = pMax.GetMax(q0.GetMax(q1));
	}
	
	// Initialize orthographic matrix
	ndFloat32 left    = pMin.m_x;
	ndFloat32 right   = pMax.m_x;
	ndFloat32 bottom  = pMin.m_y;
	ndFloat32 top	  = pMax.m_y;
	ndFloat32 nearVal = pMin.m_z;
	ndFloat32 farVal  = pMax.m_z;
	
	//m_orthoProjectionMatrix = ndGetIdentityMatrix();
	ndMatrix orthoMatrix(ndGetIdentityMatrix());
	orthoMatrix[0][0] = ndFloat32(2.0f) / (right - left);
	orthoMatrix[1][1] = ndFloat32(2.0f) / (top - bottom);
	orthoMatrix[2][2] = -ndFloat32(2.0f) / (farVal - nearVal);
	orthoMatrix[3][0] = -(right + left) / (right - left);
	orthoMatrix[3][1] = -(top + bottom) / (top - bottom);
	orthoMatrix[3][2] = -(farVal + nearVal) / (farVal - nearVal);
	
	return orthoMatrix;
}

void ndShadowMapRenderPass::RenderScene(ndFloat32)
{
	const ndMatrix lighMatrix (CreateLightMatrix());
	UpdateCascadeSplits();

	const ndDemoCamera* const camera = m_manager->GetCamera();

	const ndMatrix invLighMatrix(lighMatrix.OrthoInverse());
	const ndMatrix camInvLight(camera->GetViewMatrix().OrthoInverse() * invLighMatrix);
	
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_frameBufferObject);
	glDisable(GL_SCISSOR_TEST);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	//glClearDepth(0.0f);
	glClear(GL_DEPTH_BUFFER_BIT);

	ndVector cameraTestPoint(ndVector::m_wOne);
	const ndMatrix& cameraProjection = camera->GetProjectionMatrix();
	ndMatrix tileMatrix(ndGetIdentityMatrix());

	tileMatrix[0][0] = ndFloat32(0.5f);
	tileMatrix[1][1] = ndFloat32(0.5f);
	for (ndInt32 i = 0; i < 4; i++)
	{
		const ndVector viewPortTile(m_viewPortTiles[i]);
		ndInt32 vp_x = ndInt32(viewPortTile.m_x * ndFloat32(2 * m_width));
		ndInt32 vp_y = ndInt32(viewPortTile.m_y * ndFloat32(2 * m_height));
		glViewport(vp_x, vp_y, m_width, m_height);
		glUseProgram(m_shaderProgram);
	
		cameraTestPoint.m_z = -m_farFrustumPlanes[i];
		const ndVector cameraPoint(cameraProjection.TransformVector1x4(cameraTestPoint));
		ndFloat32 perpectiveZ = cameraPoint.m_z / cameraPoint.m_w;
		m_cameraSpaceSplits[i] = perpectiveZ;

		const ndMatrix projectionMatrix(CalculateOrthoMatrix(camInvLight, i));
		const ndMatrix camProjection(invLighMatrix * projectionMatrix);

		tileMatrix[3][0] = viewPortTile.m_x;
		tileMatrix[3][1] = viewPortTile.m_y;
		m_lighProjectionMatrix[i] = (camProjection * m_lightProjectToTextureSpace * tileMatrix);

		for (ndList<ndDemoEntity*>::ndNode* node = m_manager->GetFirst(); node; node = node->GetNext())
		{
			ndDemoEntity* const entity = node->GetInfo();
			//if (entity->CastShadow())
			//{
			//	if (i == 1)
			//	entity->RenderShadowMap(this, camProjection);
			//}
		}
	
		ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
		glUseProgram(0);
	}
	
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

	glClearDepth(1.0f);
}