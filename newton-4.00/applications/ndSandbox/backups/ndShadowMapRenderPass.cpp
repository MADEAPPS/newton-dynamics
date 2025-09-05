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

#define ND_MAX_FAR_PLANE	ndFloat32 (50.0f)
#define ND_MIN_NEAR_PLANE	ndFloat32 (0.1f)

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
	Cleanup();
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

	m_viewPortTiles[0] = ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[1] = ndVector(ndFloat32(0.5f), ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[2] = ndVector(ndFloat32(0.0f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f));
	m_viewPortTiles[3] = ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f));

	m_lightProjectToTextureSpace = ndGetIdentityMatrix();
	m_lightProjectToTextureSpace[0][0] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace[1][1] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace[2][2] = ndFloat32(0.5f);
	m_lightProjectToTextureSpace.m_posit = ndVector(ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(0.5f), ndFloat32(1.0f));
}

void ndShadowMapRenderPass::Cleanup()
{
	if (m_shadowMapTexture != 0)
	{
		glDeleteTextures(1, &m_shadowMapTexture);
	}

	if (m_frameBufferObject != 0)
	{
		glDeleteFramebuffers(1, &m_frameBufferObject);
	}
	m_shadowMapTexture = 0;
	m_frameBufferObject = 0;
}

ndMatrix ndShadowMapRenderPass::CreateLightMatrix() const
{
	ndMatrix lighMatrix(ndGetIdentityMatrix());
#if 0
	const ndVector normUp(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector xAxis(m_manager->GetDirectionsLight() * ndVector::m_negOne);
	const ndVector xDir(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	lighMatrix.m_front = xDir;
	lighMatrix.m_right = lighMatrix.m_front.CrossProduct(normUp).Normalize();
	lighMatrix.m_up = lighMatrix.m_right.CrossProduct(lighMatrix.m_front);
#else
	lighMatrix.m_front = ndVector(ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	lighMatrix.m_up = ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	lighMatrix.m_right = lighMatrix.m_front.CrossProduct(lighMatrix.m_up);
#endif
	return lighMatrix;
}

void ndShadowMapRenderPass::UpdateCascadeSplits()
{
	const ndFloat32 farPlane = ndMin(m_camera->m_backPlane, ND_MAX_FAR_PLANE);
	const ndFloat32 nearPlane = ndMax(m_camera->m_frontPlane, ND_MIN_NEAR_PLANE);
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
	const ndVector p0(ndVector::m_zero);
	ndVector pMin(ndFloat32(1.0e10f));
	ndVector pMax(ndFloat32(-1.0e10f));

	auto CalculateFrustumPoint = [this, &camInvLight, &p0](const ndVector& p1, ndFloat32 xdist)
	{
		const ndVector p10(p1 - p0);
		ndFloat32 step = (xdist - p0.m_x) / p10.m_x;
		const ndVector lighFrutum(xdist, p0.m_y + p10.m_y * step, p0.m_z + p10.m_z * step, ndFloat32(1.0f));

		const ndMatrix camViewMatrix(m_camera->GetViewMatrix());
		const ndMatrix invLighMatrix(CreateLightMatrix().OrthoInverse());
		//const ndVector xxx0(m_camera->GetViewMatrix().TransformVector(p0));
		//const ndVector xxx1(invLighMatrix.TransformVector(xxx0));
		//const ndVector xxx2(m_camera->GetWorlToGlMatrix().TransformVector(xxx1));
		//
		//const ndVector yyy0(m_camera->GetViewMatrix().TransformVector(p1));
		//const ndVector yyy1(invLighMatrix.TransformVector(yyy0));
		//const ndVector yyy2(m_camera->GetWorlToGlMatrix().TransformVector(yyy1));
		
		const ndVector zzz0(camViewMatrix.TransformVector(lighFrutum));
		const ndVector zzz1(invLighMatrix.TransformVector(zzz0));
		const ndVector zzz2(m_camera->GetWorlToGlMatrix().TransformVector(zzz1));

		return camInvLight.TransformVector(lighFrutum);
	};
	
	for (ndInt32 i = 0; i < 4; ++i)
	{
		const ndVector p1(m_camera->m_frustum[i + 4]);

		const ndVector f0(CalculateFrustumPoint(p1, m_nearFrustumPlanes[sliceIndex]));
		pMin = pMin.GetMin(f0);
		pMax = pMax.GetMax(f0);

		const ndVector f1(CalculateFrustumPoint(p1, m_farFrustumPlanes[sliceIndex]));
		pMin = pMin.GetMin(f1);
		pMax = pMax.GetMax(f1);
	}
	
	// Initialize orthographic matrix
	ndFloat32 left    = -pMin.m_x;
	ndFloat32 right   = -pMax.m_x;
	ndFloat32 top	  = -pMin.m_y;
	ndFloat32 bottom  = -pMax.m_y;
	ndFloat32 nearVal = -pMin.m_z;
	ndFloat32 farVal  = -pMax.m_z;

	ndMatrix orthoMatrix(ndGetIdentityMatrix());
	orthoMatrix[0][0] = ndFloat32(2.0f) / (right - left);
	orthoMatrix[1][1] = ndFloat32(2.0f) / (top - bottom);
	orthoMatrix[2][2] = -ndFloat32(2.0f) / (farVal - nearVal);
	orthoMatrix[3][0] = -(right + left) / (right - left);
	orthoMatrix[3][1] = -(top + bottom) / (top - bottom);
	orthoMatrix[3][2] = -(farVal + nearVal) / (farVal - nearVal);

	const ndMatrix invLighMatrix(CreateLightMatrix().OrthoInverse());
	 
	ndVector xxx0(invLighMatrix.TransformVector(ndVector(m_nearFrustumPlanes[sliceIndex], 0.0f, 0.0f, 1.0f)));
	const ndVector xxx1(m_camera->GetWorlToGlMatrix().TransformVector(xxx0));
	const ndVector xxx2(orthoMatrix.TransformVector(xxx1));
	
	ndVector yyy0(invLighMatrix.TransformVector(ndVector(m_farFrustumPlanes[sliceIndex], 0.0f, 0.0f, 1.0f)));
	const ndVector yyy1(m_camera->GetWorlToGlMatrix().TransformVector(yyy0));
	const ndVector yyy2(orthoMatrix.TransformVector(yyy1));
	
	return m_camera->GetWorlToGlMatrix() * orthoMatrix;
}

void ndShadowMapRenderPass::CalculateWorldSpaceSubFrustum(ndVector* const frustum, ndInt32 sliceIndex) const
{
	const ndVector p0(ndVector::m_zero);

	const ndMatrix& viewMatrix = m_camera->GetViewMatrix();
	auto CalculateFrustumPoint = [this, &viewMatrix, &p0](const ndVector& p1, ndFloat32 xdist)
	{
		const ndVector p10(p1 - p0);
		ndFloat32 step = (xdist - p0.m_x) / p10.m_x;
		const ndVector point(xdist, p0.m_y + p10.m_y * step, p0.m_z + p10.m_z * step, ndFloat32(1.0f));
		return viewMatrix.TransformVector(point);
	};

	for (ndInt32 i = 0; i < 4; ++i)
	{
		const ndVector p1(m_camera->m_frustum[i + 4]);
		frustum[0 * 4 + i] = CalculateFrustumPoint(p1, m_nearFrustumPlanes[sliceIndex]);
		frustum[1 * 4 + i] = CalculateFrustumPoint(p1, m_farFrustumPlanes[sliceIndex]);
	}
}

ndMatrix ndShadowMapRenderPass::CreateLightMatrix(const ndVector& origin) const
{
	ndMatrix lighMatrix(ndGetIdentityMatrix());
#if 1
	const ndVector normUp(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	//const ndVector xDir(m_manager->GetDirectionsLight() * ndVector::m_negOne);
	lighMatrix.m_front = m_manager->GetDirectionsLight() * ndVector::m_negOne;
	lighMatrix.m_right = lighMatrix.m_front.CrossProduct(normUp).Normalize();
	lighMatrix.m_up = lighMatrix.m_right.CrossProduct(lighMatrix.m_front);
#else
	lighMatrix.m_front = ndVector(ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	lighMatrix.m_up = ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
	lighMatrix.m_right = lighMatrix.m_front.CrossProduct(lighMatrix.m_up);
#endif

	lighMatrix.m_posit = (origin & ndVector::m_triplexMask) | ndVector::m_wOne;
	return lighMatrix;
}

ndMatrix ndShadowMapRenderPass::CalculateLightSpaceMatrix(ndInt32 sliceIndex) const
{
	ndVector frustum[8];
	CalculateWorldSpaceSubFrustum(frustum, sliceIndex);

	ndVector origin (ndVector::m_zero);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		origin += frustum[i];
	}
	origin = origin.Scale(ndFloat32(1.0f / 8.0f));

	const ndMatrix& worldToGl = m_camera->GetWorlToGlMatrix();
	const ndMatrix invLightSpaceMatrix(CreateLightMatrix(origin).OrthoInverse() * worldToGl);

	ndVector pMin(ndFloat32(1.0e10f));
	ndVector pMax(ndFloat32(-1.0e10f));
	for (ndInt32 i = 0; i < 8; ++i)
	{
		ndVector point(invLightSpaceMatrix.TransformVector(frustum[i]));
		pMin = pMin.GetMin(point);
		pMax = pMax.GetMax(point);
	}

	// pad the volume to capture rounding errors.
	pMin -= ndVector(4.0f);
	pMax += ndVector(4.0f);

	ndFloat32 left = pMin.m_x;
	ndFloat32 right = pMax.m_x;
	ndFloat32 top = pMin.m_y;
	ndFloat32 bottom = pMax.m_y;
	ndFloat32 nearVal = pMin.m_z;
	ndFloat32 farVal = pMax.m_z;

	ndMatrix orthoMatrix(ndGetIdentityMatrix());
	orthoMatrix[0][0] = ndFloat32(2.0f) / (right - left);
	orthoMatrix[1][1] = ndFloat32(2.0f) / (top - bottom);
	orthoMatrix[2][2] = -ndFloat32(2.0f) / (farVal - nearVal);
	orthoMatrix[3][0] = -(right + left) / (right - left);
	orthoMatrix[3][1] = -(top + bottom) / (top - bottom);
	orthoMatrix[3][2] = -(farVal + nearVal) / (farVal - nearVal);

	return invLightSpaceMatrix * orthoMatrix;
}

void ndShadowMapRenderPass::RenderScene(ndFloat32)
{

	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_frameBufferObject);
	glDisable(GL_SCISSOR_TEST);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	
	glClear(GL_DEPTH_BUFFER_BIT);
	glUseProgram(m_shaderProgram);
	
	glPolygonOffset(GLfloat(1.0f), GLfloat(1024.0f * 8.0f));
	glEnable(GL_POLYGON_OFFSET_FILL);

	m_camera = m_manager->GetCamera();
	UpdateCascadeSplits();

	ndMatrix tileMatrix(ndGetIdentityMatrix());
	tileMatrix[0][0] = ndFloat32(0.5f);
	tileMatrix[1][1] = ndFloat32(0.5f);

	ndVector cameraTestPoint(ndVector::m_wOne);
	const ndMatrix& cameraProjection = m_camera->GetProjectionMatrix();
	for (ndInt32 i = 0; i < 4; i++)
	{
		ndMatrix lightSpaceMatrix(CalculateLightSpaceMatrix(i));

		const ndVector viewPortTile(m_viewPortTiles[i]);
		ndInt32 vp_x = ndInt32(viewPortTile.m_x * ndFloat32(2 * m_width));
		ndInt32 vp_y = ndInt32(viewPortTile.m_y * ndFloat32(2 * m_height));

		cameraTestPoint.m_x = m_farFrustumPlanes[i];
		const ndVector cameraPoint(cameraProjection.TransformVector1x4(cameraTestPoint));
		m_cameraSpaceSplits[i] = GLfloat(ndFloat32 (0.5f) * cameraPoint.m_z / cameraPoint.m_w + ndFloat32(0.5f));

		tileMatrix[3][0] = viewPortTile.m_x;
		tileMatrix[3][1] = viewPortTile.m_y;
		m_lighProjectionMatrix[i] = glMatrix(lightSpaceMatrix * m_lightProjectToTextureSpace * tileMatrix);

		glViewport(vp_x, vp_y, m_width, m_height);
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = m_manager->GetFirst(); node; node = node->GetNext())
		{
			ndDemoEntity* const entity = *node->GetInfo();
			if (entity->CastShadow())
			{
				//if (i==3)
				entity->RenderShadowMap(this, lightSpaceMatrix);
			}
		}
	}

	glDisable(GL_POLYGON_OFFSET_FILL);
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
	glUseProgram(0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}