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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderSceneNode.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderShaderCache.h"
#include "ndRenderPassShadowsImplement.h"

#define ND_SHADOW_MAP_RESOLUTION	(1024 * 4)
#define ND_MAX_FAR_PLANE			ndFloat32 (50.0f)
#define ND_MIN_NEAR_PLANE			ndFloat32 (0.1f)


ndRenderPassShadowsImplement::ndRenderPassShadowsImplement(ndRenderContext* const context)
	:ndClassAlloc()
	,m_context(context)
{
	m_width = ND_SHADOW_MAP_RESOLUTION;
	m_height = ND_SHADOW_MAP_RESOLUTION;

	glGenFramebuffers(1, &m_frameBufferObject);

	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBufferObject);
	// Disable writes to the color buffer
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	
	// Create the depth buffer textures, onle one 4 x 4 tile
	glGenTextures(1, &m_shadowMapTexture);
	glBindTexture(GL_TEXTURE_2D, m_shadowMapTexture);
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, m_width * 2, m_height * 2, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_shadowMapTexture, 0);
	
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	ndAssert(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);

	//m_generateShadowMapsBlock.GetShaderParameters(*m_context->m_shaderCache);
	m_generateIntanceShadowMapsBlock.GetShaderParameters(*m_context->m_shaderCache);
	
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

ndRenderPassShadowsImplement::~ndRenderPassShadowsImplement()
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

void ndRenderPassShadowsImplement::CalculateWorldSpaceSubFrustum(const ndRenderSceneCamera* const camera, ndVector* const frustum, ndInt32 sliceIndex) const
{
	const ndVector p0(ndVector::m_zero);

	const ndMatrix& viewMatrix = camera->m_viewMatrix;
	auto CalculateFrustumPoint = [this, &viewMatrix, &p0](const ndVector& p1, ndFloat32 xdist)
	{
		const ndVector p10(p1 - p0);
		ndFloat32 step = (xdist - p0.m_x) / p10.m_x;
		const ndVector point(xdist, p0.m_y + p10.m_y * step, p0.m_z + p10.m_z * step, ndFloat32(1.0f));
		return viewMatrix.TransformVector(point);
	};

	for (ndInt32 i = 0; i < 4; ++i)
	{
		const ndVector p1(camera->m_frustum[i + 4]);
		frustum[0 * 4 + i] = CalculateFrustumPoint(p1, m_nearFrustumPlanes[sliceIndex]);
		frustum[1 * 4 + i] = CalculateFrustumPoint(p1, m_farFrustumPlanes[sliceIndex]);
	}
}

ndMatrix ndRenderPassShadowsImplement::CreateLightMatrix(const ndVector& origin) const
{
	ndMatrix lighMatrix(ndGetIdentityMatrix());

	const ndVector normUp(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));

	ndRender* const owner = m_context->m_owner;
	lighMatrix.m_front = owner->m_sunLightDir * ndVector::m_negOne;
	lighMatrix.m_right = lighMatrix.m_front.CrossProduct(normUp).Normalize();
	lighMatrix.m_up = lighMatrix.m_right.CrossProduct(lighMatrix.m_front);

	lighMatrix.m_posit = (origin & ndVector::m_triplexMask) | ndVector::m_wOne;
	return lighMatrix;
}

ndMatrix ndRenderPassShadowsImplement::CalculateLightSpaceMatrix(const ndRenderSceneCamera* const camera, ndInt32 sliceIndex) const
{
	ndVector frustum[8];
	CalculateWorldSpaceSubFrustum(camera, frustum, sliceIndex);

	ndVector origin(ndVector::m_zero);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		origin += frustum[i];
	}
	origin = origin.Scale(ndFloat32(1.0f / 8.0f));
	
	const ndMatrix& worldToGl = camera->m_worldToOpenGl;
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

void ndRenderPassShadowsImplement::UpdateCascadeSplits(const ndRenderSceneCamera* const camera)
{
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

void ndRenderPassShadowsImplement::RenderScene(const ndRenderSceneCamera* const camera)
{
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_frameBufferObject);
	glDisable(GL_SCISSOR_TEST);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glClear(GL_DEPTH_BUFFER_BIT);

	//glPolygonOffset(GLfloat(1.0f), GLfloat(1024.0f * 8.0f));
	glPolygonOffset(GLfloat(1.0f), GLfloat(1024.0f * 16.0f));
	glEnable(GL_POLYGON_OFFSET_FILL);

	UpdateCascadeSplits(camera);
	
	ndMatrix tileMatrix(ndGetIdentityMatrix());
	tileMatrix[0][0] = ndFloat32(0.5f);
	tileMatrix[1][1] = ndFloat32(0.5f);
	
	ndVector cameraTestPoint(ndVector::m_wOne);
	const ndMatrix& cameraProjection = camera->m_projectionMatrix;
	
	ndRender* const owner = m_context->m_owner;

	auto RenderPrimitive = [this, &owner, camera, &cameraTestPoint, &cameraProjection, &tileMatrix](ndRenderPassMode modepass)
	{
		const ndList<ndSharedPtr<ndRenderSceneNode>>& scene = owner->m_scene;
		for (ndInt32 i = 0; i < 4; i++)
		{
			const ndMatrix lightSpaceMatrix(CalculateLightSpaceMatrix(camera, i));
			const ndVector viewPortTile(m_viewPortTiles[i]);
			ndInt32 vp_x = ndInt32(viewPortTile.m_x * ndFloat32(2 * m_width));
			ndInt32 vp_y = ndInt32(viewPortTile.m_y * ndFloat32(2 * m_height));

			cameraTestPoint.m_x = m_farFrustumPlanes[i];
			const ndVector cameraPoint(cameraProjection.TransformVector1x4(cameraTestPoint));
			m_cameraSpaceSplits[i] = GLfloat(ndFloat32(0.5f) * cameraPoint.m_z / cameraPoint.m_w + ndFloat32(0.5f));

			tileMatrix[3][0] = viewPortTile.m_x;
			tileMatrix[3][1] = viewPortTile.m_y;
			m_lighProjectionMatrix[i] = lightSpaceMatrix * m_lightProjectToTextureSpace * tileMatrix;

			glViewport(vp_x, vp_y, m_width, m_height);
			for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = scene.GetFirst(); node; node = node->GetNext())
			{
				ndRenderSceneNode* const sceneNode = *node->GetInfo();
				sceneNode->Render(owner, 0.0f, lightSpaceMatrix, modepass);
			}
		}
	};

	// render simple primitive pass
	RenderPrimitive(m_generateShadowMaps);

	// render instance primitives, fucking big mistake
	RenderPrimitive(m_m_generateInstanceShadowMaps);

	glDisable(GL_POLYGON_OFFSET_FILL);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	m_context->SetViewport(m_context->GetWidth(), m_context->GetHeight());
}