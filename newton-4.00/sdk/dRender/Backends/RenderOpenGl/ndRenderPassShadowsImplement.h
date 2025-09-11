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
#ifndef __ND_RENDER_PASS_SHADOWS_IMPLEMNETH__
#define __ND_RENDER_PASS_SHADOWS_IMPLEMNETH__

#include "ndRenderPass.h"
#include "ndRenderContext.h"
#include "ndRenderShaderCache.h"

class ndRenderPassShadowsImplement : public ndClassAlloc
{
	public:
	ndRenderPassShadowsImplement(ndRenderContext* const context);
	virtual ~ndRenderPassShadowsImplement();

	virtual void RenderScene(const ndRenderSceneCamera* const camera);

	private:
	ndMatrix CreateLightMatrix(const ndVector& origin) const;
	void UpdateCascadeSplits(const ndRenderSceneCamera* const camera);
	ndMatrix CalculateLightSpaceMatrix(const ndRenderSceneCamera* const camera, ndInt32 sliceIndex) const;
	void CalculateWorldSpaceSubFrustum(const ndRenderSceneCamera* const camera, ndVector* const frustum, ndInt32 sliceIndex) const;

	ndMatrix m_lightProjectToTextureSpace;
	ndMatrix m_lighProjectionMatrix[4];
	ndVector m_viewPortTiles[4];
	ndVector m_nearFrustumPlanes;
	ndVector m_farFrustumPlanes;
	
	ndVector m_cameraSpaceSplits;

	ndRenderContext* m_context;

	ndInt32 m_width;
	ndInt32 m_height;
	GLuint m_shadowMapTexture;
	GLuint m_frameBufferObject;
	ndRenderShaderGenerateShadowMapBlock m_generateShadowMapsBlock;
	
	friend class ndRenderPrimitiveMeshImplement;
	friend class ndRenderShaderOpaqueDiffusedShadowColorBlock;
};

#endif