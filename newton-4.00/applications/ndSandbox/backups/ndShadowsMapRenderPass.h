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
#ifndef __ND_SHADOW_MAPS_RENDER_PASS_H__
#define __ND_SHADOW_MAPS_RENDER_PASS_H__

#include "ndRenderPass.h"

class ndDemoEntity;
class ndDemoEntityManager;

#define ND_SHADOW_MAP_RESOLUTION (1024 * 4)

class ndShadowMapRenderPass: public ndRenderPass
{
	public:
	ndShadowMapRenderPass();
	~ndShadowMapRenderPass();

	virtual void Cleanup();
	virtual void Init(ndDemoEntityManager* const manager, ndInt32 arguments, ...);

	void RenderScene(ndFloat32 timestep);

	GLuint GetShadowMapTexture() const;
	GLuint GetShaderModelProjMatrix() const;
	glVector4 GetCameraSpaceSplits() const;
	const glMatrix* GetLightProjMatrix() const;


	ndMatrix CreateLightMatrix(const ndVector& origin) const;
	ndMatrix CalculateLightSpaceMatrix(ndInt32 indexSlice) const;
	void CalculateWorldSpaceSubFrustum(ndVector* const frustum, ndInt32 indexSlice) const;

	private:
	void UpdateCascadeSplits();
	ndMatrix CreateLightMatrix() const;
	ndMatrix CalculateOrthoMatrix(const ndMatrix& camInvLight, ndInt32 sliceIndex) const;
	
	ndMatrix m_lightProjectToTextureSpace;
	ndVector m_nearFrustumPlanes;
	ndVector m_farFrustumPlanes;
	ndVector m_viewPortTiles[4];

	glMatrix m_lighProjectionMatrix[4];
	glVector4 m_cameraSpaceSplits;

	ndDemoCamera* m_camera;
	ndInt32 m_width;
	ndInt32 m_height;

	GLuint m_shaderProgram;
	GLuint m_shadowMapTexture;
	GLuint m_frameBufferObject;
	GLuint m_modelProjectionMatrixLocation;
};

inline GLuint ndShadowMapRenderPass::GetShadowMapTexture() const
{
	return m_shadowMapTexture;
}

inline GLuint ndShadowMapRenderPass::GetShaderModelProjMatrix() const
{
	return m_modelProjectionMatrixLocation;
}

inline glVector4 ndShadowMapRenderPass::GetCameraSpaceSplits() const
{
	return m_cameraSpaceSplits;
}

inline const glMatrix* ndShadowMapRenderPass::GetLightProjMatrix() const
{
	return m_lighProjectionMatrix;
}
#endif