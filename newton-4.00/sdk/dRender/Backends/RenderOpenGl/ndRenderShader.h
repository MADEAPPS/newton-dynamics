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
#ifndef __RENDER_SHADER_BLOCK_H__
#define __RENDER_SHADER_BLOCK_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"

class ndRenderPrimitiveImplement;

class ndRenderShaderBlock
{
	public:
	ndRenderShaderBlock();
	virtual ~ndRenderShaderBlock();

	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) = 0;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const = 0;

	protected:
	void EndParameters();
	virtual void SetParameters(GLuint shader);

	GLuint m_shader;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderSetZbufferCleanBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;
	GLint m_viewModelProjectionMatrix;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateShadowMapBlock : public ndRenderShaderSetZbufferCleanBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateInstanceShadowMapBlock : public ndRenderShaderGenerateShadowMapBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;

	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDebugFlatShadedDiffusedBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_diffuseColor;
	GLint m_directionalLightAmbient;
	GLint m_directionalLightIntesity;
	GLint m_directionalLightDirection;
	GLint m_projectMatrixLocation;
	GLint m_viewModelMatrixLocation;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderDebugWireframeDiffuseBlock : public ndRenderShaderDebugFlatShadedDiffusedBlock
{
	public:
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedColorBlock : public ndRenderShaderDebugFlatShadedDiffusedBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_texture;
	GLint m_cameraToWorld;
	GLint m_specularColor;
	GLint m_specularAlpha;
	GLint m_environmentMap;
	GLint m_reflectionColor;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedShadowColorBlock : public ndRenderShaderOpaqueDiffusedColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_worldMatrix;
	GLint m_shadowSlices;
	GLint m_depthMapTexture;
	GLint m_directionLightViewProjectionMatrixShadow;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderTransparentDiffusedShadowColorBlock : public ndRenderShaderOpaqueDiffusedColorBlock
{
	public:
	void SetWidingMode(bool clockwise) const;
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_opacity;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderInstancedOpaqueDiffusedShadowBlock : public ndRenderShaderOpaqueDiffusedShadowColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;
};


// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderGenerateSkinShadowMapBlock : public ndRenderShaderGenerateShadowMapBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_matrixPalette;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderOpaqueDiffusedShadowSkinColorBlock : public ndRenderShaderOpaqueDiffusedShadowColorBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_matrixPalette;
};


// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderLineArrayBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	protected:
	virtual void SetParameters(GLuint shader) override;

	GLint m_viewModelProjectionMatrix;
};

#endif
