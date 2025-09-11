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
#ifndef __RENDER_SHADER_CACHE_H__
#define __RENDER_SHADER_CACHE_H__

#include "ndRenderStdafx.h"
#include "ndRenderContext.h"

class ndRenderPrimitiveMeshImplement;

class ndRenderShaderBlock
{
	public:
	ndRenderShaderBlock();
	virtual ~ndRenderShaderBlock();

	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) = 0;
	virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const = 0;

	GLuint m_shader;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderSetZbufferCleanBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

	GLint viewModelProjectionMatrix;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderGenerateShadowMapBlock : public ndRenderSetZbufferCleanBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;

	void EndRender();
	void BeginRender();
	virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;
};

// *********************************************************************
// 
// *********************************************************************
class ndDebugFlatShadedDiffusedBlock : public ndRenderShaderBlock
{
	public:
	virtual void GetShaderParameters(const ndRenderShaderCache* const shaderCache) override;
	virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;

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
class ndDebugWireframeDiffuseBlock : public ndDebugFlatShadedDiffusedBlock
{
	public:
	virtual void Render(const ndRenderPrimitiveMeshImplement* const self, const ndRender* const render, const ndMatrix& modelMatrix) const override;
};

// *********************************************************************
// 
// *********************************************************************
class ndRenderShaderCache
{
	public:
	ndRenderShaderCache();
	~ndRenderShaderCache();
	
	private:
	void Cleanup();
	bool CreateAllEffects();
	GLuint CreateShaderEffect (const char* const vertexShader, const char* const pixelShader, const char* const geometryShader = nullptr);

	public:
	union
	{
		struct
		{
			GLuint m_skyBoxEffect;
			GLuint m_diffuseEffect;
			GLuint m_setZbufferEffect;
			GLuint m_diffuseShadowEffect;
			GLuint m_diffuseIntanceEffect;
			GLuint m_diffuseTransparentEffect;
			GLuint m_generateShadowMapsEffect;
			GLuint m_debugFlatShadedDiffuseEffect;
		};
		GLuint m_shaders[128];
	};

	static const char* m_skyBoxVertex;
	static const char* m_setZbufferVertex;
	static const char* m_generateShadowMapVertex;
	static const char* m_directionalDiffuseVertex;
	static const char* m_debugFlatShadedDiffuseVertex;
	static const char* m_directionalDiffuseShadowVertex;
	static const char* m_directionalDiffuseInstanceVertex;

	static const char* m_skyBoxPixel;
	static const char* m_doNothingPixel;
	static const char* m_directionalDiffusePixel;
	static const char* m_debugFlatShadedDiffusePixel;
	static const char* m_directionalDiffuseShadowPixel;
	static const char* m_directionalDiffuseTransparentPixel;

	friend class ndRenderContext;
	friend class ndRenderSetZbufferCleanBlock;
	friend class ndDebugWireframeDiffuseBlock;
	friend class ndDebugFlatShadedDiffusedBlock;
};
#endif
