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
			GLuint m_lineEffect;
			GLuint m_skyBoxEffect;
			GLuint m_diffuseEffect;
			GLuint m_wireFrameEffect;
			GLuint m_setZbufferEffect;
			GLuint m_diffuseShadowEffect;
			GLuint m_diffuseShadowSkinEffect;
			GLuint m_diffuseTransparentEffect;
			GLuint m_generateShadowMapsEffect;
			GLuint m_diffuseShadowIntanceEffect;
			GLuint m_generateShadowMapsSkinEffect;
			GLuint m_debugFlatShadedDiffuseEffect;
			GLuint m_generateInstancedShadowMapsEffect;
		};
		GLuint m_shaders[128];
	};

	// vertex shaders
	static const char* m_skyBoxVertex;
	static const char* m_wireFrameVertex;
	static const char* m_setZbufferVertex;
	static const char* m_wireFrameVertexColor;
	static const char* m_generateShadowMapVertex;
	static const char* m_directionalDiffuseVertex;
	static const char* m_generateShadowMapSkinVertex;
	static const char* m_debugFlatShadedDiffuseVertex;
	static const char* m_directionalDiffuseShadowVertex;
	static const char* m_directionalDiffuseInstanceVertex;
	static const char* m_generateInstancedShadowMapVertex;
	static const char* m_directionalDiffuseShadowSkinVertex;

	// pixel shaders
	static const char* m_skyBoxPixel;
	static const char* m_doNothingPixel;
	static const char* m_wireFramePixel;
	static const char* m_directionalDiffusePixel;
	static const char* m_debugFlatShadedDiffusePixel;
	static const char* m_directionalDiffuseShadowPixel;
	static const char* m_directionalDiffuseTransparentPixel;
	

	friend class ndRenderContext;
	friend class ndRenderShaderSetZbufferCleanBlock;
	friend class ndRenderShaderDebugWireframeDiffuseBlock;
	friend class ndRenderShaderDebugFlatShadedDiffusedBlock;
};

#endif
