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
#ifndef __SHADER_CACHE_H__
#define __SHADER_CACHE_H__

#include "ndSandboxStdafx.h"
#include "ndOpenGlUtil.h"

class ndShaderCache
{
	public:
	ndShaderCache();
	~ndShaderCache();

	void Cleanup();
	bool CreateAllEffects();
	
	private:
	void LoadShaderCode (const char* const name, char* const buffer);
	GLuint CreateShaderEffect (const char* const vertexShader, const char* const pixelShader, const char* const geometryShader = nullptr);

	public:
	union
	{
		struct
		{
			GLuint m_skyBox;
			GLuint m_wireFrame;
			GLuint m_colorPoint;
			GLuint m_flatShaded;
			GLuint m_thickPoints;
			GLuint m_zBufferDebug;
			GLuint m_texturedDecal;
			GLuint m_diffuseEffect;
			GLuint m_spriteSpheres;
			GLuint m_diffuseDebrisEffect;
			GLuint m_diffuseIntanceEffect;
			GLuint m_skinningDiffuseEffect;

			// shadow programs
			GLuint m_shadowMaps;
			GLuint m_diffuseShadowEffect;
		};
		GLuint m_shaders[128];
	};
};


#endif
