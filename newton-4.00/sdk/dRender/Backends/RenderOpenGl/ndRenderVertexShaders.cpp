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
#include "ndRenderShaderCache.h"


const char* ndRenderShaderCache::m_skyBoxVertex =
R""""(
	#version 450 core

	uniform mat4 invViewModelProjectionTextureMatrix;
	in vec3 vertex;
	out vec3 texCoord;
 
	void main() 
	{	
		texCoord = vec3 (invViewModelProjectionTextureMatrix * vec4(vertex, 1.0));
		gl_Position = vec4(vertex, 1.0);
	}
)"""";

const char* ndRenderShaderCache::m_setZbufferVertex =
R""""(
	#version 450 core

	// using the same vertex buffer
	layout(location = 0) in vec3 in_position;

	uniform mat4 viewModelProjectionMatrix;
	void main()
	{
		gl_Position = viewModelProjectionMatrix * vec4(in_position, 1.0);
	}
)"""";


const char* ndRenderShaderCache::m_generateShadowMapVertex =
R""""(
	#version 450 core

	// using the same vertex buffer
	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;

	uniform mat4 viewModelProjectionMatrix;
	void main()
	{
		gl_Position = viewModelProjectionMatrix * vec4(in_position, 1.0);
	}
)"""";

const char* ndRenderShaderCache::m_debugFlatShadedDiffuseVertex =
R""""(
	#version 450 core

	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;

	uniform mat4 viewModelMatrix;
	uniform mat4 projectionMatrix;

	out vec3 posit;
	out vec3 normal;

	void main()
	{
		posit = vec3(viewModelMatrix * vec4(in_position, 1.0));
		normal = vec3(normalize(viewModelMatrix * vec4(in_normal, 0.0)));
		gl_Position = projectionMatrix * vec4(posit, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffuseVertex =
R""""(
	#version 450 core

	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;

	//uniform mat4 normalMatrix;
	uniform mat4 viewModelMatrix;
	uniform mat4 projectionMatrix;

	out vec3 posit;
	out vec3 normal;
	out vec2 uv;

	void main()
	{
		posit = vec3(viewModelMatrix * vec4(in_position, 1.0));
		//normal = vec3(normalize(normalMatrix * vec4(in_normal, 0.0)));
		normal = vec3(normalize(viewModelMatrix * vec4(in_normal, 0.0)));
		uv = in_uv;
		gl_Position = projectionMatrix * vec4(posit, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffuseShadowVertex =
R""""(
	#version 450 core

	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;

	uniform mat4 viewModelMatrix;
	uniform mat4 projectionMatrix;
	uniform mat4 modelWorldMatrix;

	out vec4 worldPosit;
	out vec3 posit;
	out vec3 normal;
	out vec2 uv;

	void main()
	{
		worldPosit = modelWorldMatrix * vec4(in_position, 1.0);
		posit = vec3(viewModelMatrix * vec4(in_position, 1.0));
		normal = vec3(normalize(viewModelMatrix * vec4(in_normal, 0.0)));
		uv = in_uv;
		gl_Position = projectionMatrix * vec4(posit, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffuseInstanceVertex =
R""""(
	#version 450 core

	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;
	layout(location = 3) in mat4 in_matrixPalette;

	uniform mat4 viewModelMatrix;
	uniform mat4 projectionMatrix;

	out vec3 posit;
	out vec3 normal;
	out vec2 uv;

	void main()
	{
		vec4 instancePosit = in_matrixPalette * vec4(in_position, 1.0);
		vec3 instanceNormal = vec3(in_matrixPalette * vec4(in_normal, 0.0));

		posit = vec3(viewModelMatrix * instancePosit);
		normal = vec3(normalize(viewModelMatrix * vec4(instanceNormal, 0.0)));

		uv = in_uv;
		gl_Position = projectionMatrix * vec4(posit, 1.0);
	}

)"""";

