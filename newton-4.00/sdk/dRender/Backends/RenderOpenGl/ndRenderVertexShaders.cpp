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

const char* ndRenderShaderCache::m_wireFrameVertex =
R""""(
	#version 450 core

	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_vertexColor;

	out vec4 color;
	uniform mat4 viewModelProjectionMatrix;
 
	void main() 
	{	
		color = vec4(in_vertexColor, 1.0f);
		gl_Position = viewModelProjectionMatrix * vec4(in_position, 1.0f);
	}
)"""";

const char* ndRenderShaderCache::m_setZbufferVertex =
R""""(
	#version 450 core

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

const char* ndRenderShaderCache::m_generateInstancedShadowMapVertex =
R""""(
	#version 450 core

	// using the same vertex buffer
	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;
	layout(location = 3) in mat4 in_matrixPalette;

	uniform mat4 viewModelProjectionMatrix;
	void main()
	{
		vec4 instancePosit = in_matrixPalette * vec4(in_position, 1.0);
		gl_Position = viewModelProjectionMatrix * instancePosit;
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
	uniform mat4 modelWorldMatrix;

	out vec4 worldPosit;
	out vec3 posit;
	out vec3 normal;
	out vec2 uv;

	void main()
	{
		vec4 instancePosit = in_matrixPalette * vec4(in_position, 1.0);
		vec3 instanceNormal = vec3(in_matrixPalette * vec4(in_normal, 0.0));

		worldPosit = modelWorldMatrix * instancePosit;
		posit = vec3(viewModelMatrix * instancePosit);
		normal = vec3(normalize(viewModelMatrix * vec4(instanceNormal, 0.0)));

		uv = in_uv;
		gl_Position = projectionMatrix * vec4(posit, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffuseShadowSkinVertex =
R""""(
	#version 450 core

	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;
	layout(location = 3) in vec4 in_boneWeights;
	layout(location = 4) in ivec4 in_boneIndices;

	uniform mat4 viewModelMatrix;
	uniform mat4 projectionMatrix;
	uniform mat4 modelWorldMatrix;
	uniform mat4 matrixPalette[128];

	out vec4 worldPosit;
	out vec3 posit;
	out vec3 normal;
	out vec2 uv;

	void main()
	{
		vec4 pointNormal = vec4(in_normal, 0.0f);
		vec4 pointVertex = vec4(in_position, 1.0f);

		vec4 weightedNormal = vec4 (0.0f, 0.0f, 0.0f, 0.0f);	
		vec4 weightedVertex = vec4 (0.0f, 0.0f, 0.0f, 1.0f);	
		for (int i = 0; i < 4; i++) 
		{
			float weigh = in_boneWeights[i];
			int matrixIndex = in_boneIndices[i];
			weightedVertex += matrixPalette[matrixIndex] * pointVertex * weigh;
			weightedNormal += matrixPalette[matrixIndex] * pointNormal * weigh;
		}
		weightedVertex.w = 1.0;
		weightedNormal = normalize (weightedNormal);

		worldPosit = modelWorldMatrix * weightedVertex;
		posit = vec3 (viewModelMatrix * weightedVertex);
		normal = vec3 (viewModelMatrix * weightedNormal);

		uv = in_uv;
		gl_Position = projectionMatrix * vec4(posit, 1.0);
	}
)"""";

const char* ndRenderShaderCache::m_generateShadowMapSkinVertex =
R""""(
	#version 450 core

	// using the same vertex buffer
	layout(location = 0) in vec3 in_position;
	layout(location = 1) in vec3 in_normal;
	layout(location = 2) in vec2 in_uv;
	layout(location = 3) in vec4 in_boneWeights;
	layout(location = 4) in ivec4 in_boneIndices;

	uniform mat4 viewModelProjectionMatrix;
	uniform mat4 matrixPalette[128];

	void main()
	{
		vec4 pointVertex = vec4(in_position, 1.0f);
		vec4 weightedVertex = vec4 (0.0f, 0.0f, 0.0f, 1.0f);	
		for (int i = 0; i < 4; i++) 
		{
			float weigh = in_boneWeights[i];
			int matrixIndex = in_boneIndices[i];
			weightedVertex += matrixPalette[matrixIndex] * pointVertex * weigh;
		}
		weightedVertex.w = 1.0;

		gl_Position = viewModelProjectionMatrix * weightedVertex;
	}
)"""";

