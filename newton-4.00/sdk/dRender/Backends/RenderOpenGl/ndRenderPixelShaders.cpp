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

const char* ndRenderShaderCache::m_doNothingPixel =
R""""(
	#version 450 core

	void main()
	{	
	}
)"""";

const char* ndRenderShaderCache::m_wireFramePixel =
R""""(
	#version 450 core

	in vec4 color;
	out vec4 pixelColor;

	void main()
	{	
		pixelColor = color;
	}
)"""";

const char* ndRenderShaderCache::m_skyBoxPixel =
R""""(
	#version 450 core

	in vec3 texCoord;
	out vec4 pixelColor;
	uniform samplerCube cubemap;

	void main (void) 
	{	
		pixelColor = texture(cubemap, texCoord);
	}
)"""";

const char* ndRenderShaderCache::m_debugFlatShadedDiffusePixel =
R""""(
	#version 450 core

	uniform vec3 diffuseColor;
	uniform vec3 directionalLightAmbient;
	uniform vec3 directionalLightIntesity;
	uniform vec3 directionalLightDirection;

	in vec3 posit;
	in vec3 normal;

	out vec4 pixelColor;
	
	// implement a simple blinn model
	void main()
	{
		vec3 normalDir = normalize (normal);

		// calculate emisive, just a constant;
		vec3 emissive = diffuseColor * directionalLightAmbient;

		// calculate lambert diffuse component
		float diffuseReflection = max (dot (normalDir, directionalLightDirection), 0.0);
		vec3 diffuse = diffuseColor * directionalLightIntesity * diffuseReflection;
	
		// add all contributions
		vec3 color = emissive + diffuse;

		pixelColor = vec4(color, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffusePixel =
R""""(
	#version 450 core

	layout(binding = 0) uniform sampler2D texture0;
	layout(binding = 1) uniform samplerCube environmentMap;

	uniform mat4 cameraToWorld;
	uniform vec3 diffuseColor;
	uniform vec3 specularColor;
	uniform vec3 reflectionColor;

	uniform vec3 directionalLightAmbient;
	uniform vec3 directionalLightIntesity;
	uniform vec3 directionalLightDirection;
	uniform float specularAlpha;

	in vec3 posit;
	in vec3 normal;
	in vec2 uv;

	out vec4 pixelColor;
	
	// implement a simple blinn model
	void main()
	{
		vec3 normalDir = normalize (normal);

		// calculate emisive, just a constant;
		vec3 emissive = diffuseColor * directionalLightAmbient;

		// calculate lambert diffuse component
		float diffuseReflection = max (dot (normalDir, directionalLightDirection), 0.0);
		vec3 diffuse = diffuseColor * directionalLightIntesity * diffuseReflection;

		// calculate Blinn specular component
		vec3 cameraDir = - normalize(posit);
		vec3 blinnDir = normalize(cameraDir + directionalLightDirection);
		float reflectionSign = (diffuseReflection >= 0.01) ? 1.0 : 0.0;
		float specularReflection = reflectionSign * pow(max (dot (normalDir, blinnDir), 0.0), specularAlpha);
		vec3 specular = specularColor * directionalLightIntesity * specularReflection;

		// calculate reflection	
		vec4 reflectionDir = vec4(normalDir * (2.0 * dot(cameraDir, normalDir)) - cameraDir, 0.0f);
		vec3 worldSpaceReflectionDir = vec3(cameraToWorld * reflectionDir);
		vec3 reflection = directionalLightAmbient * reflectionColor * vec3(texture(environmentMap, worldSpaceReflectionDir));
//reflection = vec3 (0, 0, 0);

		// add all contributions
		//vec3 color = vec3(1.0, 0.0, 0.0);
		//vec3 color = reflection;
		//vec3 color = emissive + diffuse;
		//vec3 color = emissive + specular;
		vec3 color = reflection + (emissive + diffuse + specular) * vec3 (texture(texture0, uv));

		pixelColor = vec4(color, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffuseShadowPixel =
R""""(
	#version 450 core

	layout(binding = 0) uniform sampler2D texture0;
	layout(binding = 1) uniform sampler2D shadowMapTexture;
	layout(binding = 2) uniform samplerCube environmentMap;

	uniform vec3 diffuseColor;
	uniform vec3 specularColor;
	uniform vec3 reflectionColor;

	uniform vec3 directionalLightAmbient;
	uniform vec3 directionalLightIntesity;
	uniform vec3 directionalLightDirection;
	uniform float specularAlpha;

	uniform mat4 cameraToWorld;
	uniform mat4 directionaLightViewProjectionMatrix[4];
	uniform vec4 shadowSlices; 

	in vec4 worldPosit;
	in vec3 posit;
	in vec3 normal;
	in vec2 uv;

	out vec4 pixelColor;
	
	// implement a simple Blinn model
	void main()
	{
		vec3 normalDir = normalize (normal);

		// calculate emisive, just a constant;
		vec3 emissive = diffuseColor * directionalLightAmbient;

		// calculate Lambert diffuse component
		float diffuseReflection = max (dot (normalDir, directionalLightDirection), 0.0);
		vec3 diffuse = diffuseColor * directionalLightIntesity * diffuseReflection;

		// calculate Blinn specular component
		vec3 cameraDir = - normalize(posit);
		vec3 blinnDir = normalize(cameraDir + directionalLightDirection);
		float reflectionSign = (diffuseReflection >= 0.01) ? 1.0 : 0.0;
		float specularReflection = reflectionSign * pow(max (dot (normalDir, blinnDir), 0.0), specularAlpha);
		vec3 specular = specularColor * directionalLightIntesity * specularReflection;

		vec3 color = specular + diffuse;

		// calculate the shadow tile
		int index = 4;
		if (gl_FragCoord.z < shadowSlices.w)
		{
			index = 3;
			if (gl_FragCoord.z < shadowSlices.z)
			{
				index = 2;
				if (gl_FragCoord.z < shadowSlices.y)
				{
					index = 1;
					if (gl_FragCoord.z < shadowSlices.x)
					{
						index = 0;
					}
				}
			}
		}

		if (index < 4)
		{
			vec4 pointInDepthMapSpace = directionaLightViewProjectionMatrix[index] * worldPosit;

			float textDepth = texture(shadowMapTexture, vec2(pointInDepthMapSpace)).x;
			if (textDepth < pointInDepthMapSpace.z)
			{
				color = vec3(0.0f, 0.0f, 0.0f);
				// enable this to show the shadows sections
				#if 0
					if (index < 3)
					{
						color[index] = 1.0f;
					}
					else
					{
						color[0] = 1.0f;
						color[1] = 1.0f;
					}
				#endif
			}
		}
		// calculate reflection	
		//vec3 reflectionDir = normalDir * (2.0 * dot(cameraDir, normalDir)) - cameraDir;
		//vec3 reflection = directionalLightAmbient * reflectionColor * vec3(texture(environmentMap, reflectionDir));
		vec4 reflectionDir = vec4(normalDir * (2.0 * dot(cameraDir, normalDir)) - cameraDir, 0.0f);
		vec3 worldSpaceReflectionDir = vec3(cameraToWorld * reflectionDir);
		vec3 reflection = directionalLightAmbient * reflectionColor * vec3(texture(environmentMap, worldSpaceReflectionDir));
		
		// add all contributions
		color = color + emissive;
		color = color * vec3 (texture(texture0, uv));
		color = color + reflection;
		
		pixelColor = vec4(color, 1.0);
	}

)"""";

const char* ndRenderShaderCache::m_directionalDiffuseTransparentPixel =
R""""(
	#version 450 core

	layout(binding = 0) uniform sampler2D texture0;
	layout(binding = 1) uniform samplerCube environmentMap;

	uniform vec3 diffuseColor;
	uniform vec3 specularColor;
	uniform vec3 reflectionColor;

	uniform vec3 directionalLightAmbient;
	uniform vec3 directionalLightIntesity;
	uniform vec3 directionalLightDirection;

	uniform float opacity;
	uniform float specularAlpha;

	in vec3 posit;
	in vec3 normal;
	in vec2 uv;

	out vec4 pixelColor;
	
	// implement a simple Blinn model
	void main()
	{
		vec3 normalDir = normalize (normal);

		// calculate emisive, just a constant;
		vec3 emissive = diffuseColor * directionalLightAmbient;

		// calculate Lambert diffuse component
		float diffuseReflection = max (dot (normalDir, directionalLightDirection), 0.0);
		vec3 diffuse = diffuseColor * directionalLightIntesity * diffuseReflection;

		// calculate Blinn specular component
		vec3 cameraDir = - normalize(posit);
		vec3 blinnDir = normalize(cameraDir + directionalLightDirection);
		float reflectionSign = (diffuseReflection >= 0.01) ? 1.0 : 0.0;
		float specularReflection = reflectionSign * pow(max (dot (normalDir, blinnDir), 0.0), specularAlpha);
		vec3 specular = specularColor * directionalLightIntesity * specularReflection;

		// calculate reflection	
		vec3 reflectionDir = normalDir * (2.0 * dot(cameraDir, normalDir)) - cameraDir;
		vec3 reflection = reflectionColor * vec3(texture(environmentMap, reflectionDir));
reflection = vec3 (0, 0, 0);

		// add all contributions
		vec3 color = reflection + (emissive + diffuse + specular) * vec3 (texture(texture0, uv));

		pixelColor = vec4(color, opacity);
	}

)"""";
