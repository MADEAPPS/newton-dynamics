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

const char* ndRenderShaderCache::m_directionalDiffusePixel =
R""""(
	#version 450 core

	uniform sampler2D texture0;

	uniform vec3 diffuseColor;
	uniform vec3 specularColor;
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

		// add all contributions
		//vec3 color = emissive + diffuse;
		//vec3 color = emissive + specular;
		vec3 color = (emissive + diffuse + specular) * vec3 (texture(texture0, uv));

		pixelColor = vec4(color, 1.0);
	}

)"""";
