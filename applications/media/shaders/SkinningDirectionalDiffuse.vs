/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#version 120

uniform sampler2D texture;

varying vec3 normal;
varying vec3 position;

//uniform mat4 matrixPallete[128];
//attribute vec4 bonesIndices;
//attribute vec4 bonesWeights;
//attribute vec4 textureEnableOnOff;

//varying vec4 diffuse;
//varying vec4 ambient;
//varying vec3 normal;
//varying vec3 lightDir;
//varying vec3 halfVector;
//varying vec4 textureColorOnOff;

void main()
{
/*	
	//convert the bone index from floats to ints indices
	ivec4 IntBoneIndex = ivec4(bonesIndices);

	// first transform the normal into eye space and normalize the result 
//	normal = normalize(gl_NormalMatrix * gl_Normal);
	vec4 tmpNormal = vec4 (gl_Normal[0], gl_Normal[1], gl_Normal[2], 0);
	tmpNormal = normalize(matrixPallete[IntBoneIndex[0]] * tmpNormal);
	tmpNormal = gl_ModelViewMatrix * tmpNormal;
	normal = vec3 (tmpNormal[0], tmpNormal[1], tmpNormal[2]);
	
	// now normalize the light direction
	lightDir = normalize(vec3(gl_LightSource[0].position));

	// Normalize the halfVector to pass it to the fragment shader 
	halfVector = normalize(gl_LightSource[0].halfVector.xyz);
	
	// Compute the diffuse, ambient and globalAmbient terms 
	diffuse = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
	ambient  = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
	ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;

	vec4 posit = vec4 (0, 0, 0, 0);	
	for (int i = 0; i < 4; i++) {
		// this iofail		
		posit += matrixPallete[IntBoneIndex[i]] * gl_Vertex * bonesWeights[i];
	}

	textureColorOnOff = textureEnableOnOff; 
	gl_TexCoord[0] = gl_MultiTexCoord0;	
	gl_Position = gl_ModelViewProjectionMatrix * posit;
*/

	// get normal in camera space
	normal = gl_NormalMatrix * gl_Normal;

	// get position is camera space
	position = vec3 (gl_ModelViewMatrix * gl_Vertex);

	// get position is perective space
//	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_TexCoord[0] = vec4 (0.0, 0.0, 0.0, 0.0);
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
} 


