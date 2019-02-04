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

attribute vec4 boneWeights;
attribute vec4 boneIndices;
uniform mat4 matrixPallete[128];

void main()
{
	vec3 weightedNormal = vec3 (0.0, 0.0, 0.0);	
	vec4 weightedVertex = vec4 (0.0, 0.0, 0.0, 1.0);	

	vec4 pointVertex = gl_Vertex;
	vec4 pointNormal = vec4(gl_Normal.x, gl_Normal.y, gl_Normal.z, 0.0);
	for (int i = 0; i < 4; i++) {
		int matrixIndex = int (boneIndices[i]);
		weightedVertex += matrixPallete[matrixIndex] * pointVertex * boneWeights[i];
		weightedNormal += vec3 (matrixPallete[matrixIndex] * pointNormal * boneWeights[i]);
	}
	weightedVertex.w = 1.0;
	weightedNormal = normalize (weightedNormal);

	normal = gl_NormalMatrix * weightedNormal;
	position = vec3 (gl_ModelViewMatrix * weightedVertex);

	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = gl_ModelViewProjectionMatrix * weightedVertex;
} 

