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
#version 330

out vec3 normal;
out vec3 position;
out vec4 textureUV;

void main()
{	
	// get normal in camera space for ligh calculation
	normal = gl_NormalMatrix * gl_Normal;
	position = vec3 (gl_ModelViewMatrix * gl_Vertex);
	
	textureUV = gl_MultiTexCoord0;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
} 


