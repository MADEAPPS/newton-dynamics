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

varying vec3 normal;
varying vec3 position;

void main()
{	
	// get normal in camera space
	normal = gl_NormalMatrix * gl_Normal;

	// get position is camera space
	position = vec3 (gl_ModelViewMatrix * gl_Vertex);

	// get position is perective space
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
} 


