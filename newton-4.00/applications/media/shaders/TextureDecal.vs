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

attribute vec3 vertexPosition;
attribute vec2 vertexTexCoord;

void main()
{	
	//gl_TexCoord[0] = gl_MultiTexCoord0;
	//gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

	gl_TexCoord[0] = vertexTexCoord;
	gl_Position = matrixModelViewProjection * vec4(vertexPosition, 1.0);
} 


