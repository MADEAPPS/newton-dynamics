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

#version 330 core

uniform mat4 projectionMatrix;
uniform vec4 shadeColor;
uniform vec4 quadSize[4];

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in vec4 color[];
//in vec4 origin[];
out vec4 quadColor;

void main()
{	
	//vec4 posit = origin[0];
	vec4 posit = gl_in[0].gl_Position;

	quadColor = color[0];

	vec4 quad0 = posit + quadSize[0];
	gl_Position = projectionMatrix * quad0;
	EmitVertex();

	vec4 quad1 = posit + quadSize[1];
	gl_Position = projectionMatrix * quad1;
	EmitVertex();

	vec4 quad2 = posit + quadSize[2];
	gl_Position = projectionMatrix * quad2;
	EmitVertex();

	vec4 quad3 = posit + quadSize[3];
	gl_Position = projectionMatrix * quad3;
	EmitVertex();

	EndPrimitive();
}