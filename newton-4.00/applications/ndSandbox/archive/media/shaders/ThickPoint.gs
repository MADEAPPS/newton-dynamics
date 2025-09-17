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
uniform vec4 pixelSize[4];

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in vec4 color[];
in vec4 origin[];

out vec4 pixelColor;

void GetPosit(vec4 point, float wDist, int index)
{
	vec4 quadPoint = (point + pixelSize[index]) * wDist;
	gl_Position = quadPoint;
	EmitVertex();
}

void main()
{	
	vec4 pointInScreenSpace = projectionMatrix * origin[0];
	float wDist = pointInScreenSpace.w;
	pointInScreenSpace = pointInScreenSpace * (1.0/wDist);

	pixelColor = color[0];
	GetPosit(pointInScreenSpace, wDist, 0);
	GetPosit(pointInScreenSpace, wDist, 1);
	GetPosit(pointInScreenSpace, wDist, 2);
	GetPosit(pointInScreenSpace, wDist, 3);
	EndPrimitive();
}