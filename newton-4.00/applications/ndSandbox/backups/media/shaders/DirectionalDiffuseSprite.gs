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
uniform vec4 uvSize[4];

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;

in vec4 color[];
in vec4 origin[];

out vec4 quadUV;
out vec4 quadColor;
out vec4 spriteOrigin;

void GetPosit(int index)
{
	quadUV = uvSize[index];
	spriteOrigin = origin[0];

	vec4 p = spriteOrigin + quadSize[index];
	gl_Position = projectionMatrix * p;
	EmitVertex();
}

void main()
{	
	quadColor = color[0];
	GetPosit(0);
	GetPosit(1);
	GetPosit(2);
	GetPosit(3);
	EndPrimitive();
}
