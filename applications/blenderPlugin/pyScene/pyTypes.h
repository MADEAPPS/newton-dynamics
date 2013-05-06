/////////////////////////////////////////////////////////////////////////////
// Name:        puTypes.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#pragma once

struct pyTrianglePoint
{
	int vertex;
	int normal;
	int uv0;
	int uv1;
};

struct pyTriangle
{
	pyTrianglePoint p0;
	pyTrianglePoint p1;
	pyTrianglePoint p2;
	int materialIndex;
};

struct pyVertex
{
	double x;
	double y;
	double z;
};

struct pyMatrix4x4
{
	double e00, e01, e02, e03;
	double e10, e11, e12, e13;
	double e20, e21, e22, e23;
	double e30, e31, e32, e33;
};
