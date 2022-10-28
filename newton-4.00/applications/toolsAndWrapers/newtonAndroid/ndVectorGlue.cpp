/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#include "ndNewton.h"
#include "ndVectorGlue.h"

ndVectorGlue::ndVectorGlue()
	:ndVector()
{
}

ndVectorGlue::ndVectorGlue(const ndVector& vector)
	:ndVector(vector)
{
}

ndVectorGlue::ndVectorGlue(const ndVectorGlue& vector)
	:ndVector(vector)
{
}

ndVectorGlue::ndVectorGlue(float x, float y, float z, float w)
	:ndVector(ndFloat32(x), ndFloat32(y), ndFloat32(z), ndFloat32(w))
{
}

float ndVectorGlue::Get(int i) const
{
	return (*this)[i];
}

void ndVectorGlue::Set(int i, float value)
{
	(*this)[i] = value;
}
