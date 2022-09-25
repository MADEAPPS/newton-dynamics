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

#ifndef _ND_VECTOR4_H_
#define _ND_VECTOR4_H_

#include "ndVector.h"

class vector4 : public ndVector
{
	public:
	vector4()
		:ndVector()
	{
	}

	vector4(float x, float y, float z, float w)
		:ndVector(ndFloat32(x), ndFloat32(y), ndFloat32(z), ndFloat32(y))
	{
	}

	float Get(int i) const
	{
		return (*this)[i];
	}

	void Set(int i, float value)
	{
		(*this)[i] = value;
	}
};


#endif 

