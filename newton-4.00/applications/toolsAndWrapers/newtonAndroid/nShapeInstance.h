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

#ifndef _N_SHAPE_INSTANCE_H_
#define _N_SHAPE_INSTANCE_H_

#include "nShape.h"
#include "ndShape.h"
#include "ndShapeInstance.h"

class nShapeInstance : public ndShapeInstance
{
	public:
	nShapeInstance(nShape* const shape)
		:ndShapeInstance(shape->m_shape)
	{
	}

	~nShapeInstance()
	{
	}
};

#endif 

