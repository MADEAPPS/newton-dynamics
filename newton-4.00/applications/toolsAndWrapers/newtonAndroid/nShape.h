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

#ifndef _N_SHAPE_H_
#define _N_SHAPE_H_

#include "ndShape.h"

class nShape
{
	protected:
	nShape(ndShape* const shape)
		:m_shape(shape)
	{
	}

	virtual ~nShape()
	{
	}

	ndShape* m_shape;
	friend class ndShapeInstanceGlue;
};

#endif 

