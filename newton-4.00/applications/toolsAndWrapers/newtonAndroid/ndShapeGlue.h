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

#ifndef _ND_SHAPE_GLUE_H_
#define _ND_SHAPE_GLUE_H_

#include "ndShapeBox.h"

class ndShapeGlue
{
	protected:
	ndShapeGlue(ndShape* const shape)
		:m_shape(shape)
	{
		m_shape->AddRef();
	}

	~ndShapeGlue()
	{
		ndAssert(m_shape);
		m_shape->Release();
	}

	ndShape* m_shape;
	friend class ndShapeInstanceGlue;
};

#endif 

