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

#ifndef _ND_SHAPE_INSTANCE_GLUE_H_
#define _ND_SHAPE_INSTANCE_GLUE_H_

#include "ndShape.h"
#include "ndShapeInstance.h"

class ndShapeInstanceGlue: public ndContainersFreeListAlloc<ndShapeInstanceGlue>
{
	protected:
	ndShapeInstanceGlue(ndShape* const shape)
		:ndContainersFreeListAlloc<ndShapeInstanceGlue>()
		,m_shapeInstance(new ndShapeInstance(shape))
		,m_ownData(true)
	{
	}

	ndShapeInstanceGlue(ndShapeInstance* const shapeInstance)
		:ndContainersFreeListAlloc<ndShapeInstanceGlue>()
		,m_shapeInstance(shapeInstance)
		,m_ownData(false)
	{
	}

	~ndShapeInstanceGlue()
	{
		if (m_ownData)
		{
			delete m_shapeInstance;
		}
	}

	ndShapeInstance* m_shapeInstance;
	bool m_ownData;
	friend class ndRigidBodyGlue;
	friend class ndMeshEffectGlue;
};

#endif 

