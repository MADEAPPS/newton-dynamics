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

#include "ndShapeGlue.h"
#include "ndShapeInstance.h"

class ndShapeInstanceGlue
{
	public:
	ndShapeInstanceGlue(ndShapeGlue* const shape)
		:m_instance(new ndShapeInstance(shape->m_shape))
	{
	}

	ndShapeInstanceGlue(ndShapeInstance* const shapeInstance)
		:m_instance(shapeInstance)
	{
		m_instance->m_shape->AddRef();
	}

	//ndShapeInstanceGlue(ndShapeInstanceGlue* const shapeInstance)
	//	:m_instance(new ndShapeInstance(*(*shapeInstance->m_instance)))
	//{
	//}

	~ndShapeInstanceGlue()
	{
	}

	private:
	ndSharedPtr<ndShapeInstance> m_instance;
	friend class ndRigidBodyGlue;
	friend class ndMeshEffectGlue;
};

#endif 

