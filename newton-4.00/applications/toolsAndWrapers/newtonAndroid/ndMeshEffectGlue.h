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

#ifndef _ND_MESH_EFFECT_GLUE_H_
#define _ND_MESH_EFFECT_GLUE_H_

#include "ndMeshEffect.h"
#include "ndShapeInstanceGlue.h"

class ndMeshEffectGlue : public ndMeshEffect
{
	public:
	ndMeshEffectGlue()
		:ndMeshEffect()
	{
	}

	ndMeshEffectGlue(const ndShapeInstanceGlue& shapeInstance)
		:ndMeshEffect(shapeInstance)
	{
	}

	int GetVertexSize()
	{
		return int (ndMeshEffect::GetPropertiesCount());
	}
};

#endif 

