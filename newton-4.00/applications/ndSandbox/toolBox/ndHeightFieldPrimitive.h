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

#ifndef __HeightFieldPrimitive_H__
#define __HeightFieldPrimitive_H__

#include "ndSandboxStdafx.h"

class ndDemoEntityManager;
void AddHeightfield(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance);
ndBodyKinematic* BuildHeightFieldTerrain(ndDemoEntityManager* const scene, const dMatrix& location = dGetIdentityMatrix());
#endif 

