/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
ndSharedPtr<ndBody> BuildHeightFieldTerrain(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location = ndGetIdentityMatrix());
//void AddHeightfieldSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, ndDemoEntity* const rootEntity, const ndMatrix& matrix);
#endif 

