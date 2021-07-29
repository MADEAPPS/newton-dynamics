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

#ifndef __SceneCompoundCollision_h__
#define __SceneCompoundCollision_h__

#include "ndSandboxStdafx.h"

class ndDemoEntityManager;
ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& location = dGetIdentityMatrix());
#endif 

