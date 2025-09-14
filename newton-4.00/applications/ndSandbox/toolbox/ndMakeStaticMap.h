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

#ifndef _D_MAKE_STATIC_MAP_H_
#define _D_MAKE_STATIC_MAP_H_

#include "ndSandboxStdafx.h"
class ndDemoEntityManager;

//ndBodyKinematic* BuildPlayArena(ndDemoEntityManager* const scene, bool kinematic = false);
//ndBodyKinematic* BuildFlatPlane(ndDemoEntityManager* const scene, bool optimized, bool kinematic = false);
//ndBodyKinematic* BuildFloorBox(ndDemoEntityManager* const scene, const ndMatrix& location, bool kinematic = false);
//ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, bool optimized, bool kinematic = false);
//ndBodyKinematic* BuildSplineTrack(ndDemoEntityManager* const scene, const char* const meshName, bool optimized, bool kinematic = false);
//ndBodyKinematic* BuildGridPlane(ndDemoEntityManager* const scene, ndInt32 grids, ndFloat32 gridSize, ndFloat32 perturbation, bool kinematic = false);

ndSharedPtr<ndBody> BuildPlayArena(ndDemoEntityManager* const scene, bool kinematic = false);

ndSharedPtr<ndBody> BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName, bool optimized, bool kinematic = false);
ndSharedPtr<ndBody> BuildGridPlane(ndDemoEntityManager* const scene, ndInt32 grids, ndFloat32 gridSize, ndFloat32 perturbation, bool kinematic = false);

ndSharedPtr<ndBody> BuildFlatPlane(ndDemoEntityManager* const scene, const ndMatrix& location, const char* const textureName, bool optimized, bool kinematic = false);
ndSharedPtr<ndBody> BuildFloorBox(ndDemoEntityManager* const scene, const ndMatrix& location, const char* const textureName, ndFloat32 uvTiling, bool kinematic = false);

#endif