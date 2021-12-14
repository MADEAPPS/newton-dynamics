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

#ifndef __ND_PERLIN_NOISE_H__
#define __ND_PERLIN_NOISE_H__

#include "ndCoreStdafx.h"

D_CORE_API ndFloat32 dPerlinNoise(ndFloat32 x);
D_CORE_API ndFloat32 dPerlinNoise(ndFloat32 x, ndFloat32 y);
D_CORE_API ndFloat32 dPerlinNoise(ndFloat32 x, ndFloat32 y, ndFloat32 z);

D_CORE_API ndFloat32 BrownianMotion(ndInt32 octaves, ndFloat32 persistence, ndFloat32 x);
D_CORE_API ndFloat32 BrownianMotion(ndInt32 octaves, ndFloat32 persistence, ndFloat32 x, ndFloat32 y);
D_CORE_API ndFloat32 BrownianMotion(ndInt32 octaves, ndFloat32 persistence, ndFloat32 x, ndFloat32 y, ndFloat32 z);
#endif

