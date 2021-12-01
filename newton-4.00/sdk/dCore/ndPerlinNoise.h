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

D_CORE_API dFloat32 dPerlinNoise(dFloat32 x);
D_CORE_API dFloat32 dPerlinNoise(dFloat32 x, dFloat32 y);
D_CORE_API dFloat32 dPerlinNoise(dFloat32 x, dFloat32 y, dFloat32 z);

D_CORE_API dFloat32 BrownianMotion(dInt32 octaves, dFloat32 persistence, dFloat32 x);
D_CORE_API dFloat32 BrownianMotion(dInt32 octaves, dFloat32 persistence, dFloat32 x, dFloat32 y);
D_CORE_API dFloat32 BrownianMotion(dInt32 octaves, dFloat32 persistence, dFloat32 x, dFloat32 y, dFloat32 z);
#endif

