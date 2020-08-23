/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// HeightFieldPrimitive.h: interface for the HeightFieldPrimitive class.
//
//////////////////////////////////////////////////////////////////////

#ifndef __HeightFieldPrimitive_H__
#define __HeightFieldPrimitive_H__

#define HEIGHTFIELD_DEFAULT_SIZE		7
//#define HEIGHTFIELD_DEFAULT_SIZE		6
#define HEIGHTFIELD_DEFAULT_CELLSIZE	8.0f

class DemoEntityManager;
NewtonBody* CreateHeightFieldTerrain (DemoEntityManager* const scene, int sizeInPowerOfTwos, dFloat cellSize, dFloat elevationScale, dFloat roughness, dFloat maxElevation, dFloat minElevation);


#endif 

