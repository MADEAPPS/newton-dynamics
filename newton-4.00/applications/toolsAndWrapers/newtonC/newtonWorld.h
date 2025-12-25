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
#ifndef __NEWTON_WORLD_H__
#define __NEWTON_WORLD_H__


//#include "newtonConfig.h"
#include "newtonStdafx.h"


class NewtonWorld: public ndWorld
{
	public:
	NewtonWorld();
	~NewtonWorld();

	void Update(ndFloat32 timestep);

	void SetSubSteps(ndInt32 substeps);
	void SetIterations(ndInt32 iterations);
};

#endif