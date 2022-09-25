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

#ifndef _ND_WORLD_H_
#define _ND_WORLD_H_

#include "ndVector.h"

namespace nd
{
	class World : public ndWorld
	{
		public:
		World()
			:ndWorld()
		{
		}

		void SetSubSteps(int i)
		{
			ndWorld::SetSubSteps(i);
		}
	};

}

#endif 

