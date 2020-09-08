/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndNewtonStdafx.h"
#include "ndWorld.h"

template<class scene>
class ndWorldScene: public scene
{
	public:
	ndWorldScene(ndWorld* const world)
		:scene()
		,m_world(world)
	{
	}

	ndWorld* GetWorld() const
	{
		return m_world;
	}

	void ThreadFunction()
	{
		m_world->ThreadFunction();
	}

	ndWorld* m_world;
};

class ndWorldMixedScene: public ndWorldScene<ndSceneMixed>
{
	public:
	ndWorldMixedScene(ndWorld* const world)
		:ndWorldScene<ndSceneMixed>(world)
	{
	}
};

class ndWorldSegregatedScene: public ndWorldScene<ndSceneMixed>
{
	public:
		ndWorldSegregatedScene(ndWorld* const world)
		:ndWorldScene<ndSceneMixed>(world)
	{
	}
};



