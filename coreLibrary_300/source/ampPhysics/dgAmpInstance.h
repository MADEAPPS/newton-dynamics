/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_AMP_INSTANCE_H_
#define _DG_AMP_INSTANCE_H_

#include "dgAMP.h"

using namespace concurrency;
class dgAmpInstance
{
	public:
	class dgAcceleratorDescription
	{
		public:
		char m_path[128];
		char m_description[128];
	};

	DG_CLASS_ALLOCATOR(allocator);

	dgAmpInstance(dgWorld* const world);
	~dgAmpInstance(void);

	void CleanUp();
	dgInt32 GetPlatformsCount() const;
	void SelectPlaform(dgInt32 deviceIndex);
	void GetVendorString(dgInt32 deviceIndex, char* const name, dgInt32 maxlength) const;

	void ConstraintSolver (dgInt32 islandCount, const dgIsland* const islandArray, dgFloat32 timestep);
	
	private:
	void AddArraysWithFunction() ;
	static void AddElements(index<1> idx, array_view<int, 1> sum, array_view<int, 1> a, array_view<int, 1> b) restrict(amp);


	dgWorld* m_world;
	accelerator m_accelerator;
	dgList<dgAcceleratorDescription> m_acceleratorList;
};


#endif