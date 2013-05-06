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


#include "dgOpencl.h"
#include "dgWorld.h"
#include "dgOpenclInstance.h"

dgOpenclInstance::dgOpenclInstance(dgWorld* const world)
	:m_world (world)
	,m_opencl (dgOpencl::GetOpenCL(world->GetAllocator()))
{
	_ASSERTE (0);
}

dgOpenclInstance::~dgOpenclInstance(void)
{
	_ASSERTE (0);
}


void dgOpenclInstance::CleanUp()
{
	_ASSERTE (0);
}

void dgOpenclInstance::SelectPlaform(dgInt32 deviceIndex)
{
	_ASSERTE (0);
}

dgInt32 dgOpenclInstance::GetPlatformsCount() const
{
	_ASSERTE (0);
	return 0;
}

void dgOpenclInstance::GetVendorString(dgInt32 deviceIndex, char* const name, dgInt32 maxlength) const
{
	_ASSERTE (0);
}

