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

#include "stdafx.h"
#include "CNewton.h"


// Name: CNewton 
// Create an instance of the Newton world class.
//
// Remarks: this function must be called before any of the other API function.
//
// See also: ~CNewton
dNewton::dNewton()
{
//	m_world = NewtonCreate (AllocMemory, FreeMemory );
//	NewtonWorldSetUserData (m_world, this);
}


// Name: CNewton 
// Destory an instance of the Newton world class.
//
// Remarks: this function will destroyed all entity owned by Newton, the application should not make any reference to any Newton object after this call.
//
// See also: ~CNewton
dNewton::~dNewton()
{
//	NewtonDestroy (m_world);
}


// Name: BodyLeaveWorld 
// This function is called by Newton when a body leave the world
void  dNewton::BodyLeaveWorld (const NewtonWorld* newtonWorld, const NewtonBody* body)
{
//	dNewton* cworld;
//	dNewtonBody* cbody;
//	cworld = (dNewton*) NewtonWorldGetUserData (newtonWorld);
//	cbody = (dNewtonBody*) NewtonBodyGetUserData (body);
//	cworld->OnBodyLeaveWorld (cbody);
}

// Name: AllocMemory 
// Used to allocate all memory use by newton.
//
// Remarks: the default is based on malloc, applications with proprietary memory managers need to reimplement this function
void*  dNewton::AllocMemory (int size)
{
//	dNewton *world;
//	world = (dNewton *)NewtonWorldGetUserData (newtonWorld);
//	return world->Alloc (size);
}

// Name: FreeMemory 
// call by Newton to release memory
// 
// Remarks: the default is based on free, applications with proprietary memory managers need to reimplement this function
void  dNewton::FreeMemory (void *ptr, int size)
{
//	dNewton *world;
//	world = (dNewton *)NewtonWorldGetUserData (newtonWorld);
//	world->Free (ptr, size);
}


void dNewton::Update (dFloat timestep)
{
//	NewtonUpdate (m_world, timestep);
}
