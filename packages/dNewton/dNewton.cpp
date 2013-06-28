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

#include "dStdAfxNewton.h"
#include "dNewton.h"


int dNewton::m_totalMemoryUsed = 0;
bool dNewton::m_memorySystemInitialized = false;


void* dNewton::DefualtAlloc (int sizeInBytes)
{
	m_totalMemoryUsed += sizeInBytes;
	return new char[sizeInBytes];
}

// memory free use by the engine
void dNewton::DefualtFree (void* ptr, int sizeInBytes)
{
	m_totalMemoryUsed -= sizeInBytes;
	delete[] (char*)ptr;
}


void dNewton::SetAllocationDrivers (CNewtonAllocMemory alloc, CNewtonFreeMemory free)
{
	if (!m_memorySystemInitialized) {
		m_memorySystemInitialized = true;
		NewtonSetMemorySystem (alloc, free);
	}
}

dNewton::dNewton()
{
	if (!m_memorySystemInitialized) {
		SetAllocationDrivers (DefualtAlloc, DefualtFree);
	}

	// create a newtop world
	m_world = NewtonCreate();

	// for two way comunication between low and high lever, link the world with this class for 
	NewtonWorldSetUserData(m_world, this);

	// set the simplified solver mode (faster but less accurate)
	NewtonSetSolverModel (m_world, 1);

	// by default runs on fout micro threads
	NewtonSetThreadsCount(m_world, 4);
}

dNewton::~dNewton()
{
	NewtonWaitForUpdateToFinish (m_world);
	NewtonDestroy (m_world);
}


void dNewton::Update (dFloat timestepInSecunds)
{
	NewtonUpdate (m_world, timestepInSecunds);
}

void dNewton::UpdateAsync (dFloat timestepInSecunds)
{
	NewtonUpdateAsync(m_world, timestepInSecunds);
}
