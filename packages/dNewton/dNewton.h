/* Copyright (c) <2003-2013> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_NEWTON_H_
#define _D_NEWTON_H_

#include "dStdAfxNewton.h"
#include "dNewtonAlloc.h"

class dNewtonBody;

class dNewton: public dNewtonAlloc
{
	public:
	class ScopeLock
	{
		public:
		CNEWTON_API ScopeLock (unsigned* const lock);
		CNEWTON_API ~ScopeLock();

		unsigned* m_atomicLock;
	};

	CNEWTON_API dNewton();
	CNEWTON_API virtual ~dNewton();

	CNEWTON_API virtual void Update (dFloat timestepInSecunds);
	CNEWTON_API virtual void UpdateAsync (dFloat timestepInSecunds);
	CNEWTON_API void WaitForUpdateToFinish ();

	CNEWTON_API dFloat GetInteplationParam(dFloat timestepInSecunds) const;

	CNEWTON_API void SetMaxUpdatesPerIterations (int update);
	CNEWTON_API NewtonWorld* GetNewton () const;
	
	CNEWTON_API dNewtonBody* GetFirstBody() const;
	CNEWTON_API dNewtonBody* GetNextBody(const dNewtonBody* const body) const;

	CNEWTON_API int GetBodyCount() const;
	CNEWTON_API void DestroyAllBodies();

	protected:
	CNEWTON_API void ResetTimer();
	CNEWTON_API dLong GetTimeInMicrosenconds() const; 

	private:
	CNEWTON_API static void OnCollisionDestructorCallback (const NewtonWorld* const newtonWorld, const NewtonCollision* const collision);
	CNEWTON_API static void OnCollisionCopyConstruct (const NewtonWorld* const newtonWorld, NewtonCollision* const collision, const NewtonCollision* const sourceCollision);
	

	NewtonWorld* m_world;
	dLong m_frequency;
	dLong m_baseCount;
	dLong m_microseconds;
	int m_maxUpdatePerIterations;
};

#endif
