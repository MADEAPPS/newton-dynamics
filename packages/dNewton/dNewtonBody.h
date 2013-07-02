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

#ifndef _D_NEWTON_BODY_H_
#define _D_NEWTON_BODY_H_

#include "dStdAfxNewton.h"
#include "dNewtonAlloc.h"

class dNewton;
class dNewtonCollision;

class dNewtonBody: public dNewtonAlloc
{
	public:

	CNEWTON_API dNewtonBody(dNewton* const world, dFloat mass, const dNewtonCollision* const collision, void* const userData, const dMatrix& location);
	CNEWTON_API virtual ~dNewtonBody();

	CNEWTON_API void SetMatrix (const dFloat* const matrix);
	CNEWTON_API void GetMatrix (dFloat* const matrix) const;

	CNEWTON_API void SetForce (const dFloat* const force) const;
	CNEWTON_API void SetTorque (const dFloat* const torque) const;

	CNEWTON_API void GetMassAndInertia (dFloat& mass, dFloat& Ixx, dFloat& Iyy, dFloat& Izz) const;

	CNEWTON_API virtual void OnForceAndTorque (dFloat timestep, int threadIndex) = 0;
	CNEWTON_API virtual void OnBodyTransform (const dFloat* const matrix, int threadIndex);

	CNEWTON_API void* GetUserData() const;
	CNEWTON_API void SetUserData(void* const userData);

	
	CNEWTON_API dNewton* GetNewton () const;
	CNEWTON_API NewtonBody* GetNewtonBody () const;
	CNEWTON_API dNewtonCollision* GetCollision() const;

	CNEWTON_API bool GetSleepState() const;
	CNEWTON_API dMatrix GetVisualMatrix (dFloat param) const;

	protected:
	CNEWTON_API dNewtonBody();
	void SetBody (NewtonBody* const body);

	private: 
	CNEWTON_API static void OnBodyDestroy (const NewtonBody* const body);
	CNEWTON_API static void OnForceAndTorque (const NewtonBody* body, dFloat timestep, int threadIndex);
	CNEWTON_API static void OnBodyTransform (const NewtonBody* const body, const dFloat* const matrix, int threadIndex);

	dVector m_posit0;
	dVector m_posit1;
	dQuaternion m_rotat0;
	dQuaternion m_rotat1;
	NewtonBody* m_body;
	void* m_userData;

	mutable unsigned m_lock;
	
};

#endif
