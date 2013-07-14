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
#include "dNewtonBody.h"
#include "dNewtonCollision.h"
#include "dNewtonTranformLerp.h"

dNewtonBody::dNewtonBody()
	:dNewtonAlloc()
	,dNewtonTransformLerp()
	,m_body(NULL)
	,m_lock(0)
	,m_userData(NULL)
{
}

dNewtonBody::dNewtonBody (dNewton* const dWorld, dFloat mass, const dNewtonCollision* const collision, void* const userData, const dFloat* const matrix)
	:dNewtonAlloc()
	,dNewtonTransformLerp(matrix)
	,m_lock(0)
	,m_userData(userData)
{
	NewtonWorld* const world = dWorld->GetNewton ();
	NewtonBody* const body = NewtonCreateDynamicBody (world, collision->GetShape(), matrix);

	NewtonCollision* const shape = NewtonBodyGetCollision(body);
	NewtonBodySetMassProperties(body, mass, shape);

	SetBody (body);
}

dNewtonBody::~dNewtonBody()
{
	if (NewtonBodyGetDestructorCallback(m_body)) {
		NewtonBodySetDestructorCallback (m_body, NULL);
		NewtonDestroyBody(NewtonBodyGetWorld(m_body), m_body);
	}
}


NewtonBody* dNewtonBody::GetNewtonBody () const
{
	return m_body;
}

void dNewtonBody::SetUserData(void* const userData)
{
	m_userData = userData;
}

void* dNewtonBody::GetUserData() const
{
	return m_userData;
}

void dNewtonBody::SetMatrix (const dFloat* const matrix)
{
	NewtonBodySetMatrix(m_body, matrix);
}

void dNewtonBody::GetMatrix (dFloat* const matrix) const
{
	NewtonBodyGetMatrix(m_body, matrix);
}

void dNewtonBody::SetVeloc (const dFloat* const veloc)
{
	NewtonBodySetVelocity(m_body, veloc);
}

void dNewtonBody::GetVeloc (dFloat* const veloc) const
{
	NewtonBodyGetVelocity(m_body, veloc);
}

void dNewtonBody::SetOmega (const dFloat* const omega)
{
	NewtonBodySetOmega(m_body, omega);
}

void dNewtonBody::GetOmega (dFloat* const omega) const
{
	NewtonBodyGetOmega(m_body, omega);
}


void dNewtonBody::GetMassAndInertia (dFloat& mass, dFloat& Ixx, dFloat& Iyy, dFloat& Izz) const
{
	NewtonBodyGetMassMatrix(m_body, &mass, &Ixx, &Iyy, &Izz);
}

dNewton* dNewtonBody::GetNewton () const
{
	return (dNewton*) NewtonWorldGetUserData(NewtonBodyGetWorld(m_body));
}

void dNewtonBody::SetBody (NewtonBody* const body)
{
	m_body = body;
	NewtonBodySetUserData(m_body, this);
	NewtonBodySetTransformCallback (m_body, OnBodyTransform);
	NewtonBodySetDestructorCallback (m_body, OnBodyDestroy);
//	NewtonBodySetForceAndTorqueCallback(m_body, OnForceAndTorque);
}

void dNewtonBody::OnBodyDestroy (const NewtonBody* const body)
{
	dNewtonBody* const me = (dNewtonBody*) NewtonBodyGetUserData(body);
	if (me) {
		NewtonBodySetDestructorCallback (me->m_body, NULL);
		delete me;
	}
}


void dNewtonBody::GetVisualMatrix (dFloat param, dFloat* const matrix) const
{
	dNewton::ScopeLock scopelock (&m_lock);
	InterplateMatrix (param, matrix);
}


void dNewtonBody::OnBodyTransform (const dFloat* const matrix, int threadIndex)
{
	dNewton::ScopeLock scopelock (&m_lock);
	Update (matrix);
}

void dNewtonBody::OnBodyTransform (const NewtonBody* const body, const dFloat* const matrix, int threadIndex)
{
	dNewtonBody* const me = (dNewtonBody*) NewtonBodyGetUserData(body);
	dAssert (me);
	me->OnBodyTransform (matrix, threadIndex);
}


dNewtonCollision* dNewtonBody::GetCollision() const
{
	return (dNewtonCollisionScene*) NewtonCollisionGetUserData(NewtonBodyGetCollision (m_body));
}