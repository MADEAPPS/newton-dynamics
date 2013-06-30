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


dNewtonBody::dNewtonBody()
	:m_posit0 (0.0f, 0.0f, 0.0f, 0.0f)
	,m_posit1 (0.0f, 0.0f, 0.0f, 0.0f)
	,m_rotat0()
	,m_rotat1()
	,m_body(NULL)
	,m_lock(0)
{
}

dNewtonBody::~dNewtonBody()
{
	if (NewtonBodyGetDestructorCallback(m_body)) {
		NewtonBodySetDestructorCallback (m_body, NULL);
		NewtonDestroyBody(NewtonBodyGetWorld(m_body), m_body);
	}
}

void* dNewtonBody::operator new (size_t size)
{
	return NewtonAlloc(int (size));
}

void dNewtonBody::operator delete (void* ptr)
{
	NewtonFree(ptr);
}

NewtonBody* dNewtonBody::GetNewtonBody () const
{
	return m_body;
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
}


void dNewtonBody::OnBodyDestroy (const NewtonBody* const body)
{
	dNewtonBody* const me = (dNewtonBody*) NewtonBodyGetUserData(body);
	if (me) {
		NewtonBodySetDestructorCallback (me->m_body, NULL);
		delete me;
	}
}

void dNewtonBody::OnBodyTransform (const dFloat* const matrix, int threadIndex)
{
	dMatrix transform (matrix);
	dQuaternion rot (transform);
	dNewton::ScopeLock scopelock (&m_lock);

	m_posit0 = m_posit1;
	m_rotat0 = m_rotat1;
	m_posit1 = transform.m_posit;
	m_rotat1 = rot;

	dFloat angle = m_rotat0.DotProduct(m_rotat1);
	if (angle < 0.0f) {
		m_rotat1.Scale(-1.0f);
	}
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