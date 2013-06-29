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
#include "dNewtonCollision.h"



dNewtonCollision::dNewtonCollision(NewtonCollision* const shape)
	:m_shape (shape)
{
	NewtonCollisionSetUserData (m_shape, this);
}


dNewtonCollision* dNewtonCollision::Clone(NewtonCollision* const shape) const
{
	return new dNewtonCollision (shape);
}

dNewtonCollision::~dNewtonCollision()
{
	if (m_shape) {
		NewtonCollisionSetUserData (m_shape, NULL);
		NewtonDestroyCollision(m_shape);
	}
}

NewtonCollision* dNewtonCollision::GetShape() const
{
	return m_shape;
}


void* dNewtonCollision::operator new (size_t size)
{
	return NewtonAlloc(int (size));
}

void dNewtonCollision::operator delete (void* ptr)
{
	NewtonFree(ptr);
}


