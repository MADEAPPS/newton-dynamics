/* 
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
#include "dNewtonJoint.h"

dNewtonJoint::dNewtonJoint()
	:dAlloc()
	,m_joint(NULL)
{
}

dNewtonJoint::~dNewtonJoint()
{
}

void dNewtonJoint::SetJoint(dCustomJoint* const joint)
{
	m_joint = joint;
	m_joint->SetUserData(this);
	m_joint->SetUserDestructorCallback(DestructorCallback);
}

void dNewtonJoint::Destroy()
{
	if (m_joint) {
		delete m_joint;
		m_joint = NULL;
	}
}

void dNewtonJoint::SetStiffness(dFloat stiffness)
{
	m_joint->SetStiffness(stiffness);
}

// This callback is triggered on join destruction and can be triggered when a body connected to this joint is destroyed.
// This will set the reference joint pointer to NULL so that we won't try to delete it again if the method Destroy is called
void dNewtonJoint::DestructorCallback(const dCustomJoint* const me) 
{
	dNewtonJoint* joint = static_cast<dNewtonJoint*>(me->GetUserData());
	if (joint) {
		joint->m_joint = NULL;
	}
}
