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
#include "dNewtonJoint.h"


void dNewtonJoint::SetJoint(CustomJoint* const joint)
{
	m_joint = joint;
	m_joint->SetUserData (this);
	m_joint->SetUserDestructorCallback (OnJointDestroyCallback);
	m_joint->SetUserSubmintConstraintCallback (OnSubmitConstraintCallback);
}

dNewtonJoint::~dNewtonJoint()
{
	if (m_joint && m_joint->GetUserData()) {
		m_joint->SetUserData (NULL);
		m_joint->SetUserDestructorCallback (NULL);
		m_joint->SetUserSubmintConstraintCallback (NULL);
		delete m_joint;
	}
}

void dNewtonJoint::OnJointDestroyCallback (const NewtonUserJoint* const me)
{
	CustomJoint* const customJoint = (CustomJoint*)me;

	dNewtonJoint* const joint = (dNewtonJoint*) customJoint->GetUserData();
	joint->m_joint = NULL;
	customJoint->SetUserData(NULL);
	delete joint;
}

void dNewtonJoint::OnSubmitConstraintCallback (const NewtonUserJoint* const me, dFloat timestep, int threadIndex)
{
	CustomJoint* const customJoint = (CustomJoint*)me;
	dNewtonJoint* const joint = (dNewtonJoint*) customJoint->GetUserData();
	joint->OnSubmitConstraint (timestep, threadIndex);
}