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
#include "dNewtonDynamicBody.h"


dNewtonJoint::dNewtonJoint(dJointType type)
	:m_type(type) 
	,m_joint(NULL)
{
}

void dNewtonJoint::OnSubmitConstraint (dFloat timestep, int threadIndex)
{
}


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

dNewtonDynamicBody* dNewtonJoint::GetBody0 () const
{
	return (dNewtonDynamicBody*)NewtonBodyGetUserData (m_joint->GetBody0()); 
}

dNewtonDynamicBody* dNewtonJoint::GetBody1 () const
{
	return  m_joint->GetBody1() ? (dNewtonDynamicBody*)NewtonBodyGetUserData (m_joint->GetBody1()) : NULL; 
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


dNewtonBallAndSocketJoint::dNewtonBallAndSocketJoint(const dFloat* const pinAndPivotFrame, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
    :dNewtonJoint(m_ballAndSocket)
{
    SetJoint (new CustomBallAndSocket (dMatrix(pinAndPivotFrame), body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
}


dNewtonHingeJoint::dNewtonHingeJoint(const dFloat* const pinAndPivotFrame, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
	:dNewtonJoint(m_hinge)
{
	SetJoint (new CustomHinge (dMatrix(pinAndPivotFrame), body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
}

dFloat dNewtonHingeJoint::GetFriction () const
{
	return ((CustomHinge*)m_joint)->GetFriction();
}

void dNewtonHingeJoint::SetFriction (dFloat friction)
{
	((CustomHinge*)m_joint)->SetFriction(friction);
}

void dNewtonHingeJoint::EnableLimits(bool state)
{
    ((CustomHinge*)m_joint)->EnableLimits(state);
}

void dNewtonHingeJoint::SetLimis(dFloat minAngle, dFloat maxAngle)
{
    ((CustomHinge*)m_joint)->SetLimis(minAngle, maxAngle);
}

dNewtonSliderJoint::dNewtonSliderJoint(const dFloat* const pinAndPivotFrame, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
	:dNewtonJoint(m_slider)
{
	SetJoint (new CustomSlider (dMatrix(pinAndPivotFrame), body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
}

void dNewtonSliderJoint::EnableLimits(bool state)
{
    ((CustomSlider*) m_joint)->EnableLimits(state);
}

void dNewtonSliderJoint::SetLimis(dFloat minDist, dFloat maxDist)
{
    ((CustomSlider*) m_joint)->SetLimis(minDist, maxDist);
}

dNewtonUniversalJoint::dNewtonUniversalJoint(const dFloat* const pinAndPivotFrame, dNewtonDynamicBody* const body0, dNewtonDynamicBody* const body1)
	:dNewtonJoint(m_universal)
{
	SetJoint (new CustomUniversal (dMatrix(pinAndPivotFrame), body0->GetNewtonBody(), body1 ? body1->GetNewtonBody() : NULL));
}

void dNewtonUniversalJoint::EnableLimit_0(bool state)
{
	((CustomUniversal*) m_joint)->EnableLimit_0(state);
}

void dNewtonUniversalJoint::EnableLimit_1(bool state)
{
	((CustomUniversal*) m_joint)->EnableLimit_1(state);
}

void dNewtonUniversalJoint::SetLimis_0(dFloat minAngle, dFloat maxAngle)
{
	((CustomUniversal*) m_joint)->SetLimis_0 (minAngle, maxAngle);
}

void dNewtonUniversalJoint::SetLimis_1(dFloat minAngle, dFloat maxAngle)
{
	((CustomUniversal*) m_joint)->SetLimis_1 (minAngle, maxAngle);
}
