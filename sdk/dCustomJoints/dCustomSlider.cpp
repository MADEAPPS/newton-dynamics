/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/



// dCustomSlider.cpp: implementation of the dCustomSlider class.
//
//////////////////////////////////////////////////////////////////////
#include "dCustomJointLibraryStdAfx.h"
#include "dCustomSlider.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//dInitRtti(dCustomSlider);
IMPLEMENT_CUSTOM_JOINT(dCustomSlider);

dCustomSlider::dCustomSlider (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
	:dCustom6dof(pinAndPivotFrame, child, parent)
	,m_speed(0.0f)
	,m_posit(0.0f)
	,m_minDist(-1.0f)
	,m_maxDist(1.0f)
	,m_spring(0.0f)
	,m_damper(0.0f)
	,m_springDamperRelaxation(0.6f)
	,m_limitsOn(false)
	,m_setAsSpringDamper(false)
	,m_lastRowWasUsed(false)
{
	m_xAxis = 0;
}

dCustomSlider::dCustomSlider (const dMatrix& pinAndPivotFrameChild, const dMatrix& pinAndPivotFrameParent, NewtonBody* const child, NewtonBody* const parent)
	:dCustom6dof(pinAndPivotFrameChild, pinAndPivotFrameParent, child, parent)
	,m_speed(0.0f)
	,m_posit(0.0f)
	,m_minDist(-1.0f)
	,m_maxDist(1.0f)
	,m_spring(0.0f)
	,m_damper(0.0f)
	,m_springDamperRelaxation(0.97f)
	,m_limitsOn(false)
	,m_setAsSpringDamper(false)
	,m_lastRowWasUsed(false)
{
	m_xAxis = 0;
}

void dCustomSlider::Deserialize (NewtonDeserializeCallback callback, void* const userData)
{
	callback (userData, &m_speed, sizeof (dFloat));
	callback (userData, &m_posit, sizeof (dFloat));
	callback (userData, &m_minDist, sizeof (dFloat));
	callback (userData, &m_maxDist, sizeof (dFloat));
	callback (userData, &m_spring, sizeof (dFloat));
	callback (userData, &m_damper, sizeof (dFloat));
	callback (userData, &m_springDamperRelaxation, sizeof (dFloat));
	callback (userData, &m_flags, sizeof (int));
}

dCustomSlider::~dCustomSlider()
{
}

void dCustomSlider::Serialize (NewtonSerializeCallback callback, void* const userData) const
{
	dCustomJoint::Serialize (callback, userData);

	callback (userData, &m_speed, sizeof (dFloat));
	callback (userData, &m_posit, sizeof (dFloat));
	callback (userData, &m_minDist, sizeof (dFloat));
	callback (userData, &m_maxDist, sizeof (dFloat));
	callback (userData, &m_spring, sizeof (dFloat));
	callback (userData, &m_damper, sizeof (dFloat));
	callback (userData, &m_springDamperRelaxation, sizeof (dFloat));
	callback (userData, &m_flags, sizeof (int));
}


void dCustomSlider::EnableLimits(bool state)
{
	m_limitsOn = state;
}

void dCustomSlider::SetLimits(dFloat minDist, dFloat maxDist)
{
	m_minDist = minDist;
	m_maxDist = maxDist;
}

void dCustomSlider::SetAsSpringDamper(bool state, dFloat springDamperRelaxation, dFloat spring, dFloat damper)
{
	m_spring = spring;
	m_damper = damper;
	m_setAsSpringDamper = state;
	m_springDamperRelaxation = dClamp(springDamperRelaxation, dFloat(0.0f), dFloat(0.99f));
}

dFloat dCustomSlider::GetJointPosit () const
{
	return m_posit;
}

dFloat dCustomSlider::GetJointSpeed () const
{
	return m_speed;
}

#if 0
void dCustomSlider::SubmitConstraints (dFloat timestep, int threadIndex)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (matrix0, matrix1);

	// Restrict the movement on the pivot point along all two orthonormal axis direction perpendicular to the motion
	dVector p0(matrix0.m_posit);
	dVector p1(matrix1.m_posit + matrix1.m_front.Scale((p0 - matrix1.m_posit).DotProduct3(matrix1.m_front)));
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);

 	// three rows to restrict rotation around around the parent coordinate system
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle (matrix0.m_up, matrix1.m_up, matrix1.m_front), &matrix1.m_front[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle (matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
	NewtonUserJointAddAngularRow(m_joint, CalculateAngle (matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);

	// calculate position and speed	
	dVector veloc0(0.0f); 
	dVector veloc1(0.0f);  
	dAssert (m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_front);
	m_speed = (veloc0 - veloc1).DotProduct3(matrix1.m_front);

	m_lastRowWasUsed = false;
	SubmitConstraintsFreeDof (timestep, matrix0, matrix1);
 }

void dCustomSlider::SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
{
	// if limit are enable ...
	if (m_limitsOn && m_setAsSpringDamper) {
		m_lastRowWasUsed = true;
		if (m_posit < m_minDist) {
			const dVector& p0 = matrix0.m_posit;
			dVector p1 (p0 + matrix0.m_front.Scale (m_minDist - m_posit));
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		} else if (m_posit > m_maxDist) {
			const dVector& p0 = matrix0.m_posit;
			dVector p1 (p0 + matrix0.m_front.Scale (m_maxDist - m_posit));
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		} else {
			const dVector& p0 = matrix0.m_posit;
			const dVector& p1 = matrix1.m_posit;
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		}

		const dFloat velCut = 0.5f;
		// I should just calculate acceleration to cancel the relative velocity and neglect the position, 
		// but for now this si good enough
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
		const dFloat speedStep = dAbs (stopAccel * timestep);
		if ((m_posit < m_minDist) && (speedStep > velCut))  {
			//NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, dFloat (0.0f));
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, dFloat (0.0f));
		} else if ((m_posit > m_maxDist) && (speedStep > velCut))  {
			//NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, dFloat (0.0f));
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowMaximumFriction(m_joint, dFloat(0.0f));
		} else {
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
		}

	} else if (m_limitsOn) {
		if (m_posit < m_minDist) {
			// get a point along the up vector and set a constraint  
			const dVector& p0 = matrix0.m_posit;
			dVector p1 (p0 + matrix0.m_front.Scale (m_minDist - m_posit));
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
			m_lastRowWasUsed = true;
		} else if (m_posit > m_maxDist) {
			// get a point along the up vector and set a constraint  

			const dVector& p0 = matrix0.m_posit;
			dVector p1 (p0 + matrix0.m_front.Scale (m_maxDist - m_posit));
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);
			m_lastRowWasUsed = true;
		} else {
/*
			// uncomment this for a slider with friction

			// take any point on body0 (origin)
			const dVector& p0 = matrix0.m_posit;

			dVector veloc0(0.0f); 
			dVector veloc1(0.0f); 
			dVector omega1(0.0f); 

			NewtonBodyGetVelocity(m_body0, &veloc0[0]);
			NewtonBodyGetVelocity(m_body1, &veloc1[0]);
			NewtonBodyGetOmega(m_body1, &omega1[0]);

			// this assumes the origin of the bodies the matrix pivot are the same
			veloc1 += omega1 * (matrix1.m_posit - p0);

			dFloat relAccel; 
			relAccel = ((veloc1 - veloc0) % matrix0.m_front) / timestep;

			#define MaxFriction 10.0f
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p0[0], &matrix0.m_front[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAccel);
			NewtonUserJointSetRowMinimumFriction (m_joint, -MaxFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, MaxFriction);
			m_lastRowWasUsed = false;
*/
		}
	} else if (m_setAsSpringDamper) {
		m_lastRowWasUsed = true;
		const dVector& p0 = matrix0.m_posit;
		const dVector& p1 = matrix1.m_posit;
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	} 
}
#endif

void dCustomSlider::SubmitConstraintsFreeDof(int freeDof, const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep, int threadIndex)
{
	dAssert(freeDof == 1);

	dVector veloc0;
	dVector veloc1;
	dAssert(m_body0);
	NewtonBodyGetPointVelocity(m_body0, &matrix0.m_posit[0], &veloc0[0]);
	if (m_body1) {
		NewtonBodyGetPointVelocity(m_body1, &matrix1.m_posit[0], &veloc1[0]);
	}
	m_posit = (matrix0.m_posit - matrix1.m_posit).DotProduct3(matrix1.m_front);
	m_speed = (veloc0 - veloc1).DotProduct3(matrix1.m_front);

	// if limit are enable ...
	if (m_limitsOn && m_setAsSpringDamper) {
		m_lastRowWasUsed = true;
		if (m_posit < m_minDist) {
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_minDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		} else if (m_posit > m_maxDist) {
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_maxDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		} else {
			const dVector& p0 = matrix0.m_posit;
			const dVector& p1 = matrix1.m_posit;
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		}

		const dFloat velCut = 0.5f;
		// I should just calculate acceleration to cancel the relative velocity and neglect the position, 
		// but for now this si good enough
		const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
		const dFloat speedStep = dAbs(stopAccel * timestep);
		if ((m_posit < m_minDist) && (speedStep > velCut)) {
			//NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, dFloat (0.0f));
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, dFloat(0.0f));
		}
		else if ((m_posit > m_maxDist) && (speedStep > velCut)) {
			//NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, dFloat (0.0f));
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowMaximumFriction(m_joint, dFloat(0.0f));
		}
		else {
			NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
		}
	} else if (m_limitsOn) {
		if (m_posit < m_minDist) {
			// get a point along the up vector and set a constraint  
			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_minDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			m_lastRowWasUsed = true;
		} else if (m_posit > m_maxDist) {
			// get a point along the up vector and set a constraint  

			const dVector& p0 = matrix0.m_posit;
			dVector p1(p0 + matrix0.m_front.Scale(m_maxDist - m_posit));
			NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			m_lastRowWasUsed = true;
		} else {
			/*
			// uncomment this for a slider with friction

			// take any point on body0 (origin)
			const dVector& p0 = matrix0.m_posit;

			dVector veloc0(0.0f);
			dVector veloc1(0.0f);
			dVector omega1(0.0f);

			NewtonBodyGetVelocity(m_body0, &veloc0[0]);
			NewtonBodyGetVelocity(m_body1, &veloc1[0]);
			NewtonBodyGetOmega(m_body1, &omega1[0]);

			// this assumes the origin of the bodies the matrix pivot are the same
			veloc1 += omega1 * (matrix1.m_posit - p0);

			dFloat relAccel;
			relAccel = ((veloc1 - veloc0) % matrix0.m_front) / timestep;

			#define MaxFriction 10.0f
			NewtonUserJointAddLinearRow (m_joint, &p0[0], &p0[0], &matrix0.m_front[0]);
			NewtonUserJointSetRowAcceleration (m_joint, relAccel);
			NewtonUserJointSetRowMinimumFriction (m_joint, -MaxFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, MaxFriction);
			m_lastRowWasUsed = false;
			*/
		}
	} else if (m_setAsSpringDamper) {
		m_lastRowWasUsed = true;
		const dVector& p0 = matrix0.m_posit;
		const dVector& p1 = matrix1.m_posit;
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		NewtonUserJointSetRowSpringDamperAcceleration(m_joint, m_springDamperRelaxation, m_spring, m_damper);
	}
}