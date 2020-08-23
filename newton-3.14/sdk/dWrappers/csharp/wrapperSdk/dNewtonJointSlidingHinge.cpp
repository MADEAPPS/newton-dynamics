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
#include "dNewtonJointSlidingHinge.h"

dNewtonJointSlidingHinge::dNewtonJointSlidingHinge(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1)
	:dNewtonJoint()
{
	dMatrix bodyMatrix;
	NewtonBody* const netwonBody0 = (NewtonBody*)body0;
	NewtonBody* const netwonBody1 = (NewtonBody*)body1;
	NewtonBodyGetMatrix(netwonBody0, &bodyMatrix[0][0]);

	dMatrix matrix(pintAndPivotMatrix * bodyMatrix);
	dCustomSlidingContact* const joint = new dCustomSlidingContact(matrix, netwonBody0, netwonBody1);
	SetJoint(joint);
}


void dNewtonJointSlidingHinge::SetLimits(bool enable, dFloat minDistance, dFloat maxDistance)
{
	dCustomSlidingContact* const joint = (dCustomSlidingContact*)m_joint;
	joint->EnableLimits(enable);
	if (enable) {
		joint->SetLimits(dMin(minDistance, 0.0f), dMax(maxDistance, 0.0f));
	}
}

void dNewtonJointSlidingHinge::SetAsSpringDamper(bool enable, dFloat forceMixing, dFloat springConst, dFloat damperConst)
{
	dCustomSlidingContact* const joint = (dCustomSlidingContact*)m_joint;
	joint->SetAsSpringDamper(enable, dClamp(forceMixing, 0.7f, 0.99f), dAbs(springConst), dAbs(damperConst));
}


