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

#ifndef _D_NEWTON_JOINT_UNIVERSAL_H_
#define _D_NEWTON_JOINT_UNIVERSAL_H_

#include "stdafx.h"
#include "dNewtonJoint.h"

class dNewtonJointDoubleHinge: public dNewtonJoint
{
	public:
		dNewtonJointDoubleHinge(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1);

	void SetLimits_0(bool enable, dFloat minVal, dFloat maxAngle);
	void SetLimits_1(bool enable, dFloat minVal, dFloat maxAngle);
};


class dNewtonJointDoubleHingeActuator : public dNewtonJoint
{
	public:
		dNewtonJointDoubleHingeActuator(const dMatrix pintAndPivotMatrix, void* const body0, void* const body1);

	dFloat GetAngle0() const;
	void SetMaxToque0(dFloat torque);
	void SetAngularRate0(dFloat rate);
	void SetTargetAngle0(dFloat angle, dFloat minLimit, dFloat maxLimit);

	dFloat GetAngle1() const;
	void SetMaxToque1(dFloat torque);
	void SetAngularRate1(dFloat rate);
	void SetTargetAngle1(dFloat angle, dFloat minLimit, dFloat maxLimit);
};


#endif
