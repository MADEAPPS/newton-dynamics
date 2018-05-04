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
#include "dNewtonJointRelational.h"

dNewtonJointGear::dNewtonJointGear(dFloat ratio, const dVector pin0, const dVector pin1, void* const body0, void* const body1)
	:dNewtonJoint()
{
	dMatrix bodyMatrix0;
	dMatrix bodyMatrix1;
	NewtonBody* const netwonBody0 = (NewtonBody*)body0;
	NewtonBody* const netwonBody1 = (NewtonBody*)body1;
	NewtonBodyGetMatrix(netwonBody0, &bodyMatrix0[0][0]);
	NewtonBodyGetMatrix(netwonBody1, &bodyMatrix1[0][0]);

	dVector childPin(bodyMatrix0.RotateVector(dVector(pin0[0], pin0[1], pin0[2], 0.0f)));
	dVector parentPin(bodyMatrix1.RotateVector(dVector(pin1[0], pin1[1], pin1[2], 0.0f)));

	dCustomGear* const gear = new dCustomGear(dAbs(ratio), childPin, parentPin, netwonBody0, netwonBody1);
	SetJoint(gear);
}

dNewtonJointDifferentialGear::dNewtonJointDifferentialGear(dFloat ratio, const dVector childPin, const dVector parentPin, const dVector referencePin, void* const child, void* const parent, void* const parentReference)
	:dNewtonJoint()
{
	dMatrix bodyMatrix0;
	dMatrix bodyMatrix1;
	dMatrix bodyMatrix2;
	NewtonBody* const childBody = (NewtonBody*)child;
	NewtonBody* const parentBody = (NewtonBody*)parent;
	NewtonBody* const parentReferenceBody = (NewtonBody*)parentReference;

	NewtonBodyGetMatrix (childBody, &bodyMatrix0[0][0]);
	NewtonBodyGetMatrix (parentBody, &bodyMatrix1[0][0]);
	NewtonBodyGetMatrix (parentReferenceBody, &bodyMatrix2[0][0]);

	dVector pin0 (bodyMatrix0.RotateVector(childPin));
	dVector pin1 (bodyMatrix1.RotateVector(parentPin));
	dVector pin2 (bodyMatrix2.RotateVector(referencePin));

	dCustomDifferentialGear* const gear = new dCustomDifferentialGear(ratio, pin0, pin1, pin2, childBody, parentBody, parentReferenceBody);
	SetJoint(gear);
}