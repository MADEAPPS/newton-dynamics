#include "stdafx.h"
#include "NewtonWrapper.h"
#include "CustomJoint.h"
#include "CustomBallAndSocket.h"
#include "CustomHinge.h"
#include "CustomDryRollingFriction.h"

extern "C"
{

	NEWTONWRAPPER_API void WRAP_NewtonDestroyCustomJoint(CustomJoint* joint)
	{
		if (joint != 0)
			delete joint;
	}

	NEWTONWRAPPER_API CustomBallAndSocket* WRAP_NewtonCreateBallAndSocket(float* matrix, NewtonBody* child, NewtonBody* parent)
	{
		return new CustomBallAndSocket(matrix, child, parent);
	}

	NEWTONWRAPPER_API CustomBallAndSocketWithFriction* WRAP_NewtonCreateBallAndSocketWithFriction(float* matrix, NewtonBody* child, NewtonBody* parent, float friction)
	{
		return new CustomBallAndSocketWithFriction(matrix, child, parent, friction);
	}

	NEWTONWRAPPER_API CustomHinge* WRAP_NewtonCreateHinge(float* matrix, NewtonBody* child, NewtonBody* parent)
	{
		return new CustomHinge(matrix, child, parent);
	}

	//NEWTONWRAPPER_API void fnNewtonHingeEnableLimits(CustomHinge* hinge, bool state)
	//{
	//	hinge->EnableLimits(state);
	//}
	//
	//NEWTONWRAPPER_API void fnNewtonHingeSetLimits(CustomHinge* hinge, float minAngle, float maxAngle)
	//{
	//	hinge->SetLimits(minAngle, maxAngle);
	//}
	//
	//NEWTONWRAPPER_API void fnNewtonHingeSetFriction(CustomHinge* hinge, float friction)
	//{
	//	hinge->SetFriction(friction);
	//}

	//CUSTOM_JOINTS_API dFloat GetJointAngle() const;
	//CUSTOM_JOINTS_API dVector GetPinAxis() const;
	//CUSTOM_JOINTS_API dFloat GetJointOmega() const;
	//CUSTOM_JOINTS_API void SetFriction(dFloat frictionTorque);

	NEWTONWRAPPER_API CustomDryRollingFriction* WRAP_NewtonCreateRollingFriction(NewtonBody* child, float radius, float coeff)
	{
		return new CustomDryRollingFriction(child, radius, coeff);
	}

}

