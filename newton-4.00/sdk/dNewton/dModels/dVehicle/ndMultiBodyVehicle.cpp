/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyDynamic.h"
#include "ndBodyKinematic.h"
#include "ndMultiBodyVehicle.h"
#include "dJoints/ndJointHinge.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTireJoint.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

#define D_MAX_CONTACT_SPEED_TRESHOLD	ndFloat32 (0.1f)
#define D_MAX_CONTACT_PENETRATION		ndFloat32 (1.0e-2f)
#define D_MIN_CONTACT_CLOSE_DISTANCE2	ndFloat32 (5.0e-2f * 5.0e-2f)

#define D_MAX_SIDESLIP_ANGLE			ndFloat32(20.0f)
#define D_MAX_STEERING_RATE				ndFloat32(0.03f)
#define D_MAX_SIZE_SLIP_RATE			ndFloat32(2.0f)

ndMultiBodyVehicle::ndDownForce::ndDownForce()
	:m_gravity(ndFloat32(-10.0f))
	,m_suspensionStiffnessModifier(ndFloat32(1.0f))
{
	m_downForceTable[0].m_speed = ndFloat32(0.0f) * ndFloat32(0.27f);
	m_downForceTable[0].m_forceFactor = 0.0f;
	m_downForceTable[0].m_aerodynamicDownforceConstant = ndFloat32(0.0f);

	m_downForceTable[1].m_speed = ndFloat32(30.0f) * ndFloat32(0.27f);
	m_downForceTable[1].m_forceFactor = 0.5f;
	m_downForceTable[1].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[0]);

	m_downForceTable[2].m_speed = ndFloat32(60.0f) * ndFloat32(0.27f);
	m_downForceTable[2].m_forceFactor = 1.0f;
	m_downForceTable[2].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[1]);

	m_downForceTable[3].m_speed = ndFloat32(140.0f) * ndFloat32(0.27f);
	m_downForceTable[3].m_forceFactor = 2.0f;
	m_downForceTable[3].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[2]);

	m_downForceTable[4].m_speed = ndFloat32(1000.0f) * ndFloat32(0.27f);
	m_downForceTable[4].m_forceFactor = 2.0f;
	m_downForceTable[4].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[3]);
}

ndFloat32 ndMultiBodyVehicle::ndDownForce::CalculateFactor(const ndSpeedForcePair* const entry0) const
{
	const ndSpeedForcePair* const entry1 = entry0 + 1;
	ndFloat32 num = ndMax(entry1->m_forceFactor - entry0->m_forceFactor, ndFloat32(0.0f));
	ndFloat32 den = ndMax(ndAbs(entry1->m_speed - entry0->m_speed), ndFloat32(1.0f));
	return num / (den * den);
}

ndFloat32 ndMultiBodyVehicle::ndDownForce::GetDownforceFactor(ndFloat32 speed) const
{
	ndAssert(speed >= ndFloat32(0.0f));
	ndInt32 index = 0;
	for (ndInt32 i = sizeof(m_downForceTable) / sizeof(m_downForceTable[0]) - 1; i; i--)
	{
		if (m_downForceTable[i].m_speed <= speed)
		{
			index = i;
			break;
		}
	}

	index = ndMin(index, ndInt32 (sizeof(m_downForceTable) / sizeof(m_downForceTable[0])) - 2);
	ndFloat32 deltaSpeed = speed - m_downForceTable[index].m_speed;
	ndFloat32 downForceFactor = m_downForceTable[index].m_forceFactor + m_downForceTable[index + 1].m_aerodynamicDownforceConstant * deltaSpeed * deltaSpeed;
	return downForceFactor * m_gravity;
}


#if 0
ndMultiBodyVehicleTorsionBar* ndMultiBodyVehicle::AddTorsionBar(ndBodyKinematic* const)
{
	ndAssert(0);
	return nullptr;

	//m_torsionBar = ndSharedPtr<ndMultiBodyVehicleTorsionBar>(new ndMultiBodyVehicleTorsionBar(this, sentinel));
	//return *m_torsionBar;
}
#endif

ndMultiBodyVehicle::ndMultiBodyVehicle()
	:ndModelArticulation()
	,m_localFrame(ndGetIdentityMatrix())
	,m_tireShape(new ndShapeChamferCylinder(ndFloat32(0.75f), ndFloat32(0.5f)))
	,m_downForce()
{
	m_iniliazed = false;
	m_motor = nullptr;
	m_gearBox = nullptr;
	m_chassis = nullptr;
	m_torsionBar = nullptr;

	m_steeringRate = D_MAX_STEERING_RATE;
	m_maxSideslipRate = D_MAX_SIZE_SLIP_RATE;
	m_maxSideslipAngle = D_MAX_SIDESLIP_ANGLE;

	m_tireShape->AddRef();
}

ndMultiBodyVehicle::~ndMultiBodyVehicle()
{
	m_tireShape->Release();
}

void ndMultiBodyVehicle::FinalizeBuild()
{
	struct ndIndexPair
	{
		ndInt32 m_m0;
		ndInt32 m_m1;
	};
	ndFixSizeArray<ndNode*, 64> stack;
	ndFixSizeArray<ndInt32, 64> bodyId;
	ndFixSizeArray<ndFloat32, 64> invMass;
	ndFixSizeArray<ndFloat32, 64> rhsAccel;
	ndFixSizeArray<ndMatrix, 64> invInertia;
	ndFixSizeArray<ndIndexPair, 64> jointArray;
	ndFixSizeArray<ndJacobianPair, 64> jacobianArray;
	ndFixSizeArray<const ndBodyKinematic*, 64> bodyArray;

	const ndVector upDir(m_localFrame.m_up);
	const ndMatrix savedMatrix(GetRoot()->m_body->GetMatrix());
	SetTransform(ndGetIdentityMatrix());

	ndInt32 bodyIndex = 0;

	// get all jacobians
	stack.PushBack(GetRoot());
	while (stack.GetCount())
	{
		ndNode* const node = stack.Pop();

		const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		bodyArray.PushBack(body);
		bodyId.PushBack(bodyIndex);

		invMass.PushBack(body->GetInvMass());
		invInertia.PushBack(body->CalculateInvInertiaMatrix());
		
		if (node->m_joint)
		{
			ndJointBilateralConstraint* const joint = *node->m_joint;

			auto FindParentId = [joint, &bodyArray, &bodyId]()
			{
				const ndBodyKinematic* const parentBody = joint->GetBody1()->GetAsBodyKinematic();
				for (ndInt32 i = bodyArray.GetCount() - 1; i >= 0; --i)
				{
					if (bodyArray[i] == parentBody)
					{
						return bodyId[i];
					}
				}
				ndAssert(0);
				return -1;
			};

			ndInt32 m0 = bodyIndex;
			ndInt32 m1 = FindParentId();

			const ndVector com0(bodyArray[m0]->GetMatrix().TransformVector(bodyArray[m0]->GetCentreOfMass()));
			const ndVector com1(bodyArray[m1]->GetMatrix().TransformVector(bodyArray[m1]->GetCentreOfMass()));
			const ndVector r(com1 - com0);

			ndInt32 rowIndex = jointArray.GetCount();
			jacobianArray[rowIndex].m_jacobianM0.m_linear = upDir;
			jacobianArray[rowIndex].m_jacobianM1.m_linear = upDir.Scale(ndFloat32(-1.0f));
			jacobianArray[rowIndex].m_jacobianM0.m_angular = jacobianArray[rowIndex].m_jacobianM0.m_linear.CrossProduct(r);
			jacobianArray[rowIndex].m_jacobianM1.m_angular = jacobianArray[rowIndex].m_jacobianM1.m_linear.CrossProduct(r);

			ndIndexPair pair;
			pair.m_m0 = m0;
			pair.m_m1 = m1;
			jointArray.PushBack(pair);
			rhsAccel.PushBack(ndFloat32 (0.0f));

			if (!strcmp(joint->ClassName(), "ndMultiBodyVehicleTireJoint"))
			{
				//ndAssert(0);
			}
		}

		bodyIndex++;
		for (ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		{
			stack.PushBack(child);
		}
	}

	// build Mass Matrix
	ndFixSizeArray<ndFixSizeArray<ndFloat32, 64>, 64> massMatrix(64);
	massMatrix.SetCount(jointArray.GetCount());
	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		massMatrix[i].SetCount(jointArray.GetCount());
	}

	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		ndInt32 m0 = jointArray[i].m_m0;
		ndInt32 m1 = jointArray[i].m_m1;

		ndJacobian JinvMass0(jacobianArray[i].m_jacobianM0);
		ndJacobian JinvMass1(jacobianArray[i].m_jacobianM1);

		JinvMass0.m_linear = JinvMass0.m_linear.Scale(invMass[m0]);
		JinvMass1.m_linear = JinvMass1.m_linear.Scale(invMass[m1]);
		JinvMass0.m_angular = invInertia[m0].RotateVector(JinvMass0.m_angular);
		JinvMass1.m_angular = invInertia[m1].RotateVector(JinvMass1.m_angular);

		ndVector diagDot(
			JinvMass0.m_linear * jacobianArray[i].m_jacobianM0.m_linear +
			JinvMass0.m_angular * jacobianArray[i].m_jacobianM0.m_angular +
			JinvMass1.m_linear * jacobianArray[i].m_jacobianM1.m_linear +
			JinvMass1.m_angular * jacobianArray[i].m_jacobianM1.m_angular);
		ndFloat32 diagonal = diagDot.AddHorizontal().GetScalar() * 1.001f;
		massMatrix[i][i] = diagonal;

		for (ndInt32 j = i + 1; j < jointArray.GetCount(); ++j)
		{
			ndFloat32 offDiag = ndFloat32(0.0f);
			if (m0 == jointArray[j].m_m0)
			{
				ndAssert(0);
				const ndJacobian& Jt(jacobianArray[j].m_jacobianM0);
				ndVector dot(JinvMass0.m_linear * Jt.m_linear + JinvMass0.m_angular * Jt.m_angular);
				offDiag += dot.AddHorizontal().GetScalar();
			}
			if (m0 == jointArray[j].m_m1)
			{
				ndAssert(0);
				const ndJacobian& Jt(jacobianArray[j].m_jacobianM1);
				ndVector dot(JinvMass0.m_linear * Jt.m_linear + JinvMass0.m_angular * Jt.m_angular);
				offDiag += dot.AddHorizontal().GetScalar();
			}
			if (m1 == jointArray[j].m_m0)
			{
				ndAssert(0);
				const ndJacobian& Jt(jacobianArray[j].m_jacobianM0);
				ndVector dot(JinvMass1.m_linear* Jt.m_linear + JinvMass1.m_angular * Jt.m_angular);
				offDiag += dot.AddHorizontal().GetScalar();
			}
			if (m1 == jointArray[j].m_m1)
			{
				const ndJacobian& Jt(jacobianArray[j].m_jacobianM1);
				ndVector dot(JinvMass1.m_linear * Jt.m_linear + JinvMass1.m_angular * Jt.m_angular);
				offDiag += dot.AddHorizontal().GetScalar();
			}
			massMatrix[i][j] = offDiag;
			massMatrix[j][i] = offDiag;
		}
	}
	ndAssert(ndTestPSDmatrix(jointArray.GetCount(), jointArray.GetCapacity(), &massMatrix[0][0]));


	SetTransform(savedMatrix);
	m_iniliazed = true;
}

const ndMatrix& ndMultiBodyVehicle::GetLocalFrame() const
{
	return m_localFrame;
}

void ndMultiBodyVehicle::SetLocalFrame(const ndMatrix& localframe)
{
	m_localFrame.m_front = (localframe.m_front & ndVector::m_triplexMask).Normalize();
	m_localFrame.m_up = localframe.m_up & ndVector::m_triplexMask;
	m_localFrame.m_right = m_localFrame.m_front.CrossProduct(m_localFrame.m_up).Normalize();
	m_localFrame.m_up = m_localFrame.m_right.CrossProduct(m_localFrame.m_front).Normalize();
}

ndBodyDynamic* ndMultiBodyVehicle::GetChassis() const
{
	return m_chassis;
}

ndMultiBodyVehicleMotor* ndMultiBodyVehicle::GetMotor() const
{
	return m_motor;
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::GetGearBox() const
{
	return m_gearBox;
}

const ndList<ndMultiBodyVehicleTireJoint*>& ndMultiBodyVehicle::GetTireList() const
{
	return m_tireList;
}

ndMultiBodyVehicle* ndMultiBodyVehicle::GetAsMultiBodyVehicle()
{
	return this;
}

ndFloat32 ndMultiBodyVehicle::GetSpeed() const
{
	const ndVector dir(m_chassis->GetMatrix().RotateVector(m_localFrame.m_front));
	const ndFloat32 speed = ndAbs(m_chassis->GetVelocity().DotProduct(dir).GetScalar());
	return speed;
}

void ndMultiBodyVehicle::AddChassis(const ndSharedPtr<ndBody>& chassis)
{
	m_iniliazed = false;
	m_chassis = chassis->GetAsBodyDynamic();
	ndAssert(m_chassis);

	ndAssert(!GetRoot() || (GetRoot()->m_body == chassis));
	if (!FindByBody(*chassis))
	{
		AddRootBody(chassis);
	}
}

void ndMultiBodyVehicle::AddTire(const ndSharedPtr<ndBody>& tireBody, const ndSharedPtr<ndJointBilateralConstraint>& tireJoint)
{
	m_iniliazed = false;
	ndAssert(!strcmp(tireJoint->ClassName(), "ndMultiBodyVehicleTireJoint"));
	m_tireList.Append((ndMultiBodyVehicleTireJoint*) *tireJoint);

	ndNode* const node = FindByBody(*tireBody);
	ndAssert(!node || ((node->m_body->GetAsBody() == *tireBody) && ((*node->m_joint == *tireJoint))));
	if (!node)
	{
		ndAssert(tireJoint->GetBody1() == GetRoot()->m_body->GetAsBody());
		AddLimb(GetRoot(), tireBody, tireJoint);
	}
	tireBody->GetAsBodyDynamic()->SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
}

void ndMultiBodyVehicle::AddMotor(const ndSharedPtr<ndBody>& motorBody, const ndSharedPtr<ndJointBilateralConstraint>& motorJoint)
{
	m_iniliazed = false;
	ndAssert(!strcmp(motorJoint->ClassName(), "ndMultiBodyVehicleMotor"));
	m_motor = (ndMultiBodyVehicleMotor*)*motorJoint;

	ndNode* const node = FindByBody(*motorBody);
	ndAssert(!node || ((node->m_body->GetAsBody() == *motorBody) && ((*node->m_joint == *motorJoint))));
	if (!node)
	{
		ndAssert(motorJoint->GetBody1() == GetRoot()->m_body->GetAsBody());
		AddLimb(GetRoot(), motorBody, motorJoint);
	}
	motorBody->GetAsBodyDynamic()->SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire)
{
	ndAssert(m_chassis);
	ndMatrix tireFrame(ndGetIdentityMatrix());
	tireFrame.m_front = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
	tireFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = ndVector(-1.0f, 0.0f, 0.0f, 0.0f);
	const ndMatrix chassiMatrix(m_chassis->GetMatrix());
	ndMatrix matrix(tireFrame * m_localFrame * chassiMatrix);
	matrix.m_posit = tire->GetMatrix().m_posit;

	m_iniliazed = false;
	ndBodyDynamic* const tireBody = tire->GetAsBodyDynamic();

	// make tire inertia spherical
	ndVector inertia(tireBody->GetMassMatrix());
	ndFloat32 maxInertia(ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z));
	inertia.m_x = maxInertia;
	inertia.m_y = maxInertia;
	inertia.m_z = maxInertia;
	tireBody->SetMassMatrix(inertia);

	ndSharedPtr<ndJointBilateralConstraint> tireJoint (new ndMultiBodyVehicleTireJoint(matrix, tireBody, m_chassis, desc, this));
	AddTire(tire, tireJoint);
	return m_tireList.GetLast()->GetInfo();
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddAxleTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire, const ndSharedPtr<ndBody>& axleBody)
{
	ndAssert(m_chassis);

	m_iniliazed = false;
	ndMatrix tireFrame(ndGetIdentityMatrix());
	tireFrame.m_front = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
	tireFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = ndVector(-1.0f, 0.0f, 0.0f, 0.0f);
	ndMatrix matrix(tireFrame * m_localFrame * axleBody->GetMatrix());
	matrix.m_posit = tire->GetMatrix().m_posit;
	
	ndBodyDynamic* const tireBody = tire->GetAsBodyDynamic();
	// make tire inertia spherical
	ndVector inertia(tireBody->GetMassMatrix());
	ndFloat32 maxInertia(ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z));
	inertia.m_x = maxInertia;
	inertia.m_y = maxInertia;
	inertia.m_z = maxInertia;
	tireBody->SetMassMatrix(inertia);
	
	ndSharedPtr<ndJointBilateralConstraint> tireJoint(new ndMultiBodyVehicleTireJoint(matrix, tireBody, axleBody->GetAsBodyDynamic(), desc, this));
	m_tireList.Append((ndMultiBodyVehicleTireJoint*)*tireJoint);
	ndNode* const parentNode = FindByBody(*axleBody);
	ndAssert(parentNode);
	AddLimb(parentNode, tire, tireJoint);

	tireBody->SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
	return m_tireList.GetLast()->GetInfo();
}

ndShapeInstance ndMultiBodyVehicle::CreateTireShape(ndFloat32 radius, ndFloat32 width) const
{
	ndShapeInstance tireCollision(m_tireShape);
	ndVector scale(ndFloat32 (2.0f) * width, radius, radius, 0.0f);
	tireCollision.SetScale(scale);
	return tireCollision;
}

ndBodyKinematic* ndMultiBodyVehicle::CreateInternalBodyPart(ndFloat32 mass, ndFloat32 radius) const
{
	ndShapeInstance diffCollision(new ndShapeSphere(radius));
	diffCollision.SetCollisionMode(false);

	ndBodyDynamic* const body = new ndBodyDynamic();
	ndAssert(m_chassis);
	const ndMatrix matrix(m_localFrame * m_chassis->GetMatrix());
	body->SetMatrix(matrix);
	body->SetCollisionShape(diffCollision);
	body->SetMassMatrix(mass, diffCollision);
	body->SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
	return body;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, ndFloat32 slipOmegaLock)
{
	ndAssert(m_chassis);
	ndSharedPtr<ndBody> differentialBody (CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> differentialJoint(new ndMultiBodyVehicleDifferential(differentialBody->GetAsBodyDynamic(), m_chassis, slipOmegaLock));
	AddDifferential(differentialBody, differentialJoint);
	
	m_iniliazed = false;
	const ndVector pin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_front));
	const ndVector upPin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_up));
	const ndVector drivePin(leftTire->GetBody0()->GetMatrix().RotateVector(leftTire->GetLocalMatrix0().m_front));
	
	ndSharedPtr<ndJointBilateralConstraint> leftAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin, differentialBody->GetAsBodyKinematic(), drivePin, leftTire->GetBody0()));
	ndSharedPtr<ndJointBilateralConstraint> rightAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin.Scale(ndFloat32(-1.0f)), differentialBody->GetAsBodyKinematic(), drivePin, rightTire->GetBody0()));
	AddDifferentialAxle(leftAxle);
	AddDifferentialAxle(rightAxle);

	ndMultiBodyVehicleDifferential* const joint = (ndMultiBodyVehicleDifferential*)*differentialJoint;
	return joint;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, ndFloat32 slipOmegaLock)
{
	ndAssert(m_chassis);
	ndSharedPtr<ndBody> differentialBody(CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> differentialJoint(new ndMultiBodyVehicleDifferential(differentialBody->GetAsBodyKinematic(), m_chassis, slipOmegaLock));
	AddDifferential(differentialBody, differentialJoint);

	m_iniliazed = false;
	const ndVector pin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_front));
	const ndVector upPin(differentialBody->GetMatrix().RotateVector(differentialJoint->GetLocalMatrix0().m_up));
	const ndVector drivePin(leftDifferential->GetBody0()->GetMatrix().RotateVector(leftDifferential->GetLocalMatrix0().m_front.Scale(ndFloat32(-1.0f))));
	
	ndSharedPtr<ndJointBilateralConstraint> leftAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin, differentialBody->GetAsBodyKinematic(), drivePin, leftDifferential->GetBody0()));
	ndSharedPtr<ndJointBilateralConstraint> rightAxle (new ndMultiBodyVehicleDifferentialAxle(pin, upPin.Scale(ndFloat32(-1.0f)), differentialBody->GetAsBodyKinematic(), drivePin, rightDifferential->GetBody0()));
	AddDifferentialAxle(leftAxle);
	AddDifferentialAxle(rightAxle);

	ndMultiBodyVehicleDifferential* const joint = (ndMultiBodyVehicleDifferential*)*differentialJoint;
	return joint;
}

ndMultiBodyVehicleMotor* ndMultiBodyVehicle::AddMotor(ndFloat32 mass, ndFloat32 radius)
{
	ndAssert(m_chassis);
	m_iniliazed = false;
	ndSharedPtr<ndBody> motorBody (CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> motorJoint(new ndMultiBodyVehicleMotor(motorBody->GetAsBodyKinematic(), this));
	AddMotor(motorBody, motorJoint);
	return m_motor;
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::AddGearBox(ndMultiBodyVehicleDifferential* const differential)
{
	ndAssert(m_motor);
	m_iniliazed = false;
	ndSharedPtr<ndJointBilateralConstraint> gearBox(new ndMultiBodyVehicleGearBox(m_motor->GetBody0(), differential->GetBody0(), this));
	AddGearBox(gearBox);
	return m_gearBox;
}

bool ndMultiBodyVehicle::IsSleeping() const
{
	bool sleeping = true;
	if (GetRoot())
	{
		for (ndNode* node = GetRoot()->GetFirstIterator(); sleeping && node; node = node->GetNextIterator())
		{
			ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			sleeping = sleeping && body->GetSleepState();
		}
	}
	return sleeping;
}

void ndMultiBodyVehicle::ApplyAerodynamics(ndFloat32)
{
	m_downForce.m_suspensionStiffnessModifier = ndFloat32(1.0f);
	ndFloat32 gravity = m_downForce.GetDownforceFactor(GetSpeed());
	if (ndAbs (gravity) > ndFloat32(1.0e-2f))
	{
		const ndVector up(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
		const ndVector weight(m_chassis->GetForce());
		const ndVector downForce(up.Scale(gravity * m_chassis->GetMassMatrix().m_w));
		m_chassis->SetForce(weight + downForce);
		m_downForce.m_suspensionStiffnessModifier = up.DotProduct(weight).GetScalar() / up.DotProduct(weight + downForce.Scale (0.5f)).GetScalar();
		//dTrace(("%f\n", m_suspensionStiffnessModifier));
		
		for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
		{
			ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
			ndBodyKinematic* const tireBody = tire->GetBody0();
			const ndVector tireWeight(tireBody->GetForce());
			const ndVector tireDownForce(up.Scale(gravity * tireBody->GetMassMatrix().m_w));
			tireBody->SetForce(tireWeight + tireDownForce);
		}
	}
}

bool ndMultiBodyVehicle::CalculateNormalizedAlgniningTorque(ndMultiBodyVehicleTireJoint* const, ndFloat32 sideSlipTangent) const
{
	//I need to calculate the integration of the align torque 
	//using the calculate contact patch, form the standard brush model.
	//for now just set the torque to zero.
	ndFloat32 angle = ndAtan(sideSlipTangent);
	ndFloat32 a = ndFloat32(0.1f);

	ndFloat32 slipCos(ndCos(angle));
	ndFloat32 slipSin(ndSin(angle));
	ndFloat32 y1 = ndFloat32(2.0f) * slipSin * slipCos;
	ndFloat32 x1 = -a + ndFloat32(2.0f) * slipCos * slipCos;

	ndVector p1(x1, y1, ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector p0(-a, ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));

	//static int xxxx;
	//xxxx++;
	//if (xxxx % 1000 == 0)
	//{
	//	ndTrace(("aligning torque\n"));
	//}
	//ndFloat32 alignTorque = ndFloat32(0.0f);
	//ndFloat32 sign = ndSign(alignTorque);
	//tire->m_normalizedAligningTorque = sign * ndMax(ndAbs(alignTorque), ndAbs(tire->m_normalizedAligningTorque));
	return true;
}

void ndMultiBodyVehicle::ApplyAlignmentAndBalancing()
{
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndBodyKinematic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
		ndBodyKinematic* const chassisBody = tire->GetBody1()->GetAsBodyDynamic();
	
		bool savedSleepState = tireBody->GetSleepState();
		tire->UpdateTireSteeringAngleMatrix();
		
		ndMatrix tireMatrix;
		ndMatrix chassisMatrix;
		tire->CalculateGlobalMatrix(tireMatrix, chassisMatrix);
		
		// align tire velocity
		const ndVector chassisVelocity(chassisBody->GetVelocityAtPoint(tireMatrix.m_posit));
		const ndVector relVeloc(tireBody->GetVelocity() - chassisVelocity);
		ndVector localVeloc(chassisMatrix.UnrotateVector(relVeloc));
		bool applyProjection = (localVeloc.m_x * localVeloc.m_x + localVeloc.m_z * localVeloc.m_z) > (ndFloat32(0.05f) * ndFloat32(0.05f));
		localVeloc.m_x *= ndFloat32(0.3f);
		localVeloc.m_z *= ndFloat32(0.3f);
		const ndVector tireVelocity(chassisVelocity + chassisMatrix.RotateVector(localVeloc));
		
		// align tire angular velocity
		const ndVector chassisOmega(chassisBody->GetOmega());
		const ndVector relOmega(tireBody->GetOmega() - chassisOmega);
		ndVector localOmega(chassisMatrix.UnrotateVector(relOmega));
		applyProjection = applyProjection || (localOmega.m_y * localOmega.m_y + localOmega.m_z * localOmega.m_z) > (ndFloat32(0.05f) * ndFloat32(0.05f));
		localOmega.m_y *= ndFloat32(0.3f);
		localOmega.m_z *= ndFloat32(0.3f);
		const ndVector tireOmega(chassisOmega + chassisMatrix.RotateVector(localOmega));
		
		if (applyProjection)
		{
			tireBody->SetOmega(tireOmega);
			tireBody->SetVelocity(tireVelocity);
		}
		tireBody->RestoreSleepState(savedSleepState);
	}
	
	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferential* const diff = node->GetInfo();
		diff->AlignMatrix();
	}
	
	if (m_motor)
	{
		m_motor->AlignMatrix();
	}
}

void ndMultiBodyVehicle::Debug(ndConstraintDebugCallback& context) const
{
	if (!GetRoot())
	{
		return;
	}

	// draw vehicle coordinade system;
#if 0
	const ndBodyKinematic* const chassis = m_chassis;
	ndAssert(chassis);
	ndMatrix chassisMatrix(chassis->GetMatrix());
	chassisMatrix.m_posit = chassisMatrix.TransformVector(chassis->GetCentreOfMass());
	context.DrawFrame(chassisMatrix);
	
	ndFloat32 totalMass = chassis->GetMassMatrix().m_w;
	ndVector effectiveCom(chassisMatrix.m_posit.Scale(totalMass));
	
	// draw front direction for side slip angle reference
	// draw velocity vector
	ndVector veloc(chassis->GetVelocity());
	ndVector p0(chassisMatrix.m_posit + m_localFrame.m_up.Scale(1.0f));
	ndVector p1(p0 + chassisMatrix.RotateVector(m_localFrame.m_front).Scale(2.0f));
	ndVector p2(p0 + veloc.Scale (0.25f));
	
	context.DrawLine(p0, p2, ndVector(1.0f, 1.0f, 0.0f, 0.0f));
	context.DrawLine(p0, p1, ndVector(1.0f, 0.0f, 0.0f, 0.0f));

	// calculate beta angle
	//ndVector localVeloc(chassisMatrix.UnrotateVector(m_localFrame.UnrotateVector(veloc)));
	//if (ndAbs (localVeloc.m_x) > ndFloat32 (1.0f))
	//{
	//	ndFloat32 sideslip = ndAtan2(localVeloc.m_z, localVeloc.m_x);
	//	ndTrace(("beta=%f  v(%f %f %f)\n", sideslip* ndRadToDegree, localVeloc.m_x, localVeloc.m_y, localVeloc.m_z));
	//	if (ndAbs(sideslip * ndRadToDegree) > 45.0f)
	//	{
	//		sideslip = ndAtan2(localVeloc.m_z, localVeloc.m_x);
	//	}
	//}
	//// draw body acceleration
	////ndVector accel(m_chassis->GetAccel());
	////ndVector p3(p0 + accel.Scale(0.5f));
	////context.DrawLine(p0, p3, ndVector(0.0f, 1.0f, 1.0f, 0.0f));
	
	ndFloat32 scale = ndFloat32 (3.0f);
	ndVector weight(chassis->GetForce().Scale (scale * chassis->GetInvMass() / m_downForce.m_gravity));
	
	// draw vehicle weight;
	ndVector forceColor(ndFloat32 (0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.0f));
	ndVector lateralColor(ndFloat32(0.3f), ndFloat32(0.7f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector longitudinalColor(ndFloat32(0.7f), ndFloat32(0.3f), ndFloat32(0.0f), ndFloat32(0.0f));
	context.DrawLine(chassisMatrix.m_posit, chassisMatrix.m_posit + weight, forceColor);
	
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();
		ndMatrix tireFrame(tireBody->GetMatrix());
		totalMass += tireBody->GetMassMatrix().m_w;
		effectiveCom += tireFrame.m_posit.Scale(tireBody->GetMassMatrix().m_w);
	}
	
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		tireJoint->DebugJoint(context);
	
		// draw tire forces
		const ndBodyKinematic::ndContactMap& contactMap = tireBody->GetContactMap();
		ndFloat32 tireGravities = scale /(totalMass * m_downForce.m_gravity);
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					ndMatrix frame(contactPoint.m_normal, contactPoint.m_dir0, contactPoint.m_dir1, contactPoint.m_point);
	
					ndVector localPosit(m_localFrame.UntransformVector(chassisMatrix.UntransformVector(contactPoint.m_point)));
					ndFloat32 offset = (localPosit.m_z > ndFloat32(0.0f)) ? ndFloat32(0.2f) : ndFloat32(-0.2f);
					frame.m_posit += contactPoint.m_dir0.Scale(offset);
					frame.m_posit += contactPoint.m_normal.Scale(0.1f);
	
					// normal force
					ndFloat32 normalForce = -tireGravities * contactPoint.m_normal_Force.m_force;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_normal.Scale (normalForce), forceColor);
	
					// lateral force
					ndFloat32 lateralForce = -tireGravities * contactPoint.m_dir0_Force.m_force;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir0.Scale(lateralForce), lateralColor);
	
					// longitudinal force
					ndFloat32 longitudinalForce = tireGravities * contactPoint.m_dir1_Force.m_force;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir1.Scale(longitudinalForce), longitudinalColor);
				}
			}
		}
	}
	
	effectiveCom = effectiveCom.Scale(ndFloat32(1.0f) / totalMass);
	chassisMatrix.m_posit = effectiveCom;
	chassisMatrix.m_posit.m_w = ndFloat32(1.0f);
	context.DrawFrame(chassisMatrix);
#endif

	const ndBodyKinematic* const chassis = m_chassis;
	ndAssert(chassis);
	const ndMatrix chassisMatrix(chassis->GetMatrix());

	// draw center of mass;
	const ndCenterOfMassDynamics kinematioc (CalculateCentreOfMassKinematics(chassisMatrix));
	context.DrawFrame(kinematioc.m_centerOfMass);

	// draw vehicle velocity
	const ndVector veloc(kinematioc.m_veloc);
	const ndVector p0(kinematioc.m_centerOfMass.m_posit + kinematioc.m_centerOfMass.m_up.Scale(1.5f));
	const ndVector p1(p0 + kinematioc.m_centerOfMass.m_front.Scale(2.0f));
	const ndVector p2(p0 + kinematioc.m_centerOfMass.RotateVector(veloc.Scale(0.25f)));

	context.DrawLine(p0, p2, ndVector(1.0f, 1.0f, 0.0f, 0.0f));
	context.DrawLine(p0, p1, ndVector(1.0f, 0.0f, 0.0f, 0.0f));

	// draw tires info
	ndFloat32 scale = ndFloat32(3.0f);
	const ndVector forceColor(ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.0f));
	const ndVector lateralColor(ndFloat32(0.3f), ndFloat32(0.7f), ndFloat32(0.0f), ndFloat32(0.0f));
	const ndVector longitudinalColor(ndFloat32(0.7f), ndFloat32(0.3f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector weight(chassis->GetForce().Scale(scale * chassis->GetInvMass() / m_downForce.m_gravity));
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		tireJoint->DebugJoint(context);

		// draw tire forces
		const ndBodyKinematic::ndContactMap& contactMap = tireBody->GetContactMap();
		ndFloat32 tireGravities = scale / (kinematioc.m_mass * m_downForce.m_gravity);
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					ndMatrix frame(contactPoint.m_normal, contactPoint.m_dir0, contactPoint.m_dir1, contactPoint.m_point);

					ndVector localPosit(m_localFrame.UntransformVector(chassisMatrix.UntransformVector(contactPoint.m_point)));
					ndFloat32 offset = (localPosit.m_z > ndFloat32(0.0f)) ? ndFloat32(0.2f) : ndFloat32(-0.2f);
					frame.m_posit += contactPoint.m_dir0.Scale(offset);
					frame.m_posit += contactPoint.m_normal.Scale(0.1f);

					// normal force
					ndFloat32 normalForce = -tireGravities * contactPoint.m_normal_Force.m_force;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_normal.Scale(normalForce), forceColor);

					// lateral force
					ndFloat32 lateralForce = -tireGravities * contactPoint.m_dir0_Force.m_force;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir0.Scale(lateralForce), lateralColor);

					// longitudinal force
					ndFloat32 longitudinalForce = tireGravities * contactPoint.m_dir1_Force.m_force;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir1.Scale(longitudinalForce), longitudinalColor);
				}
			}
		}
	}
}

bool ndMultiBodyVehicle::CoulombTireModel(ndMultiBodyVehicleTireJoint* const joint, ndContactMaterial& contactPoint) const
{
	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;

	// handling dynamics friction manually
	ndFloat32 dynamicFrictionCoef = joint->m_IsApplyingBreaks ? ndFloat32(0.75f) : ndFloat32(1.0f);

	contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = frictionCoefficient;
	contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient * dynamicFrictionCoef;
	contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient * dynamicFrictionCoef;
	return true;
}

bool ndMultiBodyVehicle::CoulombFrictionCircleTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	return BrushTireModel(tire, contactPoint);
}

bool ndMultiBodyVehicle::BrushTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	// calculate longitudinal slip ratio
	const ndBodyKinematic* const chassis = m_chassis;
	ndAssert(chassis);
	const ndBodyKinematic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
	const ndBodyKinematic* const otherBody = (contactPoint.m_body0 == tireBody) ? ((ndBodyKinematic*)contactPoint.m_body1)->GetAsBodyDynamic() : ((ndBodyKinematic*)contactPoint.m_body0)->GetAsBodyDynamic();
	ndAssert(tireBody != otherBody);
	ndAssert((tireBody == contactPoint.m_body0) || (tireBody == contactPoint.m_body1));

	//tire non linear brush model is only considered 
	//when is moving faster than 0.5 m/s (approximately 1.0 miles / hours) 
	//this is just an arbitrary limit, based of the model 
	//not been defined for stationary tires.
	const ndVector contactVeloc0(tireBody->GetVelocity());
	const ndVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
	const ndVector relVeloc(contactVeloc0 - contactVeloc1);
	const ndVector longitudDir(contactPoint.m_dir0);
	const ndFloat32 relSpeed = relVeloc.DotProduct(longitudDir).GetScalar();

	if (ndAbs(relSpeed) < D_MAX_CONTACT_SPEED_TRESHOLD)
	{
		return CoulombTireModel(tire, contactPoint);
	}
	// tire is in breaking and traction mode.
	const ndVector contactVeloc(tireBody->GetVelocityAtPoint(contactPoint.m_point) - contactVeloc1);

	const ndFloat32 vr = -contactVeloc.DotProduct(longitudDir).GetScalar();
	const ndFloat32 longitudialSlip = ndClamp(vr / relSpeed, ndFloat32(-0.99f), ndFloat32(100.0f));

	const ndVector lateralDir(contactPoint.m_dir1);
	const ndFloat32 sideSpeed = relVeloc.DotProduct(lateralDir).GetScalar();
	const ndFloat32 signedLateralSlip = sideSpeed / (relSpeed + ndFloat32(1.0f));
	const ndFloat32 lateralSlip = ndAbs(signedLateralSlip);
	//CalculateNormalizedAlgniningTorque(tire, lateralSlip);
	//CalculateNormalizedAlgniningTorque(tire, signedLateralSlip);

	tire->m_lateralSlip = ndMax(tire->m_lateralSlip, lateralSlip);
	tire->m_longitudinalSlip = ndMax(tire->m_longitudinalSlip, longitudialSlip);

	const ndFloat32 den = ndFloat32(1.0f) / (longitudialSlip + ndFloat32(1.0f));
	const ndFloat32 v = lateralSlip * den;
	const ndFloat32 u = longitudialSlip * den;

	const ndTireFrictionModel& info = tire->m_frictionModel;
	const ndFloat32 sprungMassWheigh = ndFloat32(m_tireList.GetCount() ? m_tireList.GetCount() : 1);
	const ndFloat32 vehicleMass = chassis->GetMassMatrix().m_w / sprungMassWheigh;
	const ndFloat32 cz = ndAbs(vehicleMass * info.m_brush.m_laterialStiffness * v);
	const ndFloat32 cx = ndAbs(vehicleMass * info.m_brush.m_longitudinalStiffness * u);

	const ndFloat32 gamma = ndMax(ndSqrt(cx * cx + cz * cz), ndFloat32(1.0e-8f));
	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;
	const ndFloat32 normalForce = contactPoint.m_normal_Force.GetInitialGuess() + ndFloat32(1.0f);

	const ndFloat32 maxForceForce = frictionCoefficient * normalForce;
	ndFloat32 f = maxForceForce;
	if (gamma < (ndFloat32(3.0f) * maxForceForce))
	{
		const ndFloat32 b = ndFloat32(1.0f) / (ndFloat32(3.0f) * maxForceForce);
		const ndFloat32 c = ndFloat32(1.0f) / (ndFloat32(27.0f) * maxForceForce * maxForceForce);
		f = gamma * (ndFloat32(1.0f) - b * gamma + c * gamma * gamma);
	}

	const ndFloat32 lateralForce = f * cz / gamma;
	const ndFloat32 longitudinalForce = f * cx / gamma;
	//ndTrace(("(%d: u=%f f1=%f f2=%f)  ", tireBody->GetId(), longitudialSlip, longitudinalForce, lateralForce));

	ndFloat32 dynamicFrictionCoef = tire->m_IsApplyingBreaks ? ndFloat32(0.5f) : ndFloat32(1.0f);

	contactPoint.m_material.m_staticFriction1 = lateralForce * dynamicFrictionCoef;
	contactPoint.m_material.m_dynamicFriction1 = lateralForce * dynamicFrictionCoef;
	contactPoint.m_material.m_staticFriction0 = longitudinalForce * dynamicFrictionCoef;
	contactPoint.m_material.m_dynamicFriction0 = longitudinalForce * dynamicFrictionCoef;

	ndUnsigned32 newFlags = contactPoint.m_material.m_flags | m_override0Friction | m_override1Friction;
	contactPoint.m_material.m_flags = newFlags;

	//ndTrace(("brush(%f %f) ", longitudinalForce, lateralForce));
	//ndTrace(("brush(%f) ", longitudinalForce));
	return true;
}

void ndMultiBodyVehicle::AddDifferential(const ndSharedPtr<ndBody>& differentialBody, const ndSharedPtr<ndJointBilateralConstraint>& differentialJoint)
{
	ndAssert(!strcmp(differentialJoint->ClassName(), "ndMultiBodyVehicleDifferential"));
	m_differentialList.Append((ndMultiBodyVehicleDifferential*)*differentialJoint);

	ndNode* const node = FindByBody(*differentialBody);
	ndAssert(!node || ((node->m_body->GetAsBody() == *differentialBody) && ((*node->m_joint == *differentialJoint))));
	if (!node)
	{
		ndAssert(differentialJoint->GetBody1() == GetRoot()->m_body->GetAsBody());
		AddLimb(GetRoot(), differentialBody, differentialJoint);
	}
	differentialBody->GetAsBodyDynamic()->SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(10.0f));
}

void ndMultiBodyVehicle::AddDifferentialAxle(const ndSharedPtr<ndJointBilateralConstraint>& differentialAxleJoint)
{
	ndNode* const node = FindLoopByJoint(*differentialAxleJoint);
	if (!node)
	{
		AddCloseLoop(differentialAxleJoint);
	}
}

void ndMultiBodyVehicle::AddGearBox(const ndSharedPtr<ndJointBilateralConstraint>& gearBoxJoint)
{
	ndNode* const node = FindLoopByJoint(*gearBoxJoint);
	m_gearBox = (ndMultiBodyVehicleGearBox*)*gearBoxJoint;
	if (!node)
	{
		AddCloseLoop(gearBoxJoint);
	}
}

void ndMultiBodyVehicle::ApplyStabilityControl()
{
	ndAssert(m_chassis);
	const ndBodyKinematic* const chassis = m_chassis;
	const ndVector veloc(chassis->GetVelocity());
	const ndMatrix chassisMatrix(chassis->GetMatrix());

#if 0
	// control sideslip beta by manipulation the steering
	// ignoring beta rate

	// this is really terrible
	const ndVector localVeloc(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(veloc)));
	if (ndAbs(localVeloc.m_x) > ndFloat32(1.0f))
	{
		ndFloat32 sideslip = ndAtan2(localVeloc.m_z, localVeloc.m_x);
		if (ndAbs(sideslip * ndRadToDegree) > m_maxSideslipAngle)
		{
			ndFloat32 targetSteering = (sideslip > 0.0f) ? ndFloat32(-1.0f) : ndFloat32(1.0f);
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				if (tire->m_info.m_steeringAngle != 0)
				{
					//ndFloat32 steering = tire->m_normalizedSteering0 + (targetSteering - tire->m_normalizedSteering0) * m_steeringRate;
					ndFloat32 steering = tire->m_normalizedSteering0 + (targetSteering - tire->m_normalizedSteering0) * 0.002;
					tire->m_normalizedSteering = steering;
				}
			}
		}
		else
		{
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				if (tire->m_info.m_steeringAngle != 0)
				{
					ndFloat32 steering = tire->m_normalizedSteering0 + (tire->m_normalizedSteering - tire->m_normalizedSteering0) * m_steeringRate;
					tire->m_normalizedSteering = steering;
				}
			}
		}
	}

#elif 1
	// control beta rate by manipulation the steering
	// this may not be the be mode, but it does works;

	// this seem to be the best controller I got, but I really need a closed loop control
	const ndVector localVeloc(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(veloc)));
	if (ndAbs(localVeloc.m_x) > ndFloat32(1.0f))
	{
		ndFloat32 sideslip = ndAtan2(localVeloc.m_z, localVeloc.m_x);
		if (ndAbs(sideslip * ndRadToDegree) > m_maxSideslipAngle)
		{
			const ndVector omega(chassis->GetOmega());
			const ndVector accel(chassis->GetAccel());
			const ndVector localOmega(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(omega)));
			const ndVector localAccel(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(accel)));

			// From Giancarlo Genta's book *Motor Vehicle Dynamics* (page 231, equation 5.52)
			// Original equation:
			// lateralAcceleration = longitudinalSpeed * (betaRate + yawRate) + beta * longitudinalAcceleration
			//
			// Note: When deriving the equation in a y-up coordinate system, it transforms into:
			// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate) + beta * longitudinalAcceleration
			// 
			// In my opinion, this version makes more sense.
			//
			// Assuming constant longitudinal velocity, the term beta * longitudinalAcceleration becomes zero:
			// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate)
			// from where we can get the beta rate
			// betaRate = lateralAcceleration / longitudinalSpeed + yawRate;
			ndFloat32 betaRate = localAccel.m_z / localVeloc.m_x + localOmega.m_y;

			//ndTrace(("%d: betaRate %f = %f + %f\n", xxxxx, betaRate, localAccel.m_z / localVeloc.m_x, localOmega.m_y));
			if (ndAbs(betaRate) > m_maxSideslipRate)
			{
				ndFloat32 targetSteering = (betaRate > m_maxSideslipRate) ? ndFloat32(1.0f) : ndFloat32(-1.0f);
				//ndTrace(("a=%f b=%f b'=%f fz=%f w=%f steer=(", localAccel.m_z, sideslip * ndRadToDegree, betaRate, sideslipRate, localOmega.m_y));
				for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
					if (tire->m_info.m_steeringAngle != 0)
					{
						ndFloat32 steering = tire->m_normalizedSteering0 + (targetSteering - tire->m_normalizedSteering0) * m_steeringRate * 0.5f;
						tire->m_normalizedSteering = steering;
					}
				}
			}
			else
			{
				for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
					if (tire->m_info.m_steeringAngle != 0)
					{
						ndFloat32 steering = tire->m_normalizedSteering0 + (tire->m_normalizedSteering - tire->m_normalizedSteering0) * m_steeringRate;
						tire->m_normalizedSteering = steering;
					}
				}
			}
		}
	}

#else

	const ndVector omega(chassis->GetOmega());
	const ndVector accel(chassis->GetAccel());
	const ndVector alpha(chassis->GetAlpha());
	const ndVector localVeloc(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(veloc)));
	const ndVector localOmega(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(omega)));
	const ndVector localAccel(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(accel)));
	const ndVector localAlpha(m_localFrame.UnrotateVector(chassisMatrix.UnrotateVector(alpha)));

	if (ndAbs(localVeloc.m_x) > ndFloat32(3.0f))
	{
		// From Giancarlo Genta's book *Motor Vehicle Dynamics* (page 231, equation 5.52)
		// Original equation:
		// lateralAcceleration = longitudinalSpeed * (betaRate + yawRate) + beta * longitudinalAcceleration
		//
		// Note: When deriving the equation in a y-up coordinate system, it transforms into:
		// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate) + beta * longitudinalAcceleration
		// 
		// In my opinion, this version makes more sense.
		//
		// Assuming constant longitudinal velocity, the term beta * longitudinalAcceleration becomes zero:
		// lateralAcceleration = longitudinalSpeed * (betaRate - yawRate)
		// from where we can get the beta rate
		// betaRate = lateralAcceleration / longitudinalSpeed + yawRate;
		ndFloat32 betaRate = localAccel.m_z / localVeloc.m_x + localOmega.m_y;
		//if (ndAbs(betaRate) > D_MAX_SIZE_SLIP_RATE)
		if (ndAbs(betaRate) > ndFloat32 (0.15f))
		{
			const ndMatrix vehicleMatrix(m_chassis->GetMatrix());
			const ndVector com(vehicleMatrix.TransformVector(m_chassis->GetCentreOfMass()));
			for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
			{
				ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
				const ndMatrix hubMatrix(tire->CalculateBaseFrame());
				const ndVector hubPosit(hubMatrix.m_posit - com);
				const ndVector tireTorque(hubPosit.CrossProduct(tire->GetForceBody1()));
				const ndVector locaTorque(m_localFrame.UnrotateVector(vehicleMatrix.UnrotateVector(tireTorque)));

				ndVector xxx(tire->GetForceBody1());
				ndVector force1(m_localFrame.UnrotateVector(hubMatrix.UnrotateVector(tire->GetForceBody1())));
				ndVector force0(tire->GetForceBody0());

				if (betaRate < 0.0f)
				{
					if (tire->GetBody0()->GetId() == 4)
					{
						ndTrace(("applyBreakControl: "));
						tire->SetHandBreak(0.02f);
					}
				}
				else
				{
					if (tire->GetBody0()->GetId() == 3)
					{
						ndTrace(("applyBreakControl: "));
						tire->SetHandBreak(0.02f);
					}
				}
			}
		}
		ndTrace(("%d: betaRate %f = %f - %f;  YawRate = %f\n", xxxxx, betaRate, localAccel.m_z / localVeloc.m_x, localOmega.m_y, localAlpha.m_y));
	}
#endif

	//xxxxx++;
}

void ndMultiBodyVehicle::ApplyTireModel(ndFixSizeArray<ndTireContactPair, 128>& tireContacts)
{
	ndInt32 savedContactCount = tireContacts.GetCount();
	for (ndInt32 i = tireContacts.GetCount() - 1; i >= 0; --i)
	{
		ndContact* const contact = tireContacts[i].m_contact;
		ndMultiBodyVehicleTireJoint* const tire = tireContacts[i].m_tireJoint;
		ndContactPointList& contactPoints = contact->GetContactPoints();
		ndMatrix tireBasisMatrix(tire->GetLocalMatrix1() * tire->GetBody1()->GetMatrix());
		tireBasisMatrix.m_posit = tire->GetBody0()->GetMatrix().m_posit;
		bool useCoulombModel = (tire->m_frictionModel.m_frictionModel == ndTireFrictionModel::ndFrictionModel::m_coulomb) ? true : false;

		const ndVector tireUp(m_localFrame.UnrotateVector(tireBasisMatrix.m_up));
		const ndVector tireFront(m_localFrame.UnrotateVector(tireBasisMatrix.m_front));
		for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
		{
			ndContactMaterial& contactPoint = contactNode->GetInfo();
			const ndVector localNormal(m_localFrame.UntransformVector(contactPoint.m_normal));
			ndFloat32 contactPathLocation = ndAbs(localNormal.DotProduct(tireFront).GetScalar());
			if (contactPathLocation < ndFloat32(0.71f))
			{
				// align tire friction direction
				const ndVector longitudinalDir(localNormal.CrossProduct(tireFront).Normalize());
				const ndVector lateralDir(longitudinalDir.CrossProduct(localNormal));

				contactPoint.m_dir1 = m_localFrame.RotateVector(lateralDir);
				contactPoint.m_dir0 = m_localFrame.RotateVector(longitudinalDir);

				bool isOutOfContactPatch = useCoulombModel;
				if (!isOutOfContactPatch)
				{
					// check if the contact is in the contact patch,
					// the is the 45 degree point around the tire vehicle axis. 
					const ndVector dir(m_localFrame.UnrotateVector(contactPoint.m_point - tireBasisMatrix.m_posit));
					ndAssert(dir.DotProduct(dir).GetScalar() > ndFloat32(0.0f));
					ndFloat32 contactPatch = tireUp.DotProduct(dir.Normalize()).GetScalar();
					isOutOfContactPatch = (contactPatch > ndFloat32(-0.71f));
				}
				if (isOutOfContactPatch)
				{
					// remove this contact
					tireContacts[i] = tireContacts[tireContacts.GetCount() - 1];
					tireContacts.Pop();
					break;
				}
			}
		}
	}

	if (tireContacts.GetCount() && (tireContacts.GetCount() == savedContactCount))
	{
		//ndTrace (("frame:%d ", xxxxx))
		//xxxxx++;

		for (ndInt32 i = tireContacts.GetCount() - 1; i >= 0 ; --i)
		{
			ndContact* const contact = tireContacts[i].m_contact;
			ndMultiBodyVehicleTireJoint* const tire = tireContacts[i].m_tireJoint;
			ndContactPointList& contactPoints = contact->GetContactPoints();
			for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
			{
				ndContactMaterial& contactPoint = contactNode->GetInfo();
				switch (tire->m_frictionModel.m_frictionModel)
				{
					case ndTireFrictionModel::m_brushModel:
					{
						BrushTireModel(tire, contactPoint);
						break;
					}

					case ndTireFrictionModel::m_pacejkaSport:
					case ndTireFrictionModel::m_pacejkaTruck:
					case ndTireFrictionModel::m_pacejkaUtility:
					{
						if (PacejkaTireModel(tire, contactPoint))
						{
							contact->InvalicatdeCache();
						}
						break;
					}

					case ndTireFrictionModel::m_coulombCicleOfFriction:
					{
						CoulombFrictionCircleTireModel(tire, contactPoint);
						break;
					}

					case ndTireFrictionModel::m_coulomb:
					default:
					{
						CoulombTireModel(tire, contactPoint);
						break;
					}
				}
			}
		}
		//ApplyStabilityControl();
		//ndTrace(("\n"));
	}
}

void ndMultiBodyVehicle::ApplyTireModel()
{
	ndFixSizeArray<ndTireContactPair, 128> tireContacts;
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndAssert(((ndShape*)tire->GetBody0()->GetCollisionShape().GetShape())->GetAsShapeChamferCylinder());

		tire->m_lateralSlip = ndFloat32(0.0f);
		tire->m_longitudinalSlip = ndFloat32(0.0f);
		tire->m_normalizedAligningTorque = ndFloat32(0.0f);

		const ndBodyKinematic::ndContactMap& contactMap = tire->GetBody0()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				ndContactPointList& contactPoints = contact->GetContactPoints();
				// for mesh collision we need to remove contact duplicates, 
				// these are contact produced by two or more polygons, 
				// that can produce two contact so are close that they can generate 
				// ill formed rows in the solver mass matrix
				for (ndContactPointList::ndNode* contactNode0 = contactPoints.GetFirst(); contactNode0; contactNode0 = contactNode0->GetNext())
				{
					const ndContactPoint& contactPoint0 = contactNode0->GetInfo();
					for (ndContactPointList::ndNode* contactNode1 = contactNode0->GetNext(); contactNode1; contactNode1 = contactNode1->GetNext())
					{
						const ndContactPoint& contactPoint1 = contactNode1->GetInfo();
						const ndVector error(contactPoint1.m_point - contactPoint0.m_point);
						ndFloat32 err2 = error.DotProduct(error).GetScalar();
						if (err2 < D_MIN_CONTACT_CLOSE_DISTANCE2)
						{
							contactPoints.Remove(contactNode1);
							break;
						}
					}
				}
				ndTireContactPair pair;
				pair.m_contact = contact;
				pair.m_tireJoint = tire;
				tireContacts.PushBack(pair);
			}
		}
	}

	ApplyTireModel(tireContacts);

	// save the steering
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		tire->m_normalizedSteering0 = tire->m_normalizedSteering;
	}
}

bool ndMultiBodyVehicle::PacejkaTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	// from Wikipedia the Pacejka equation is write as, but it does not include phi.
	//F = D * sin(C * atan(Bx * (1 - E) + E * atan(Bx)))

	// From Giancarlo Genta, page 60: side and verticla shit are introduced to allow the formula 
	// to be used even when the vehicle is at rest.
	// F = D * sin(C * atan(Bx * (1 - E) * (phi + Sh) + E * atan(Bx * (phi + Sh)))) + Sv
	// My main issue with this formula is the difficulty in determining the parameters 
	// C, D, E, Bx, phi, Sh, and Sv for each force and moment.
	// Genta's book provides five tables of coefficients for five different vehicles, 
	// but nowhere does it clearly relate those coefficients to the a1 trought a13
	// to the Magic Formula parameters B, C, D, and E. 
	// Instead you have to get them form a diffren example
	// The closest reference I’ve found is this paper: 
	// http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf

	// Regarding the Pacejka implementation:
	// In Genta’s book, from pages 60 to 78, the unit handling is extremely inconsistent, 
	// to the point where the results become meaningless, in my opinion.
	// I’ve attempted to implement this model for decades, 
	// but have never been able to achieve correct results.
	// Currently, I’m comparing it to the Brush model, which appears to be more reasonable.
	// foe example the force extarec for Pacejka are independet of the normal force onteh tire
	// instead is use the paremeter D, 
	// to me this violates common sence, Imagine a pick truck, 
	// on this formual the loteral forces will be the same regaless of the truck load.

	const ndBodyKinematic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
	const ndBodyKinematic* const otherBody = (contactPoint.m_body0 == tireBody) ? ((ndBodyKinematic*)contactPoint.m_body1)->GetAsBodyDynamic() : ((ndBodyKinematic*)contactPoint.m_body0)->GetAsBodyDynamic();

	const ndFloat32 normalForce = contactPoint.m_normal_Force.GetInitialGuess() + ndFloat32(1.0f);
	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;

	const ndVector longitudDir(contactPoint.m_dir0);
	const ndVector contactVeloc0(tireBody->GetVelocity());
	const ndVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
	const ndVector wheelComVeloc(contactVeloc0 - contactVeloc1);
	const ndFloat32 wheelComSpeed_x = wheelComVeloc.DotProduct(longitudDir).GetScalar();
	if (ndAbs(wheelComSpeed_x) < D_MAX_CONTACT_SPEED_TRESHOLD)
	{
		// handle vehicle is a rest bu just doing normal rigid body dynamics.
		return true;
	}

	// calculate lateral slip angle
	const ndVector lateralDir(contactPoint.m_dir1);
	// use a dead zone and them use Sv for nonzero lateral force
	const ndFloat32 speed_z = wheelComVeloc.DotProduct(lateralDir).GetScalar();
	const ndFloat32 wheelComSpeed_z = (speed_z > ndFloat32(1.0e-3f) || (speed_z < ndFloat32(1.0e-3f))) ? speed_z : ndFloat32(0.0f);
	const ndFloat32 sideSlipAngleInRadians = ndAtan2(wheelComSpeed_z, wheelComSpeed_x);
	tire->m_lateralSlip = ndMax(tire->m_lateralSlip, ndAbs(sideSlipAngleInRadians));

	// calculate longitudinal slip
	const ndVector contactVeloc(tireBody->GetVelocityAtPoint(contactPoint.m_point) - contactVeloc1);
	const ndFloat32 vr_x = contactVeloc.DotProduct(longitudDir).GetScalar();
	const ndFloat32 longitudialSlip = ndClamp(vr_x / wheelComSpeed_x, ndFloat32(-100.0f), ndFloat32(100.0f));
	tire->m_longitudinalSlip = ndMax(tire->m_longitudinalSlip, longitudialSlip);

	//I am now using the direct coeficients B, C, E, D as explained in 
	//The multibody system approach to vehicle dynamics page 300 to 306
	auto LongitudinalForce = [](const ndTireFrictionModel::ndPacejkaTireModel& model, ndFloat32 normalForce, ndFloat32 phi)
	{
		return model.Evaluate(phi, normalForce);
	};

	auto LateralForce = [](const ndTireFrictionModel::ndPacejkaTireModel& model, ndFloat32 normalForce, ndFloat32 phi)
	{
		// phi is in degress
		phi = ndAbs(ndRadToDegree * phi);
		return model.Evaluate(phi, normalForce);
	};

	//now apply the combine effect, according to Genta book page 76
	const ndTireFrictionModel& frictionModel = tire->m_frictionModel;
	//ndFloat32 xxx0 = LateralForce(frictionModel.m_lateralPacejka, 10.0f, 5.0f * ndDegreeToRad);
	//ndFloat32 xxx1 = LongitudinalForce(frictionModel.m_longitudinalPacejka, 10.0f, 4.0f);

	ndFloat32 normalFrictionForce = normalForce * frictionCoefficient;
	normalFrictionForce = 3000.0f * frictionCoefficient;

	// Calculate combined forces.
	// I have no idea how this derivation came about.
	// There's no explanation in either Pacejka or Genta’s books, 
	// just an example where these relationships are vaguely outlined.
	// For almost 20 years, this has seemed like questionable engineering fluff.
	// The equations don't make sense dimensionally, and they even mix units 
	// within the same expressions.
	const ndFloat32 dimensionLessPhi_x = longitudialSlip / frictionModel.m_longitudinalPacejka.m_normalizingPhi;
	const ndFloat32 dimensionLessPhi_z = sideSlipAngleInRadians / frictionModel.m_lateralPacejka.m_normalizingPhi;

	// since Sv and Sh are zero of very small, them delta alpha is zero. 
	const ndFloat32 dimensionLessCombined_phi = ndSqrt(dimensionLessPhi_x * dimensionLessPhi_x + dimensionLessPhi_z * dimensionLessPhi_z);

	const ndFloat32 modifiedPhi_x = longitudialSlip * dimensionLessCombined_phi;
	const ndFloat32 modifiedPhiInRadians_z = sideSlipAngleInRadians * dimensionLessCombined_phi;

	ndFloat32 pureFz = LateralForce(frictionModel.m_lateralPacejka, normalFrictionForce, modifiedPhiInRadians_z);
	ndFloat32 pureFx = LongitudinalForce(frictionModel.m_longitudinalPacejka, normalFrictionForce, modifiedPhi_x);

	// after we calculate the compensated longitudinal and lateral forces,
	// they have to be scaled back
	ndFloat32 fx = pureFx * dimensionLessPhi_x / dimensionLessCombined_phi;
	ndFloat32 fz = pureFz * dimensionLessPhi_z / dimensionLessCombined_phi;
	
#if 0
	// plot the graph 
	FILE* outFile;
	outFile = fopen("force.csv", "wb");
	fprintf(outFile, "fx; fz, phi\n");
	for (ndFloat32 x = -20.0f; x < 20.0f; x += 0.01f)
	{
		ndFloat32 fx = LongitudinalForce(pacejkaLongitudical, 1000.0f, x);
		ndFloat32 fz = LateralForce(pacejkaLateral, 1000.0f, x * ndDegreeToRad);
		fprintf(outFile, "%g; %g; %g\n", fx, fz, x);
	}
	fclose(outFile);
#endif
	
	//BrushTireModel(tire, contactPoint);
	//ndTrace(("(%d %f %f) ", tireBody->GetId(), fx, fz));
	
	contactPoint.m_material.m_staticFriction1 = ndAbs(fz);
	contactPoint.m_material.m_dynamicFriction1 = ndAbs(fz);
	contactPoint.m_material.m_staticFriction0 = ndAbs(fx);
	contactPoint.m_material.m_dynamicFriction0 = ndAbs(fx);
	ndUnsigned32 newFlags = contactPoint.m_material.m_flags | m_override0Friction | m_override1Friction;
	contactPoint.m_material.m_flags = newFlags;
	return true;
}

void ndMultiBodyVehicle::Update(ndFloat32 timestep)
{
	if (!m_iniliazed)
	{
		FinalizeBuild();
	}

	// apply down force
	ApplyAerodynamics(timestep);

	// apply tire model
	ApplyTireModel();
}

void ndMultiBodyVehicle::PostUpdate(ndFloat32)
{
	ApplyAlignmentAndBalancing();
}
