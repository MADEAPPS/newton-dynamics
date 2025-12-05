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
#include "ndJointHinge.h"
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
//	:m_gravity(ndFloat32(-10.0f))
	:m_suspensionStiffnessModifier(ndFloat32(1.0f))
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
	//return downForceFactor * m_gravity;
	return downForceFactor;
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

ndMultiBodyVehicle::ndMultiBodyVehicle(ndFloat32 gravityMagnitud)
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

	m_gravityMagnitud = -ndAbs(gravityMagnitud);
	ndAssert(ndAbs(m_gravityMagnitud) > ndFloat32 (0.0f));

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
	ndFixSizeArray<ndMultiBodyVehicleTireJoint*, 64> tireArray;

	const ndVector upDir(m_localFrame.m_up.Scale(ndFloat32 (1.0f)));
	const ndMatrix savedMatrix(GetRoot()->m_body->GetMatrix());
	SetTransform(ndGetIdentityMatrix());

	bodyId.PushBack(0);
	bodyArray.PushBack(GetWorld()->GetSentinelBody());
	invMass.PushBack(ndFloat32 (0.0f));
	invInertia.PushBack(ndGetZeroMatrix());

	ndInt32 bodyIndex = 1;

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
			const ndVector r0(joint->CalculateGlobalMatrix0().m_posit - com0);
			const ndVector r1(joint->CalculateGlobalMatrix1().m_posit - com1);

			ndJacobianPair jacobianPair;
			jacobianPair.m_jacobianM0.m_linear = upDir.Scale(ndFloat32(-1.0f));
			jacobianPair.m_jacobianM0.m_angular = jacobianPair.m_jacobianM0.m_linear.CrossProduct(r0);

			jacobianPair.m_jacobianM1.m_linear = upDir;
			jacobianPair.m_jacobianM1.m_angular = jacobianPair.m_jacobianM1.m_linear.CrossProduct(r1);
				
			jacobianArray.PushBack(jacobianPair);

			ndIndexPair pair;
			pair.m_m0 = m0;
			pair.m_m1 = m1;
			jointArray.PushBack(pair);
			rhsAccel.PushBack(ndFloat32(0.0f));
			tireArray.PushBack(nullptr);

			if (!strcmp(joint->ClassName(), "ndMultiBodyVehicleTireJoint"))
			{
				pair.m_m0 = m0;
				pair.m_m1 = 0;

				jacobianPair.m_jacobianM0.m_linear = upDir.Scale(ndFloat32 (-1.0f));
				jacobianPair.m_jacobianM0.m_angular = ndVector::m_zero;
				jacobianPair.m_jacobianM1.m_linear = ndVector::m_zero;
				jacobianPair.m_jacobianM1.m_angular = ndVector::m_zero;

				jointArray.PushBack(pair);
				jacobianArray.PushBack(jacobianPair);
				rhsAccel.PushBack(m_gravityMagnitud);
				tireArray.PushBack((ndMultiBodyVehicleTireJoint*)joint);
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

	ndJacobian zeroJacobian;
	zeroJacobian.m_linear = ndVector::m_zero;
	zeroJacobian.m_angular = ndVector::m_zero;

	massMatrix.SetCount(jointArray.GetCount());
	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		massMatrix[i].SetCount(jointArray.GetCount());
	}

	ndFixSizeArray<ndJacobian, 64> Jt;
	ndFixSizeArray<ndJacobian, 64> JinvMass;
	for (ndInt32 i = 0; i < bodyId.GetCount(); ++i)
	{
		Jt.PushBack(zeroJacobian);
		JinvMass.PushBack(zeroJacobian);
	}

	for (ndInt32 i = 0; i < jointArray.GetCount(); ++i)
	{
		ndInt32 m0 = jointArray[i].m_m0;
		ndInt32 m1 = jointArray[i].m_m1;

		const ndJacobian& J01invMass(jacobianArray[i].m_jacobianM0);
		const ndJacobian& J10invMass(jacobianArray[i].m_jacobianM1);

		JinvMass[m0].m_linear = J01invMass.m_linear.Scale(invMass[m0]);
		JinvMass[m0].m_angular = J01invMass.m_angular.Scale(invMass[m0]);
		JinvMass[m1].m_linear = J10invMass.m_linear.Scale(invMass[m1]);
		JinvMass[m1].m_angular = J10invMass.m_angular.Scale(invMass[m1]);

		ndVector diagDot(
			JinvMass[m0].m_linear * jacobianArray[i].m_jacobianM0.m_linear +
			JinvMass[m0].m_angular * jacobianArray[i].m_jacobianM0.m_angular +
			JinvMass[m1].m_linear * jacobianArray[i].m_jacobianM1.m_linear +
			JinvMass[m1].m_angular * jacobianArray[i].m_jacobianM1.m_angular);
		ndFloat32 diagonal = diagDot.AddHorizontal().GetScalar() * ndFloat32 (1.0005f);
		massMatrix[i][i] = diagonal;

		for (ndInt32 j = i + 1; j < jointArray.GetCount(); ++j)
		{
			ndInt32 n0 = jointArray[j].m_m0;
			ndInt32 n1 = jointArray[j].m_m1;
			Jt[n0] = jacobianArray[j].m_jacobianM0;
			Jt[n1] = jacobianArray[j].m_jacobianM1;

			ndVector sum(ndVector::m_zero);
			for (ndInt32 k = 0; k < bodyId.GetCount(); ++k)
			{
				sum += JinvMass[k].m_linear * Jt[k].m_linear + JinvMass[k].m_angular * Jt[k].m_angular;
			}
			ndFloat32 offDiag = sum.AddHorizontal().GetScalar();
			massMatrix[i][j] = offDiag;
			massMatrix[j][i] = offDiag;

			Jt[n0] = zeroJacobian;
			Jt[n1] = zeroJacobian;
		}

		JinvMass[m0] = zeroJacobian;
		JinvMass[m1] = zeroJacobian;
	}

	ndInt32 stride = ndInt32 (&massMatrix[1][0] - &massMatrix[0][0]);
	ndAssert(ndTestPSDmatrix(jointArray.GetCount(), stride, &massMatrix[0][0]));

	ndFixSizeArray<ndFloat32, 64> force;
	force.SetCount(jointArray.GetCount());
	ndCholeskyFactorization(jointArray.GetCount(), stride, &massMatrix[0][0]);
	ndSolveCholesky(jointArray.GetCount(), stride, &massMatrix[0][0], &force[0], &rhsAccel[0]);

	for (ndInt32 i = 0; i < tireArray.GetCount(); ++i)
	{
		ndMultiBodyVehicleTireJoint* const tire = tireArray[i];
		if (tire)
		{
			tire->m_frictionModel.m_lateralPacejka.m_norminalNormalForce = ndAbs(force[i]);
			tire->m_frictionModel.m_longitudinalPacejka.m_norminalNormalForce = ndAbs(force[i]);
		}
	}

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
	ndFloat32 gravity = m_downForce.GetDownforceFactor(GetSpeed()) * m_gravityMagnitud;
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

	if (!GetRoot()->m_body->GetAsBodyKinematic()->GetSkeleton())
	{
		return;
	}

	// draw vehicle coordinade system;
	const ndBodyKinematic* const chassis = m_chassis;
	ndAssert(chassis);
	const ndMatrix chassisMatrix(chassis->GetMatrix());

	// draw center of mass;
	const ndCenterOfMassDynamics kinematics (CalculateCentreOfMassKinematics(chassisMatrix));
	context.DrawFrame(kinematics.m_centerOfMass);

	// draw vehicle velocity
	const ndVector veloc(kinematics.m_veloc);
	const ndVector p0(kinematics.m_centerOfMass.m_posit + kinematics.m_centerOfMass.m_up.Scale(1.5f));
	const ndVector p1(p0 + kinematics.m_centerOfMass.m_front.Scale(2.0f));
	const ndVector p2(p0 + kinematics.m_centerOfMass.RotateVector(veloc.Scale(0.25f)));

	context.DrawLine(p0, p2, ndVector(1.0f, 1.0f, 0.0f, 0.0f));
	context.DrawLine(p0, p1, ndVector(1.0f, 0.0f, 0.0f, 0.0f));

	// draw tires info
	ndFloat32 scale = ndFloat32(3.0f);
	const ndVector forceColor(ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.0f));
	const ndVector lateralColor(ndFloat32(0.3f), ndFloat32(0.7f), ndFloat32(0.0f), ndFloat32(0.0f));
	const ndVector longitudinalColor(ndFloat32(0.7f), ndFloat32(0.3f), ndFloat32(0.0f), ndFloat32(0.0f));
	//ndVector weight(chassis->GetForce().Scale(scale * chassis->GetInvMass() / m_gravityMagnitud));
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyKinematic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		tireJoint->DebugJoint(context);

		// draw tire forces
		const ndBodyKinematic::ndContactMap& contactMap = tireBody->GetContactMap();
		ndFloat32 tireGravities = scale / (kinematics.m_mass * m_gravityMagnitud);
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
					case ndTireFrictionModel::m_pacejkaSport:
					case ndTireFrictionModel::m_pacejkaTruck:
					case ndTireFrictionModel::m_pacejkaCustom:
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

bool ndMultiBodyVehicle::CoulombTireModel(ndMultiBodyVehicleTireJoint* const joint, ndContactMaterial& contactPoint) const
{
	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;

	// handling dynamics friction manually
	ndFloat32 dynamicFrictionCoef = joint->m_isApplyingBrakes ? ndFloat32(0.75f) : ndFloat32(1.0f);

	contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = frictionCoefficient;
	contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient * dynamicFrictionCoef;
	contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient * dynamicFrictionCoef;
	return true;
}

bool ndMultiBodyVehicle::CoulombFrictionCircleTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	return CoulombTireModel(tire, contactPoint);
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

	const ndVector longitudDir(contactPoint.m_dir0);
	const ndVector contactVeloc0(tireBody->GetVelocity());
	const ndVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
	const ndVector wheelComVeloc(contactVeloc0 - contactVeloc1);
	const ndFloat32 wheelComSpeed_x = wheelComVeloc.DotProduct(longitudDir).GetScalar();
	if (ndAbs(wheelComSpeed_x) < D_MAX_CONTACT_SPEED_TRESHOLD)
	{
		// handle vehicle is at rest by just doing normal rigid body dynamics.
		return true;
	}

	// calculate lateral slip angle
	const ndVector lateralDir(contactPoint.m_dir1);
	// use a dead zone and them use Sv for nonzero lateral force
	const ndFloat32 speed_z = wheelComVeloc.DotProduct(lateralDir).GetScalar();
	const ndFloat32 wheelComSpeed_z = (speed_z > ndFloat32(1.0e-3f) || (speed_z < ndFloat32(-1.0e-3f))) ? speed_z : ndFloat32(0.0f);
	const ndFloat32 sideSlipAngleInRadians = ndAtan2(wheelComSpeed_z, ndAbs(wheelComSpeed_x));
	tire->m_lateralSlip = ndMax(tire->m_lateralSlip, ndAbs(sideSlipAngleInRadians));

	// calculate longitudinal slip
	const ndVector contactVeloc(tireBody->GetVelocityAtPoint(contactPoint.m_point) - contactVeloc1);
	const ndFloat32 vr_x = contactVeloc.DotProduct(longitudDir).GetScalar();
	const ndFloat32 longitudialSlip = ndClamp(vr_x / wheelComSpeed_x, ndFloat32(-100.0f), ndFloat32(100.0f));
	tire->m_longitudinalSlip = ndMax(tire->m_longitudinalSlip, longitudialSlip);

	//I am now using the direct coeficients B, C, E, D as explained in 
	//The multibody system approach to vehicle dynamics page 300 to 306
	auto LongitudinalForce = [](const ndTireFrictionModel::ndPacejkaTireModel& model, ndFloat32 phi, ndFloat32 frictionCoefficient)
	{
		return model.Evaluate(phi, frictionCoefficient);
	};

	auto LateralForce = [](const ndTireFrictionModel::ndPacejkaTireModel& model, ndFloat32 phi, ndFloat32 frictionCoefficient)
	{
		// phi should be in degress
		phi = ndAbs(ndRadToDegree * phi);
		return model.Evaluate(phi, frictionCoefficient);
	};

	//now apply the combine effect, according to Genta book page 76
	const ndTireFrictionModel& frictionModel = tire->m_frictionModel;
	

	// ----- Combined‑force calculation notes -----
	//
	// The derivation below isn’t documented in either Pacejka or Genta books.
	// Both books only give a brief example and leave the details unexplained.
	// After two decades, the method still feels more like bad heuristicks than
	// sound engineering: the equations mix units and fail dimensional checks.
	// It may be possibel the works in some confibe experimetal statady state 
	// laboratory condition, but the are not scalable for symulations. 

	// Pages 80‑83 outline *two* inconsistent ways to calculate combined slip.
	// - Method 1 (implemented here) produces sensible, stable results,
	//   yet the book itself calls it “incorrect.”
	// - Method 2 is recommended by the authors, but I’ve never managed
	//   to get anything remotely realistic from it.

#if 1
	// Until a better reference turns up, Method 1 remains the least‑bad
	// option I’ve found stable in practice, if not theoretically satisfying,  
	// therefre I am going with that. 
	// I still find the lateral force some what too strong.

	//I am assuming sv and sv to be zero.
	//under these conditions u and v become
	ndFloat32 den = ndFloat32(1.0f) + ndAbs(longitudialSlip);
	ndFloat32 phi_x = -longitudialSlip / den;
	ndFloat32 phi_z = ndTan(sideSlipAngleInRadians) / den;
	ndFloat32 phi2 = phi_x * phi_x + phi_z * phi_z;
	if (phi2 < ndFloat32 (1.0e-6f))
	{
		// this is the vanishing phi
		return true;
	}
	ndFloat32 phi = ndSqrt(phi2);

	const ndFloat32 frictionCoefficient_x = contactPoint.m_material.m_staticFriction0;
	const ndFloat32 frictionCoefficient_z = contactPoint.m_material.m_staticFriction1;
	ndFloat32 pure_fz = LateralForce(frictionModel.m_lateralPacejka, sideSlipAngleInRadians, frictionCoefficient_z);
	ndFloat32 pure_fx = LongitudinalForce(frictionModel.m_longitudinalPacejka, longitudialSlip, frictionCoefficient_x);

	ndFloat32 fx = pure_fx * phi_x / phi;
	ndFloat32 fz = pure_fz * phi_z / phi;
#else

	// This is the second method that the slips and the use a 
	// normalized dimnetion less slip for both lateral and longitudinal
	// with not explanation as to how teh Pacekka equation can be call 
	// with these dimnation lesst values.
	// to me, this seem like nonsence, unless lots of details that are omitted in the book.
	const ndFloat32 dimensionLessPhi_x = longitudialSlip / frictionModel.m_longitudinalPacejka.m_normalizingPhi;
	const ndFloat32 dimensionLessPhi_z = sideSlipAngleInRadians / (frictionModel.m_lateralPacejka.m_normalizingPhi * ndDegreeToRad * 0.5f);

	// since Sv and Sh are zero of very small, them delta alpha is zero. 
	const ndFloat32 dimensionLessCombined_phi = ndSqrt(dimensionLessPhi_x * dimensionLessPhi_x + dimensionLessPhi_z * dimensionLessPhi_z);

	const ndFloat32 modifiedPhi_x = longitudialSlip * dimensionLessCombined_phi;
	const ndFloat32 modifiedPhiInRadians_z = sideSlipAngleInRadians * dimensionLessCombined_phi;

	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;
	ndFloat32 pureFz = LateralForce(frictionModel.m_lateralPacejka, modifiedPhiInRadians_z, frictionCoefficient);
	ndFloat32 pureFx = LongitudinalForce(frictionModel.m_longitudinalPacejka, modifiedPhi_x, frictionCoefficient);

	// after we calculate the compensated longitudinal and lateral forces,
	// they have to be scaled back
	ndFloat32 fx = pureFx * dimensionLessPhi_x / dimensionLessCombined_phi;
	ndFloat32 fz = pureFz * dimensionLessPhi_z / dimensionLessCombined_phi;

#endif
	
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
