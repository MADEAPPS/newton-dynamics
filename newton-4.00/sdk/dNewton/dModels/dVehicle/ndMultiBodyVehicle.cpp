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
* 
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

#define D_MAX_CONTACT_SPEED_TRESHOLD	ndFloat32 (1.0f)
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
	m_downForceTable[1].m_forceFactor = 1.0f;
	m_downForceTable[1].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[0]);

	m_downForceTable[2].m_speed = ndFloat32(60.0f) * ndFloat32(0.27f);
	m_downForceTable[2].m_forceFactor = 1.6f;
	m_downForceTable[2].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[1]);

	m_downForceTable[3].m_speed = ndFloat32(140.0f) * ndFloat32(0.27f);
	m_downForceTable[3].m_forceFactor = 3.0f;
	m_downForceTable[3].m_aerodynamicDownforceConstant = CalculateFactor(&m_downForceTable[2]);

	m_downForceTable[4].m_speed = ndFloat32(1000.0f) * ndFloat32(0.27f);
	m_downForceTable[4].m_forceFactor = 3.0f;
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
	//,m_dynamicSolver()
	,m_downForce()
{
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
	ndSharedPtr<ndBody> motorBody (CreateInternalBodyPart(mass, radius));
	ndSharedPtr<ndJointBilateralConstraint> motorJoint(new ndMultiBodyVehicleMotor(motorBody->GetAsBodyKinematic(), this));
	AddMotor(motorBody, motorJoint);
	return m_motor;
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::AddGearBox(ndMultiBodyVehicleDifferential* const differential)
{
	ndAssert(m_motor);
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

void ndMultiBodyVehicle::CalculateNormalizedAlgniningTorque(ndMultiBodyVehicleTireJoint* const, ndFloat32 sideSlipTangent) const
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
}

void ndMultiBodyVehicle::ApplyTireModel(ndFloat32 timestep)
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

	ApplyTireModel(timestep, tireContacts);

	// save the steering
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		tire->m_normalidedSteering0 = tire->m_normalidedSteering;
	}
}

void ndMultiBodyVehicle::CoulombTireModel(ndMultiBodyVehicleTireJoint* const joint, ndContactMaterial& contactPoint, ndFloat32) const
{
	const ndFloat32 frictionCoefficient = contactPoint.m_material.m_staticFriction0;

	// handling dynamics friction manually
	ndFloat32 dynamicFrictionCoef = joint->m_IsAapplyingBreaks ? ndFloat32(0.75f) : ndFloat32(1.0f);

	contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = frictionCoefficient;
	contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient * dynamicFrictionCoef;
	contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient * dynamicFrictionCoef;
}

void ndMultiBodyVehicle::PacejkaTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const
{
	BrushTireModel(tire, contactPoint, timestep);
}

void ndMultiBodyVehicle::CoulombFrictionCircleTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const
{
	BrushTireModel(tire, contactPoint, timestep);
}

void ndMultiBodyVehicle::BrushTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const
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

	if (ndAbs(relSpeed) > D_MAX_CONTACT_SPEED_TRESHOLD)
	{
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
		const ndFloat32 vehicleMass = chassis->GetMassMatrix().m_w;
		const ndFloat32 cz = ndAbs(vehicleMass * info.m_laterialStiffness * v);
		const ndFloat32 cx = ndAbs(vehicleMass * info.m_longitudinalStiffness * u);

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

		ndFloat32 dynamicFrictionCoef = tire->m_IsAapplyingBreaks ? ndFloat32(0.5f) : ndFloat32(1.0f);
		contactPoint.m_material.m_staticFriction1 = lateralForce * dynamicFrictionCoef;
		contactPoint.m_material.m_staticFriction0 = longitudinalForce * dynamicFrictionCoef;
		contactPoint.m_material.m_dynamicFriction1 = lateralForce * dynamicFrictionCoef;
		contactPoint.m_material.m_dynamicFriction0 = longitudinalForce * dynamicFrictionCoef;
		ndUnsigned32 newFlags = contactPoint.m_material.m_flags | m_override0Friction | m_override1Friction;
		contactPoint.m_material.m_flags = newFlags;
	}
	else
	{
		CoulombTireModel(tire, contactPoint, timestep);
	}
}

void ndMultiBodyVehicle::ApplyTireModel(ndFloat32 timestep, ndFixSizeArray<ndTireContactPair, 128>& tireContacts)
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

	if (tireContacts.GetCount() == savedContactCount)
	{
		for (ndInt32 i = tireContacts.GetCount() - 1; i >= 0; --i)
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
						BrushTireModel(tire, contactPoint, timestep);
						break;
					}

					case ndTireFrictionModel::m_pacejka:
					{
						PacejkaTireModel(tire, contactPoint, timestep);
						break;
					}

					case ndTireFrictionModel::m_coulombCicleOfFriction:
					{
						CoulombFrictionCircleTireModel(tire, contactPoint, timestep);
						break;
					}

					case ndTireFrictionModel::m_coulomb:
					default:
					{
						CoulombTireModel(tire, contactPoint, timestep);
						break;
					}
				}
			}
		}
		ApplyStabilityControl();
	}
}

void ndMultiBodyVehicle::PostUpdate(ndFloat32)
{
	ApplyAlignmentAndBalancing();
}

void ndMultiBodyVehicle::Update(ndFloat32 timestep)
{
	//ndAssert(!IsSleeping());
	// apply down force
	ApplyAerodynamics(timestep);
	// apply tire model
	ApplyTireModel(timestep);
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

//void ndMultiBodyVehicle::SetVehicleSolverModel(bool hardJoint)
//{
//	hardJoint = true;
//	//hardJoint = false;
//	if (GetRoot())
//	{
//		ndJointBilateralSolverModel useSoftSolver = hardJoint ? m_jointkinematicOpenLoop : m_jointIterativeSoft;
//
//		for (ndNode* node = GetRoot()->GetFirstChild(); node; node = node->GetNext())
//		{
//			ndJointBilateralConstraint* const joint = *node->m_joint;
//			const char* const className = joint->ClassName();
//			if (!strcmp(className, ndMultiBodyVehicleMotor::StaticClassName()) ||
//				!strcmp(className, ndMultiBodyVehicleTireJoint::StaticClassName()) ||
//				!strcmp(className, ndMultiBodyVehicleDifferential::StaticClassName()))
//			{
//				joint->SetSolverModel(useSoftSolver);
//			}
//		}
//
//		ndJointBilateralSolverModel driveTrainMode = hardJoint ? m_jointkinematicCloseLoop : m_jointIterativeSoft;
//		for (ndList<ndNode>::ndNode* node = m_closeLoops.GetFirst(); node; node = node->GetNext())
//		{
//			ndModelArticulation::ndNode& closeLoop = node->GetInfo();
//			ndJointBilateralConstraint* const joint = *closeLoop.m_joint;
//			const char* const clasName = joint->ClassName();
//			if (strcmp(clasName, ndMultiBodyVehicleDifferential::StaticClassName()) || strcmp(clasName, ndMultiBodyVehicleGearBox::StaticClassName()))
//			{
//				joint->SetSolverModel(driveTrainMode);
//			}
//		}
//
//		if (m_torsionBar)
//		{
//			ndAssert(0);
//			//m_torsionBar->SetSolverModel(driveTrainMode);
//		}
//	}
//}

void ndMultiBodyVehicle::ApplyStabilityControl()
{
	ndAssert(m_chassis);
	const ndBodyKinematic* const chassis = m_chassis;
	const ndVector veloc(chassis->GetVelocity());
	const ndMatrix chassisMatrix(chassis->GetMatrix());

	static int xxxxx;

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
					//ndFloat32 steering = tire->m_normalidedSteering0 + (targetSteering - tire->m_normalidedSteering0) * m_steeringRate;
					ndFloat32 steering = tire->m_normalidedSteering0 + (targetSteering - tire->m_normalidedSteering0) * 0.002;
					tire->m_normalidedSteering = steering;
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
					ndFloat32 steering = tire->m_normalidedSteering0 + (tire->m_normalidedSteering - tire->m_normalidedSteering0) * m_steeringRate;
					tire->m_normalidedSteering = steering;
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

			ndTrace(("%d: betaRate %f = %f + %f\n", xxxxx, betaRate, localAccel.m_z / localVeloc.m_x, localOmega.m_y));
			if (ndAbs(betaRate) > m_maxSideslipRate)
			{
				ndFloat32 targetSteering = (betaRate > m_maxSideslipRate) ? ndFloat32(1.0f) : ndFloat32(-1.0f);
				//ndTrace(("a=%f b=%f b'=%f fz=%f w=%f steer=(", localAccel.m_z, sideslip * ndRadToDegree, betaRate, sideslipRate, localOmega.m_y));
				for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = GetTireList().GetFirst(); node; node = node->GetNext())
				{
					ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
					if (tire->m_info.m_steeringAngle != 0)
					{
						ndFloat32 steering = tire->m_normalidedSteering0 + (targetSteering - tire->m_normalidedSteering0) * m_steeringRate * 0.5f;
						tire->m_normalidedSteering = steering;
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
						ndFloat32 steering = tire->m_normalidedSteering0 + (tire->m_normalidedSteering - tire->m_normalidedSteering0) * m_steeringRate;
						tire->m_normalidedSteering = steering;
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

	xxxxx++;
}