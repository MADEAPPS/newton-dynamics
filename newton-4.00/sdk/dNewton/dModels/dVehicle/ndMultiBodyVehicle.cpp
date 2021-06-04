/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndJointHinge.h"
#include "ndJointWheel.h"
#include "ndBodyDynamic.h"
#include "ndJointDoubleHinge.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

ndMultiBodyVehicle::ndMultiBodyVehicle(const dVector& frontDir, const dVector& upDir)
	:ndModel()
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
	,m_motor(nullptr)
	,m_tireShape(new ndShapeChamferCylinder(dFloat32(0.75f), dFloat32(0.5f)))
	,m_gearBox(nullptr)
	,m_torsionBar(nullptr)
	,m_tireList()
	,m_brakeTires()
	,m_handBrakeTires()
	,m_steeringTires()
	,m_differentials()
	,m_brakeTorque(dFloat32(0.0f))
	,m_steeringAngle(dFloat32 (0.0f))
	,m_handBrakeTorque(dFloat32(0.0f))
	,m_steeringAngleMemory(dFloat32(0.0f))
{
	m_tireShape->AddRef();
	m_localFrame.m_front = frontDir & dVector::m_triplexMask;
	m_localFrame.m_up = upDir & dVector::m_triplexMask;
	m_localFrame.m_right = m_localFrame.m_front.CrossProduct(m_localFrame.m_up).Normalize();
	m_localFrame.m_up = m_localFrame.m_right.CrossProduct(m_localFrame.m_front).Normalize();
}

ndMultiBodyVehicle::ndMultiBodyVehicle(const nd::TiXmlNode* const xmlNode)
	:ndModel(xmlNode)
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
	,m_tireShape(new ndShapeChamferCylinder(dFloat32(0.75f), dFloat32(0.5f)))
	,m_tireList()
	,m_handBrakeTires()
	,m_steeringTires()
	,m_gravityMag(dFloat32(0.0f))
	,m_brakeTorque(dFloat32(0.0f))
	,m_steeringAngle(dFloat32(0.0f))
	,m_handBrakeTorque(dFloat32(0.0f))
	,m_steeringAngleMemory(dFloat32(0.0f))
{
	m_tireShape->AddRef();
}

ndMultiBodyVehicle::~ndMultiBodyVehicle()
{
	m_tireShape->Release();
}

dFloat32 ndMultiBodyVehicle::GetSpeed() const
{
	const dVector dir(m_chassis->GetMatrix().RotateVector(m_localFrame.m_front));
	const dFloat32 speed = m_chassis->GetVelocity().DotProduct(dir).GetScalar();
	return speed;
}

void ndMultiBodyVehicle::AddChassis(ndBodyDynamic* const chassis, dFloat32 gravityMag)
{
	m_chassis = chassis;
	m_gravityMag = dAbs(gravityMag);
}

void ndMultiBodyVehicle::SetBrakeTorque(dFloat32 brakeToqrue)
{
	m_brakeTorque = dAbs(brakeToqrue);
}

void ndMultiBodyVehicle::SetHandBrakeTorque(dFloat32 brakeToqrue)
{
	m_handBrakeTorque = dAbs(brakeToqrue);
}

void ndMultiBodyVehicle::SetSteeringAngle(dFloat32 angleInRadians)
{
	m_steeringAngle = angleInRadians;
}

ndJointWheel* ndMultiBodyVehicle::AddAxleTire(ndWorld* const world, const ndWheelDescriptor& desc, ndBodyDynamic* const tire, ndBodyDynamic* const axleBody)
{
	dMatrix tireFrame(dGetIdentityMatrix());
	tireFrame.m_front = dVector(0.0f, 0.0f, 1.0f, 0.0f);
	tireFrame.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = dVector(-1.0f, 0.0f, 0.0f, 0.0f);
	dMatrix matrix(tireFrame * m_localFrame * axleBody->GetMatrix());
	matrix.m_posit = tire->GetMatrix().m_posit;

	// make tire inertia spherical
	dVector inertia(tire->GetMassMatrix());
	dFloat32 maxInertia(dMax(dMax(inertia.m_x, inertia.m_y), inertia.m_z));
	inertia.m_x = maxInertia;
	inertia.m_y = maxInertia;
	inertia.m_z = maxInertia;
	tire->SetMassMatrix(inertia);

	ndJointWheel* const tireJoint = new ndJointWheel(matrix, tire, axleBody, desc);
	m_tireList.Append(tireJoint);
	world->AddJoint(tireJoint);

	tire->SetDebugMaxAngularIntegrationSteepAndLinearSpeed(dFloat32(2.0f * 360.0f) * dDegreeToRad, dFloat32(100.0f));
	return tireJoint;
}

ndJointWheel* ndMultiBodyVehicle::AddTire(ndWorld* const world, const ndWheelDescriptor& desc, ndBodyDynamic* const tire)
{
	return AddAxleTire(world, desc, tire, m_chassis);
}

ndBodyDynamic* ndMultiBodyVehicle::CreateInternalBodyPart(ndWorld* const world, dFloat32 mass, dFloat32 radius) const
{
	ndShapeInstance diffCollision(new ndShapeSphere(radius));
	diffCollision.SetCollisionMode(false);

	dAssert(m_chassis);
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetMatrix(m_localFrame * m_chassis->GetMatrix());
	body->SetCollisionShape(diffCollision);
	body->SetMassMatrix(mass, diffCollision);
	world->AddBody(body);

	body->SetDebugMaxAngularIntegrationSteepAndLinearSpeed(dFloat32(2.0f * 360.0f) * dDegreeToRad, dFloat32(100.0f));
	return body;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndWorld* const world, dFloat32 mass, dFloat32 radius, ndJointWheel* const leftTire, ndJointWheel* const rightTire)
{
	ndBodyDynamic* const differentialBody = CreateInternalBodyPart(world, mass, radius);

	ndMultiBodyVehicleDifferential* const differential = new ndMultiBodyVehicleDifferential(differentialBody, m_chassis);
	world->AddJoint(differential);
	m_differentials.Append(differential);

	dVector pin0(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_front));
	dVector upPin(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_up));
	dVector leftPin1(leftTire->GetBody0()->GetMatrix().RotateVector(leftTire->GetLocalMatrix0().m_front));

	ndMultiBodyVehicleDifferentialAxle* const leftAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin, differentialBody, leftPin1, leftTire->GetBody0());
	world->AddJoint(leftAxle);

	ndMultiBodyVehicleDifferentialAxle* const rightAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin.Scale (dFloat32 (-1.0f)), differentialBody, leftPin1, rightTire->GetBody0());
	world->AddJoint(rightAxle);

	return differential;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndWorld* const world, dFloat32 mass, dFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential)
{
	ndBodyDynamic* const differentialBody = CreateInternalBodyPart(world, mass, radius);

	ndMultiBodyVehicleDifferential* const differential = new ndMultiBodyVehicleDifferential(differentialBody, m_chassis);
	world->AddJoint(differential);
	m_differentials.Append(differential);

	dVector pin0(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_front));
	dVector upPin(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_up));
	dVector leftPin1(leftDifferential->GetBody0()->GetMatrix().RotateVector(leftDifferential->GetLocalMatrix0().m_front));
	leftPin1 = leftPin1.Scale(dFloat32(-1.0f));

	ndMultiBodyVehicleDifferentialAxle* const leftAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin, differentialBody, leftPin1, leftDifferential->GetBody0());
	world->AddJoint(leftAxle);

	ndMultiBodyVehicleDifferentialAxle* const rightAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin.Scale(dFloat32(-1.0f)), differentialBody, leftPin1, rightDifferential->GetBody0());
	world->AddJoint(rightAxle);

	return differential;
}

ndMultiBodyVehicleMotor* ndMultiBodyVehicle::AddMotor(ndWorld* const world, dFloat32 mass, dFloat32 radius)
{
	//dAssert(0);
	ndBodyDynamic* const motorBody = CreateInternalBodyPart(world, mass, radius);

	m_motor = new ndMultiBodyVehicleMotor(motorBody, this);
	world->AddJoint(m_motor);

	return m_motor;
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::AddGearBox(ndWorld* const world, ndMultiBodyVehicleMotor* const motor, ndMultiBodyVehicleDifferential* const differential)
{
	dAssert(m_motor == motor);
	m_gearBox = new ndMultiBodyVehicleGearBox(motor->GetBody0(), differential->GetBody0(), this);
	world->AddJoint(m_gearBox);
	return m_gearBox;
}

ndMultiBodyVehicleTorsionBar* ndMultiBodyVehicle::AddTorsionBar(ndWorld* const world)
{
	m_torsionBar = new ndMultiBodyVehicleTorsionBar(this, world->GetSentinelBody());
	world->AddJoint(m_torsionBar);
	return m_torsionBar;
}

ndShapeInstance ndMultiBodyVehicle::CreateTireShape(dFloat32 radius, dFloat32 width) const
{
	ndShapeInstance tireCollision(m_tireShape);
	dVector scale(2.0f * width, radius, radius, 0.0f);
	tireCollision.SetScale(scale);
	return tireCollision;
}

void ndMultiBodyVehicle::SetAsSteering(ndJointWheel* const tire)
{
	m_steeringTires.Append(tire);
}

void ndMultiBodyVehicle::SetAsBrake(ndJointWheel* const tire)
{
	m_brakeTires.Append(tire);
}

void ndMultiBodyVehicle::SetAsHandBrake(ndJointWheel* const tire)
{
	m_handBrakeTires.Append(tire);
}


void ndMultiBodyVehicle::ApplyAligmentAndBalancing()
{
	const dVector chassisOmega(m_chassis->GetOmega());
	const dVector upDir(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
	for (dList<ndJointWheel*>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndJointWheel* const tire = node->GetInfo();

		ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
		dAssert(tireBody != m_chassis);
		if (!tireBody->GetSleepState())
		{
			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			tire->CalculateGlobalMatrix(tireMatrix, chassisMatrix);

			// align tire matrix 
			const dVector relPosit(tireMatrix.m_posit - chassisMatrix.m_posit);
			const dFloat32 distance = relPosit.DotProduct(upDir).GetScalar();
			const dFloat32 spinAngle = -tire->CalculateAngle(tireMatrix.m_up, chassisMatrix.m_up, chassisMatrix.m_front);

			dMatrix newTireMatrix(dPitchMatrix(spinAngle) * chassisMatrix);
			newTireMatrix.m_posit = chassisMatrix.m_posit + upDir.Scale(distance);

			dMatrix tireBodyMatrix(tire->GetLocalMatrix0().Inverse() * newTireMatrix);
			tireBody->SetMatrix(tireBodyMatrix);

			// align tire velocity
			const dVector chassiVelocity(m_chassis->GetVelocityAtPoint(tireBodyMatrix.m_posit));
			const dVector relVeloc(tireBody->GetVelocity() - chassiVelocity);
			const dFloat32 speed = relVeloc.DotProduct(upDir).GetScalar();
			const dVector tireVelocity(chassiVelocity + upDir.Scale(speed));
			tireBody->SetVelocity(tireVelocity);

			// align tire angular velocity
			const dVector relOmega(tireBody->GetOmega() - chassisOmega);
			const dFloat32 rpm = relOmega.DotProduct(chassisMatrix.m_front).GetScalar();
			const dVector tireOmega(chassisOmega + chassisMatrix.m_front.Scale(rpm));
			tireBody->SetOmega(tireOmega);
		}
	}

	for (dList<ndMultiBodyVehicleDifferential*>::dListNode* node = m_differentials.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferential* const diff = node->GetInfo();
		diff->AlignMatrix();
	}

	if (m_motor)
	{
		m_motor->AlignMatrix();
	}
}

void ndMultiBodyVehicle::ApplySteering()
{
	if (dAbs(m_steeringAngleMemory - m_steeringAngle) > dFloat32(1.0e-3f))
	{
		m_steeringAngleMemory = m_steeringAngle;
		for (dList<ndJointWheel*>::dListNode* node = m_steeringTires.GetFirst(); node; node = node->GetNext())
		{
			ndJointWheel* const tire = node->GetInfo();
			tire->SetSteeringAngle(m_steeringAngle);
		}
	}
}

void ndMultiBodyVehicle::Debug(ndConstraintDebugCallback& context) const
{
	// draw vehicle cordinade system;
	dMatrix chassisMatrix(m_chassis->GetMatrix());
	chassisMatrix.m_posit = chassisMatrix.TransformVector(m_chassis->GetCentreOfMass());
	//context.DrawFrame(chassisMatrix);

	dFloat32 totalMass = m_chassis->GetMassMatrix().m_w;
	dVector effectiveCom(chassisMatrix.m_posit.Scale(totalMass));

	// draw front direction for side slip angle reference
	dVector p0(chassisMatrix.m_posit + m_localFrame.m_up.Scale(1.0f));
	dVector p1(p0 + chassisMatrix.RotateVector(m_localFrame.m_front).Scale(0.5f));
	context.DrawLine(p0, p1, dVector(1.0f, 1.0f, 1.0f, 0.0f));

	// draw velocity vector
	dVector veloc(m_chassis->GetVelocity());
	dVector p2(p0 + veloc.Scale (0.25f));
	context.DrawLine(p0, p2, dVector(1.0f, 1.0f, 0.0f, 0.0f));

	// draw body acceleration
	//dVector accel(m_chassis->GetAccel());
	//dVector p3(p0 + accel.Scale(0.5f));
	//context.DrawLine(p0, p3, dVector(0.0f, 1.0f, 1.0f, 0.0f));

	dVector weight(m_chassis->GetForce());
	dFloat32 scale = dSqrt(weight.DotProduct(weight).GetScalar());
	weight = weight.Normalize().Scale(-2.0f);

	// draw vehicle weight;
	dVector forceColor(dFloat32 (0.8f), dFloat32(0.8f), dFloat32(0.8f), dFloat32(0.0f));
	dVector lateralColor(dFloat32(0.3f), dFloat32(0.7f), dFloat32(0.0f), dFloat32(0.0f));
	dVector longitudinalColor(dFloat32(0.7f), dFloat32(0.3f), dFloat32(0.0f), dFloat32(0.0f));
	context.DrawLine(chassisMatrix.m_posit, chassisMatrix.m_posit + weight, forceColor);

	for (dList<ndJointWheel*>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndJointWheel* const tireJoint = node->GetInfo();
		ndBodyDynamic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		// show tire center of mass;
		dMatrix tireFrame(tireBody->GetMatrix());
		dVector com(tireBody->GetCentreOfMass());
		tireFrame.m_posit = tireFrame.TransformVector(tireBody->GetCentreOfMass());
		context.DrawFrame(tireFrame);

		totalMass += tireBody->GetMassMatrix().m_w;
		effectiveCom += tireFrame.m_posit.Scale(tireBody->GetMassMatrix().m_w);

		//tire->DebugJoint(context);
		//dVector color(1.0f, 1.0f, 1.0f, 0.0f);
		//context.DrawArrow(tireBody->GetMatrix(), color, -1.0f);

		// draw tire forces
		const ndBodyKinematic::ndContactMap& contactMap = tireBody->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::dListNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					dMatrix frame(contactPoint.m_normal, contactPoint.m_dir0, contactPoint.m_dir1, contactPoint.m_point);

					dVector localPosit(m_localFrame.UntransformVector(chassisMatrix.UntransformVector(contactPoint.m_point)));
					dFloat32 offset = (localPosit.m_z > dFloat32(0.0f)) ? dFloat32(0.2f) : dFloat32(-0.2f);
					frame.m_posit += contactPoint.m_dir0.Scale(offset);
					frame.m_posit += contactPoint.m_normal.Scale(0.1f);
					//context.DrawFrame(frame);

					// normal force
					dFloat32 normalForce = dFloat32 (2.0f) * contactPoint.m_normal_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_normal.Scale (normalForce), forceColor);

					// lateral force
					dFloat32 lateralForce = -dFloat32(2.0f) * contactPoint.m_dir0_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir0.Scale(lateralForce), lateralColor);

					// longitudinal force
					dFloat32 longitudinalForce = -dFloat32(2.0f) * contactPoint.m_dir1_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir1.Scale(longitudinalForce), longitudinalColor);
				}
			}
		}
	}

	effectiveCom = effectiveCom.Scale(dFloat32(1.0f) / totalMass);
	chassisMatrix.m_posit = effectiveCom;
	chassisMatrix.m_posit.m_w = dFloat32(1.0f);
	context.DrawFrame(chassisMatrix);
}

void ndMultiBodyVehicle::ApplyBrakes()
{
	for (dList<ndJointWheel*>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndJointWheel* const tire = node->GetInfo();
		tire->SetBrakeTorque(dFloat32 (0.0f));
	}

	for (dList<ndJointWheel*>::dListNode* node = m_brakeTires.GetFirst(); node; node = node->GetNext())
	{
		ndJointWheel* const tire = node->GetInfo();
		tire->SetBrakeTorque(m_brakeTorque);
	}

	if (m_brakeTorque == dFloat32(0.0f))
	{
		for (dList<ndJointWheel*>::dListNode* node = m_handBrakeTires.GetFirst(); node; node = node->GetNext())
		{
			ndJointWheel* const tire = node->GetInfo();

			tire->SetBrakeTorque(m_handBrakeTorque);
		}
	}
}

void ndMultiBodyVehicle::BrushTireModel(const ndJointWheel* const tire, ndContactMaterial& contactPoint) const
{
	// calculate longitudinal slip ratio
	const ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
	const ndBodyDynamic* const otherBody = (contactPoint.m_body0 == tireBody) ? ((ndBodyKinematic*)contactPoint.m_body1)->GetAsBodyDynamic() : ((ndBodyKinematic*)contactPoint.m_body0)->GetAsBodyDynamic();
	dAssert(tireBody != otherBody);
	dAssert((tireBody == contactPoint.m_body0) || (tireBody == contactPoint.m_body1));

	const dVector tireVeloc(tireBody->GetVelocity());
	const dVector contactVeloc0(tireBody->GetVelocityAtPoint(contactPoint.m_point));
	const dVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
	const dVector relVeloc(contactVeloc0 - contactVeloc1);

	const dFloat32 relSpeed = dAbs (relVeloc.DotProduct(contactPoint.m_dir1).GetScalar());
	const dFloat32 tireSpeed = dMax (dAbs (tireVeloc.DotProduct(contactPoint.m_dir1).GetScalar()), dFloat32 (0.1f));
	const dFloat32 longitudialSlip = relSpeed / tireSpeed;

	// calculate side slip ratio
	const dFloat32 sideSpeed = dAbs(relVeloc.DotProduct(contactPoint.m_dir0).GetScalar());
	const dFloat32 lateralSleep = sideSpeed / (relSpeed + dFloat32 (0.01f));

	const dFloat32 den = dFloat32(1.0f) / (dFloat32(1.0f) + longitudialSlip);
	const dFloat32 v = lateralSleep * den;
	const dFloat32 u = longitudialSlip * den;

	const ndWheelDescriptor& info = tire->GetInfo();
	const dFloat32 cz = info.m_laterialStiffeness * v;
	const dFloat32 cx = info.m_longitudinalStiffeness * u;
	const dFloat32 gamma = dSqrt(cx * cx + cz * cz) + dFloat32 (1.0e-3f);

	const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
	const dFloat32 lateralFrictionCoefficient = frictionCoefficient * cz / gamma;
	const dFloat32 longitudinalFrictionCoefficient = frictionCoefficient * cx / gamma;
	//dTrace(("%f %f\n", sideSpeed, lateralFrictionCoefficient));

	//contactPoint.m_material.m_flags |= m_isSoftContact;
	//contactPoint.m_material.m_skinThickness = dFloat32 (0.25f);

	contactPoint.m_material.m_restitution = dFloat32 (0.1f);
	contactPoint.m_material.m_staticFriction0 = lateralFrictionCoefficient;
	contactPoint.m_material.m_dynamicFriction0 = lateralFrictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = longitudinalFrictionCoefficient;
	contactPoint.m_material.m_dynamicFriction1 = longitudinalFrictionCoefficient;
}

void ndMultiBodyVehicle::ApplyTiremodel()
{
	for (dList<ndJointWheel*>::dListNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndJointWheel* const tire = node->GetInfo();

		const dMatrix tireMatrix (tire->GetLocalMatrix1() * tire->GetBody1()->GetMatrix());
		const ndBodyKinematic::ndContactMap& contactMap = tire->GetBody0()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::dListNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					ndContactMaterial& contactPoint = contactNode->GetInfo();
					const dVector fronDir(contactPoint.m_normal.CrossProduct(tireMatrix.m_front));
					if (fronDir.DotProduct(fronDir).GetScalar() > dFloat32(1.0e-3f))
					{
						contactPoint.m_dir1 = fronDir.Normalize();
						contactPoint.m_dir0 = contactPoint.m_dir1.CrossProduct(contactPoint.m_normal);
						BrushTireModel(tire, contactPoint);
					}
				}
			}
		}
	}
}

ndMultiBodyVehicle::ndDownForce::ndDownForce()
{
	m_downForceTable[0].m_speed = dFloat32(5.0f) * dFloat32(0.27f);
	m_downForceTable[0].m_forceFactor = 0.5f;
	m_downForceTable[0].m_aerodynamicsConst = dFloat32(0.0f);

	m_downForceTable[1].m_speed = dFloat32(60.0f) * dFloat32(0.27f);
	m_downForceTable[1].m_forceFactor = 1.0f;
	dFloat32 num = m_downForceTable[1].m_forceFactor - m_downForceTable[0].m_forceFactor;
	dFloat32 den = m_downForceTable[1].m_speed - m_downForceTable[0].m_speed;
	m_downForceTable[1].m_aerodynamicsConst = num / (den * den);

	m_downForceTable[2].m_speed = dFloat32(200.0f) * dFloat32(0.27f);
	m_downForceTable[2].m_forceFactor = 3.0f;
	num = m_downForceTable[2].m_forceFactor - m_downForceTable[1].m_forceFactor;
	den = m_downForceTable[2].m_speed - m_downForceTable[1].m_speed;
	m_downForceTable[2].m_aerodynamicsConst = num / (den * den);
}

dFloat32 ndMultiBodyVehicle::ndDownForce::GetDownforceFactor(dFloat32 speed) const
{
	speed = dAbs(speed);
	if (speed < m_downForceTable[0].m_speed)
	{
		return m_downForceTable[0].m_forceFactor;
	}
	else if (speed < m_downForceTable[1].m_speed)
	{
		dFloat32 deltaSpeed = speed - m_downForceTable[0].m_forceFactor;
		return m_downForceTable[0].m_forceFactor + m_downForceTable[1].m_aerodynamicsConst * deltaSpeed * deltaSpeed;
	}
	else if (speed < m_downForceTable[2].m_speed)
	{
		dFloat32 deltaSpeed = speed - m_downForceTable[1].m_forceFactor;
		return m_downForceTable[1].m_forceFactor + m_downForceTable[2].m_aerodynamicsConst * deltaSpeed * deltaSpeed;
	}
	return m_downForceTable[2].m_speed;
}

void ndMultiBodyVehicle::Update(ndWorld* const, dFloat32)
{
	// apply down force
	dFloat32 downForceFactor = m_downForce.GetDownforceFactor(GetSpeed());
	if (downForceFactor > dFloat32(0.0f))
	{
		const dVector up(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
		dVector downForce(up.Scale(-m_gravityMag * downForceFactor * m_chassis->GetMassMatrix().m_w));
		m_chassis->SetForce(m_chassis->GetForce() + downForce);
	}

	ApplyAligmentAndBalancing();
	ApplyBrakes();
	ApplySteering();
	ApplyTiremodel();
}
