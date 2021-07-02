/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "ndJointDoubleHinge.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTireJoint.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

#define D_MAX_CONTACT_PENETRATION	  dFloat32 (1.0e-2f)
#define D_MIN_CONTACT_CLOSE_DISTANCE2 dFloat32 (5.0e-2f * 5.0e-2f)

ndMultiBodyVehicle::ndMultiBodyVehicle(const dVector& frontDir, const dVector& upDir)
	:ndModel()
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
	,m_motor(nullptr)
	,m_tireShape(new ndShapeChamferCylinder(dFloat32(0.75f), dFloat32(0.5f)))
	,m_gearBox(nullptr)
	,m_torsionBar(nullptr)
	,m_tireList()
	,m_differentials()
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
	const dFloat32 speed = dAbs(m_chassis->GetVelocity().DotProduct(dir).GetScalar());
	return speed;
}

void ndMultiBodyVehicle::AddChassis(ndBodyDynamic* const chassis)
{
	m_chassis = chassis;
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddAxleTire(ndWorld* const world, const ndWheelDescriptor& desc, ndBodyDynamic* const tire, ndBodyDynamic* const axleBody)
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

	ndMultiBodyVehicleTireJoint* const tireJoint = new ndMultiBodyVehicleTireJoint(matrix, tire, axleBody, desc, this);
	m_tireList.Append(tireJoint);
	world->AddJoint(tireJoint);

	tire->SetDebugMaxAngularIntegrationSteepAndLinearSpeed(dFloat32(2.0f * 360.0f) * dDegreeToRad, dFloat32(100.0f));
	return tireJoint;
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddTire(ndWorld* const world, const ndWheelDescriptor& desc, ndBodyDynamic* const tire)
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

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndWorld* const world, dFloat32 mass, dFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, dFloat32 slipOmegaLock)
{
	ndBodyDynamic* const differentialBody = CreateInternalBodyPart(world, mass, radius);

	ndMultiBodyVehicleDifferential* const differential = new ndMultiBodyVehicleDifferential(differentialBody, m_chassis, slipOmegaLock);
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

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndWorld* const world, dFloat32 mass, dFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, dFloat32 slipOmegaLock)
{
	ndBodyDynamic* const differentialBody = CreateInternalBodyPart(world, mass, radius);

	ndMultiBodyVehicleDifferential* const differential = new ndMultiBodyVehicleDifferential(differentialBody, m_chassis, slipOmegaLock);
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

void ndMultiBodyVehicle::ApplyAerodynamics()
{
	dFloat32 gravity = m_downForce.GetDownforceFactor(GetSpeed());
	if (dAbs (gravity) > dFloat32(1.0e-2f))
	{
		const dVector up(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
		const dVector weight(m_chassis->GetForce());
		const dVector downForce(up.Scale(gravity * m_chassis->GetMassMatrix().m_w));
		m_chassis->SetForce(weight + downForce);
		
		for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
		{
			ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
			ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
			const dVector tireWeight(tireBody->GetForce());
			const dVector tireDownForce(up.Scale(gravity * tireBody->GetMassMatrix().m_w));
			tireBody->SetForce(tireWeight + tireDownForce);
		}
	}
}

void ndMultiBodyVehicle::SetVehicleSolverModel(bool hardJoint)
{
	ndJointBilateralSolverModel openLoopMode = hardJoint ? m_jointkinematicOpenLoop : m_jointIterativeSoft;

	dAssert(m_chassis);
	const ndJointList& chassisJoints = m_chassis->GetJointList();
	for (ndJointList::dNode* node = chassisJoints.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		joint->SetSolverModel(openLoopMode);
	}
	
	ndJointBilateralSolverModel driveTrainMode = hardJoint ? m_jointkinematicCloseLoop : m_jointIterativeSoft;
	for (dList<ndMultiBodyVehicleDifferential*>::dNode* node = m_differentials.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		const ndJointList& jointList = joint->GetBody0()->GetJointList();
		for (ndJointList::dNode* node1 = jointList.GetFirst(); node1; node1 = node1->GetNext())
		{
			ndJointBilateralConstraint* const axle = node1->GetInfo();
			const char* const clasName = axle->ClassName();
			if (strcmp(clasName, "ndMultiBodyVehicleDifferential"))
			{
				axle->SetSolverModel(driveTrainMode);
			}
		}
	}
}

void ndMultiBodyVehicle::ApplyAligmentAndBalancing()
{
#if 1
	for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
		ndBodyDynamic* const chassisBody = tire->GetBody1()->GetAsBodyDynamic();

		bool savedSleepState = tireBody->GetSleepState();
		tire->UpdateTireSteeringAngleMatrix();
		
		dMatrix tireMatrix;
		dMatrix chassisMatrix;
		tire->CalculateGlobalMatrix(tireMatrix, chassisMatrix);
		
		// align tire velocity
		const dVector chassisVelocity(chassisBody->GetVelocityAtPoint(tireMatrix.m_posit));
		const dVector relVeloc(tireBody->GetVelocity() - chassisVelocity);
		dVector localVeloc(chassisMatrix.UnrotateVector(relVeloc));
		bool applyProjection = (localVeloc.m_x * localVeloc.m_x + localVeloc.m_z * localVeloc.m_z) > (dFloat32(0.05f) * dFloat32(0.05f));
		localVeloc.m_x *= dFloat32(0.3f);
		localVeloc.m_z *= dFloat32(0.3f);
		const dVector tireVelocity(chassisVelocity + chassisMatrix.RotateVector(localVeloc));
		
		// align tire angular velocity
		const dVector chassisOmega(chassisBody->GetOmega());
		const dVector relOmega(tireBody->GetOmega() - chassisOmega);
		dVector localOmega(chassisMatrix.UnrotateVector(relOmega));
		applyProjection = applyProjection || (localOmega.m_y * localOmega.m_y + localOmega.m_z * localOmega.m_z) > (dFloat32(0.05f) * dFloat32(0.05f));
		localOmega.m_y *= dFloat32(0.3f);
		localOmega.m_z *= dFloat32(0.3f);
		const dVector tireOmega(chassisOmega + chassisMatrix.RotateVector(localOmega));
		
		if (applyProjection)
		{
			tireBody->SetOmega(tireOmega);
			tireBody->SetVelocity(tireVelocity);
		}
		tireBody->RestoreSleepState(savedSleepState);
	}
#else
	for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
		bool savedSleepState = tireBody->GetSleepState();
		tire->UpdateTireSteeringAngleMatrix();
		tireBody->RestoreSleepState(savedSleepState);
	}
#endif

	for (dList<ndMultiBodyVehicleDifferential*>::dNode* node = m_differentials.GetFirst(); node; node = node->GetNext())
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

	const dFloat32 tireForceScale = dFloat32(3.0f);
	for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyDynamic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		// draw upper bumper
		dMatrix upperBumberMatrix(tireJoint->CalculateUpperBumperMatrix());
		//context.DrawFrame(tireJoint->CalculateUpperBumperMatrix());

		// show tire center of mass;
		dMatrix tireFrame(tireBody->GetMatrix());
		//context.DrawFrame(tireFrame);
		upperBumberMatrix.m_posit = tireFrame.m_posit;
		context.DrawFrame(upperBumberMatrix);

		totalMass += tireBody->GetMassMatrix().m_w;
		effectiveCom += tireFrame.m_posit.Scale(tireBody->GetMassMatrix().m_w);

		// draw tire forces
		const ndBodyKinematic::ndContactMap& contactMap = tireBody->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::dNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					dMatrix frame(contactPoint.m_normal, contactPoint.m_dir0, contactPoint.m_dir1, contactPoint.m_point);

					dVector localPosit(m_localFrame.UntransformVector(chassisMatrix.UntransformVector(contactPoint.m_point)));
					//dFloat32 offset = (localPosit.m_z > dFloat32(0.0f)) ? dFloat32(0.2f) : dFloat32(-0.2f);
					//frame.m_posit += contactPoint.m_dir0.Scale(offset);
					frame.m_posit += contactPoint.m_normal.Scale(0.1f);

					// normal force
					dFloat32 normalForce = tireForceScale * contactPoint.m_normal_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_normal.Scale (normalForce), forceColor);

					// lateral force
					dFloat32 lateralForce = -tireForceScale * contactPoint.m_dir0_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir0.Scale(lateralForce), lateralColor);

					// longitudinal force
					dFloat32 longitudinalForce = -tireForceScale * contactPoint.m_dir1_Force.m_force / scale;
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

void ndMultiBodyVehicle::BrushTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	// calculate longitudinal slip ratio
	const ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
	const ndBodyDynamic* const otherBody = (contactPoint.m_body0 == tireBody) ? ((ndBodyKinematic*)contactPoint.m_body1)->GetAsBodyDynamic() : ((ndBodyKinematic*)contactPoint.m_body0)->GetAsBodyDynamic();
	dAssert(tireBody != otherBody);
	dAssert((tireBody == contactPoint.m_body0) || (tireBody == contactPoint.m_body1));

	const dVector tireVeloc(tireBody->GetVelocity());
	const dFloat32 tireSpeed = dAbs(tireVeloc.DotProduct(contactPoint.m_dir1).GetScalar());
	if (dAbs(tireSpeed) > dFloat32(0.44f))
	{
		// apply brush tire model only when center travels faster that 10 miles per hours
		const dVector contactVeloc0(tireBody->GetVelocityAtPoint(contactPoint.m_point) - tireBody->GetVelocity() + tireVeloc);
		const dVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
		const dVector relVeloc(contactVeloc0 - contactVeloc1);

		const dFloat32 relSpeed = dAbs(relVeloc.DotProduct(contactPoint.m_dir1).GetScalar());
		const dFloat32 longitudialSlip = relSpeed / tireSpeed;

#if 1
		// calculate side slip ratio
		const dFloat32 sideSpeed = dAbs(relVeloc.DotProduct(contactPoint.m_dir0).GetScalar());
		const dFloat32 lateralSlip = sideSpeed / (relSpeed + dFloat32(1.0f));

		const dFloat32 den = dFloat32(1.0f) / (dFloat32(1.0f) + longitudialSlip);
		const dFloat32 v = lateralSlip * den;
		const dFloat32 u = longitudialSlip * den;

		//if (u > 0.5f || v > 0.5f)
		//dTrace(("(%d %f %f)\n", tireBody->GetId(), u, v));
		tire->m_lateralSideSlip = dMax (tire->m_lateralSideSlip, v);
		tire->m_longitidinalSideSlip = dMax(tire->m_longitidinalSideSlip, u);

		const ndWheelDescriptor& info = tire->GetInfo();
		const dFloat32 cz = info.m_laterialStiffness  * v;
		const dFloat32 cx = info.m_longitudinalStiffness  * u;
		const dFloat32 gamma = dSqrt(cx * cx + cz * cz) + dFloat32(1.0e-3f);

		const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
		const dFloat32 lateralFrictionCoefficient = frictionCoefficient * cz / gamma;
		const dFloat32 longitudinalFrictionCoefficient = frictionCoefficient * cx / gamma;
		//dTrace(("(%d %f %f) ", tireBody->GetId(), lateralFrictionCoefficient, longitudinalFrictionCoefficient));

		contactPoint.m_material.m_restitution = dFloat32(0.1f);
		contactPoint.m_material.m_staticFriction0 = lateralFrictionCoefficient;
		contactPoint.m_material.m_dynamicFriction0 = lateralFrictionCoefficient;
		contactPoint.m_material.m_staticFriction1 = longitudinalFrictionCoefficient;
		contactPoint.m_material.m_dynamicFriction1 = longitudinalFrictionCoefficient;
#else
		//dTrace(("(%d %f) ", tireBody->GetId(), contactPoint.m_normal_Force.m_force));

		const dFloat32 longitudialSlip = relSpeed / tireSpeed;

		// calculate side slip ratio
		const dFloat32 sideSpeed = dAbs(relVeloc.DotProduct(contactPoint.m_dir0).GetScalar());
		const dFloat32 lateralSlip = sideSpeed / (relSpeed + dFloat32(0.01f));

		//const dFloat32 lateralKinetiFrictionFactor = lateralSlip >= dFloat32(1.0f) ? dFloat32(0.6f) : dFloat32(1.0f);
		//const dFloat32 longitudinalKinetiFrictionFactor = longitudialSlip >= dFloat32 (1.0f) ? dFloat32(0.6f) : dFloat32(1.0f);
		const dFloat32 lateralKinetiFrictionFactor = dFloat32(1.0f);
		const dFloat32 longitudinalKinetiFrictionFactor = dFloat32(1.0f);

		//if (lateralSlip > 10.0f)
		//dTrace(("(id=%d longSlip(%f) sizegSlip(%f)\n", tireBody->GetId(), longitudialSlip, lateralSlip));

		const dFloat32 den = dFloat32(1.0f) / (dFloat32(1.0f) + longitudialSlip);
		const dFloat32 v = lateralSlip * den;
		const dFloat32 u = longitudialSlip * den;

		const ndWheelDescriptor& info = tire->GetInfo();
		const dFloat32 cz = info.m_laterialStiffness  * v;
		const dFloat32 cx = info.m_longitudinalStiffness  * u;
		const dFloat32 gamma = dSqrt(cx * cx + cz * cz) + dFloat32(1.0e-3f);

		const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
		const dFloat32 lateralFrictionCoefficient = frictionCoefficient * cz / gamma;
		const dFloat32 longitudinalFrictionCoefficient = frictionCoefficient * cx / gamma;

		//dTrace(("(id=%d longSlip(%f) sizegSlip(%f)\n", tireBody->GetId(), longitudinalFrictionCoefficient, lateralFrictionCoefficient));

		contactPoint.m_material.m_restitution = dFloat32(0.1f);
		contactPoint.m_material.m_staticFriction0 = lateralFrictionCoefficient;
		contactPoint.m_material.m_staticFriction1 = longitudinalFrictionCoefficient;

		contactPoint.m_material.m_dynamicFriction0 = lateralFrictionCoefficient * lateralKinetiFrictionFactor;
		contactPoint.m_material.m_dynamicFriction1 = longitudinalFrictionCoefficient * longitudinalKinetiFrictionFactor;

		//dTrace(("(%d %f) ", tireBody->GetId(), contactPoint.m_normal_Force.m_force));

#endif
	}
	else
	{
		const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
		contactPoint.m_material.m_restitution = dFloat32(0.1f);
		contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
		contactPoint.m_material.m_staticFriction1 = frictionCoefficient;

		contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient;
		contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient;
	}
}

void ndMultiBodyVehicle::ApplyTireModel()
{
	for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		dAssert(((ndShape*)tire->GetBody0()->GetCollisionShape().GetShape())->GetAsShapeChamferCylinder());

		tire->m_lateralSideSlip = dFloat32(0.0f);
		tire->m_longitidinalSideSlip = dFloat32(0.0f);

		const ndBodyKinematic::ndContactMap& contactMap = tire->GetBody0()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				ndContactPointList& contactPoints = contact->GetContactPoints();
				const ndBodyKinematic* const otherBody = contact->GetBody1();
				if (((ndShape*)otherBody->GetCollisionShape().GetShape())->GetAsShapeStaticMeshShape())
				{
					// for mesh collision need to dos some contact post processing
					// first remove any contact duplicate, these are contact produce by tow or 
					// more polygons, but this so are close that they can generate ill 
					// formed rows in the solver mass matrix
					for (ndContactPointList::dNode* contactNode0 = contactPoints.GetFirst(); contactNode0; contactNode0 = contactNode0->GetNext())
					{
						const ndContactPoint& contactPoint0 = contactNode0->GetInfo();
						for (ndContactPointList::dNode* contactNode1 = contactNode0->GetNext(); contactNode1; contactNode1 = contactNode1->GetNext())
						{
							const ndContactPoint& contactPoint1 = contactNode1->GetInfo();
							const dVector error(contactPoint1.m_point - contactPoint0.m_point);
							dFloat32 err2 = error.DotProduct(error).GetScalar();
							if (err2 < D_MIN_CONTACT_CLOSE_DISTANCE2)
							{
								contactPoints.Remove(contactNode1);
								break;
							}
						}
					}

					//maxPenetration = 0;
					//if (maxPenetration > D_MAX_CONTACT_PENETRATION)
					//{
					//	dAssert(0);
					//	// if the max penetration is too much, 
					//	// do a convex cast to find the tire location 
					//	// over the mesh and teleport the tire to that location.
					//	
					//	dFixSizeArray<ndContactPoint, 8> closestHit;
					//	const ndWheelDescriptor& info = tire->GetInfo();
					//	const dMatrix tireUpperBumperMatrix(tire->CalculateUpperBumperMatrix());
					//	const dVector dest(tireUpperBumperMatrix.m_posit - tireUpperBumperMatrix.m_up.Scale(info.m_maxLimit - info.m_minLimit));
					//	dFloat32 param = ndContactSolver::ConvexCast(tire->GetBody0()->GetCollisionShape(), tireUpperBumperMatrix, dest, otherBody->GetCollisionShape(), otherBody->GetMatrix(), closestHit);
					//	if (param > dFloat32(0.0f))
					//	{
					//		ndBodyKinematic* const tireBody = tire->GetBody0();
					//		dMatrix tireMatrix(tire->GetLocalMatrix0() * tireBody->GetMatrix());
					//		dFloat32 tirePosition = tireUpperBumperMatrix.m_up.DotProduct(tireUpperBumperMatrix.m_posit - tireMatrix.m_posit).GetScalar();
					//		dFloat32 tireNewPosition = param * (info.m_maxLimit - info.m_minLimit);
					//		dFloat32 positionError = dMin(tirePosition - tireNewPosition, D_MAX_CONTACT_PENETRATION);
					//		dAssert(positionError >= 0.0f);
					//		tirePosition -= positionError;
					//		tireMatrix.m_posit = tireUpperBumperMatrix.m_posit - tireUpperBumperMatrix.m_up.Scale(tirePosition);
					//		tireBody->SetMatrix(tire->GetLocalMatrix0().Inverse() * tireMatrix);
					//
					//		dAssert(closestHit.GetCount());
					//		for (dInt32 i = closestHit.GetCount() - 1; i > 0; i--)
					//		{
					//			// remove duplicates.
					//			for (dInt32 j = i - 1; j >= 0; j--)
					//			{
					//				const dVector error(closestHit[i].m_point - closestHit[j].m_point);
					//				dFloat32 err2 = error.DotProduct(error).GetScalar();
					//				if (err2 < D_MIN_CONTACT_CLOSE_DISTANCE2)
					//				{
					//					closestHit.SetCount(closestHit.GetCount() - 1);
					//					break;
					//				}
					//			}
					//		}
					//
					//		// repopulate the contact array
					//		contactPoints.RemoveAll();
					//		for (dInt32 i = closestHit.GetCount() - 1; i >= 0; i--)
					//		{
					//			ndContactMaterial* const contactPoint = &contactPoints.Append()->GetInfo();
					//
					//			const dMatrix normalBase(closestHit[i].m_normal);
					//			contactPoint->m_point = closestHit[i].m_point;
					//			contactPoint->m_normal = normalBase.m_front;
					//			contactPoint->m_dir0 = normalBase.m_up;
					//			contactPoint->m_dir1 = normalBase.m_right;
					//			contactPoint->m_penetration = closestHit[i].m_penetration;
					//			contactPoint->m_body0 = tireBody;
					//			contactPoint->m_body1 = otherBody;
					//			contactPoint->m_shapeInstance0 = &contactPoint->m_body0->GetCollisionShape();
					//			contactPoint->m_shapeInstance1 = &contactPoint->m_body1->GetCollisionShape();
					//			contactPoint->m_shapeId0 = closestHit[i].m_shapeId0;
					//			contactPoint->m_shapeId1 = closestHit[i].m_shapeId1;
					//		}
					//	}
					//}
				}

				dMatrix tireBasisMatrix (tire->GetLocalMatrix1() * tire->GetBody1()->GetMatrix());
				tireBasisMatrix.m_posit = tire->GetBody0()->GetMatrix().m_posit;
				for (ndContactPointList::dNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					ndContactMaterial& contactPoint = contactNode->GetInfo();
					dFloat32 contactPathLocation = dAbs (contactPoint.m_normal.DotProduct(tireBasisMatrix.m_front).GetScalar());
					// contact are consider on the contact patch strip only if the are less than 
					// 45 degree angle from the tire axle
					if (contactPathLocation < dFloat32 (0.71f))
					{
						// align tire friction direction
						const dVector fronDir(contactPoint.m_normal.CrossProduct(tireBasisMatrix.m_front));
						contactPoint.m_dir1 = fronDir.Normalize();
						contactPoint.m_dir0 = contactPoint.m_dir1.CrossProduct(contactPoint.m_normal);
						
						// check if the contact is in the contact patch,
						// the is the 45 degree point around the tire vehicle axis. 
						dVector dir(contactPoint.m_point - tireBasisMatrix.m_posit);
						dAssert(dir.DotProduct(dir).GetScalar() > dFloat32(0.0f));
						dFloat32 contactPatch = tireBasisMatrix.m_up.DotProduct(dir.Normalize()).GetScalar();
						if (contactPatch < dFloat32(-0.71f))
						{
							BrushTireModel(tire, contactPoint);
						}
						else
						{
							const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
							contactPoint.m_material.m_restitution = dFloat32(0.1f);
							contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
							contactPoint.m_material.m_staticFriction1 = frictionCoefficient;

							contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient;
							contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient;
						}
					}
					else
					{
						const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
						contactPoint.m_material.m_restitution = dFloat32(0.1f);
						contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
						contactPoint.m_material.m_staticFriction1 = frictionCoefficient;

						contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient;
						contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient;
					}
				}
			}
		}
	}
	//dTrace(("\n"));
}

ndMultiBodyVehicle::ndDownForce::ndDownForce()
	:m_gravity(dFloat32(-10.0f))
{
	m_downForceTable[0].m_speed = dFloat32(0.0f) * dFloat32(0.27f);
	m_downForceTable[0].m_forceFactor = 0.0f;
	m_downForceTable[0].m_aerodynamicDownforceConstant = dFloat32(0.0f);

	m_downForceTable[1].m_speed = dFloat32(30.0f) * dFloat32(0.27f);
	m_downForceTable[1].m_forceFactor = 1.0f;
	m_downForceTable[1].m_aerodynamicDownforceConstant = CalcuateFactor(&m_downForceTable[0]);

	m_downForceTable[2].m_speed = dFloat32(60.0f) * dFloat32(0.27f);
	m_downForceTable[2].m_forceFactor = 1.6f;
	m_downForceTable[2].m_aerodynamicDownforceConstant = CalcuateFactor(&m_downForceTable[1]);
	
	m_downForceTable[3].m_speed = dFloat32(140.0f) * dFloat32(0.27f);
	m_downForceTable[3].m_forceFactor = 3.0f;
	m_downForceTable[3].m_aerodynamicDownforceConstant = CalcuateFactor(&m_downForceTable[2]);

	m_downForceTable[4].m_speed = dFloat32(1000.0f) * dFloat32(0.27f);
	m_downForceTable[4].m_forceFactor = 3.0f;
	m_downForceTable[4].m_aerodynamicDownforceConstant = CalcuateFactor(&m_downForceTable[3]);

#if 0
	dFloat32 speed = 0;
	for (dInt32 i = 0; i < 100; i++)
	{
		dTrace(("%f %f\n", speed, GetDownforceFactor(speed)/m_gravity));
		speed += (150.0f / 100.0f) * 0.27f;
	}
#endif
}

dFloat32 ndMultiBodyVehicle::ndDownForce::CalcuateFactor(const ndSpeedForcePair* const entry0) const
{
	const ndSpeedForcePair* const entry1 = entry0 + 1;
	dFloat32 num = dMax(entry1->m_forceFactor - entry0->m_forceFactor, dFloat32 (0.0f));
	dFloat32 den = dMax (dAbs (entry1->m_speed - entry0->m_speed), 1.0f);
	return num / (den * den);
}

dFloat32 ndMultiBodyVehicle::ndDownForce::GetDownforceFactor(dFloat32 speed) const
{
	dAssert(speed >= dFloat32(0.0f));
	dInt32 index = 0;
	for (dInt32 i = sizeof(m_downForceTable) / sizeof(m_downForceTable[0]) - 1; i ; i--)
	{
		if (m_downForceTable[i].m_speed <= speed)
		{
			index = i;
			break;
		}
	}

	dFloat32 deltaSpeed = speed - m_downForceTable[index].m_speed;
	dFloat32 downForceFactor = m_downForceTable[index].m_forceFactor + m_downForceTable[index + 1].m_aerodynamicDownforceConstant * deltaSpeed * deltaSpeed;
	return downForceFactor * m_gravity;
}

void ndMultiBodyVehicle::PostUpdate(ndWorld* const, dFloat32)
{
	ApplyAligmentAndBalancing();
}

void ndMultiBodyVehicle::Update(ndWorld* const world, dFloat32 timestep)
{
	ApplyInputs(world, timestep);

	// Apply Vehicle Dynamics controls
	// no implemented yet

	// apply down force
	ApplyAerodynamics();
	ApplyTireModel();
}
