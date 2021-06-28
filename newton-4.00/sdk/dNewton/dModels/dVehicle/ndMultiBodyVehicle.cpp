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
	,m_suspensionStiffnessModifier(dFloat32(1.0f))
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
	,m_gravityMag(dFloat32(0.0f))
	,m_suspensionStiffnessModifier(dFloat32(1.0f))
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

void ndMultiBodyVehicle::AddChassis(ndBodyDynamic* const chassis, dFloat32 gravityMag)
{
	m_chassis = chassis;
	m_gravityMag = dAbs(gravityMag);
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
	dFloat32 downForceFactor = m_downForce.GetDownforceFactor(GetSpeed());
	m_suspensionStiffnessModifier = dFloat32(1.0f);
	if (downForceFactor > dFloat32(0.0f))
	{
		const dVector up(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
		const dVector weight(m_chassis->GetForce());
		const dVector downForce(up.Scale(-m_gravityMag * downForceFactor * m_chassis->GetMassMatrix().m_w));
		m_chassis->SetForce(weight + downForce);
		m_suspensionStiffnessModifier = up.DotProduct(weight).GetScalar() / up.DotProduct(weight + downForce).GetScalar();
		//dTrace(("%f\n", m_suspensionStiffnessModifier));
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

		dMatrix tireMatrix;
		dMatrix chassisMatrix;
		tire->CalculateTireSteeringMatrix();
		tire->CalculateGlobalMatrix(tireMatrix, chassisMatrix);
		
		// align tire matrix 
		const ndWheelDescriptor& info = tire->GetInfo();
		const dFloat32 spinAngle = -tire->CalculateAngle(tireMatrix.m_up, chassisMatrix.m_up, chassisMatrix.m_front);
		dVector localPosit(chassisMatrix.UntransformVector(tireMatrix.m_posit));
		localPosit.m_x = dFloat32(0.0f);
		localPosit.m_z = dFloat32(0.0f);
		localPosit.m_y = dClamp(localPosit.m_y, info.m_minLimit, info.m_maxLimit);
		tireMatrix = dPitchMatrix(spinAngle) * chassisMatrix;
		tireMatrix.m_posit = chassisMatrix.TransformVector(localPosit);

		// align tire velocity
		const dVector chassiVelocity(chassisBody->GetVelocityAtPoint(tireMatrix.m_posit));
		const dVector relVeloc(tireBody->GetVelocity() - chassiVelocity);

		dVector localVeloc(chassisMatrix.UnrotateVector(relVeloc));
		localVeloc.m_x = dFloat32(0.0f);
		localVeloc.m_z = dFloat32(0.0f);
		const dVector tireVelocity(chassiVelocity + chassisMatrix.RotateVector(localVeloc));
		
		// align tire angular velocity
		const dVector chassisOmega(chassisBody->GetOmega());
		const dVector relOmega(tireBody->GetOmega() - chassisOmega);
		dVector localOmega(chassisMatrix.UnrotateVector(relOmega));
		localOmega.m_y = dFloat32(0.0f);
		localOmega.m_z = dFloat32(0.0f);
		const dVector tireOmega(chassisOmega + chassisMatrix.RotateVector(localOmega));

		tireMatrix = tire->GetLocalMatrix0().Inverse() * tireMatrix;

		bool savedSleepState = tireBody->GetSleepState();
		tireBody->SetOmega(tireOmega);
		//tireBody->SetVelocity(tireVelocity);
		tireBody->SetMatrix(tireMatrix);
		tireBody->RestoreSleepState(savedSleepState);
	}
#else
	for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		tire->UpdateTireSteeringAngleMatrix();
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
//	context.DrawLine(p0, p1, dVector(1.0f, 1.0f, 1.0f, 0.0f));

	// draw velocity vector
	dVector veloc(m_chassis->GetVelocity());
	dVector p2(p0 + veloc.Scale (0.25f));
//	context.DrawLine(p0, p2, dVector(1.0f, 1.0f, 0.0f, 0.0f));

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
//	context.DrawLine(chassisMatrix.m_posit, chassisMatrix.m_posit + weight, forceColor);

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
//		context.DrawFrame(upperBumberMatrix);

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
//	context.DrawFrame(chassisMatrix);
}

void ndMultiBodyVehicle::BrushTireModel(const ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
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
	const dFloat32 lateralSlip = sideSpeed / (relSpeed + dFloat32 (0.01f));

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
	const dFloat32 gamma = dSqrt(cx * cx + cz * cz) + dFloat32 (1.0e-3f);

	const dFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
	const dFloat32 lateralFrictionCoefficient = frictionCoefficient * cz / gamma;
	const dFloat32 longitudinalFrictionCoefficient = frictionCoefficient * cx / gamma;

//dTrace(("(id=%d longSlip(%f) sizegSlip(%f)\n", tireBody->GetId(), longitudinalFrictionCoefficient, lateralFrictionCoefficient));

	contactPoint.m_material.m_restitution = dFloat32 (0.1f);
	contactPoint.m_material.m_staticFriction0 = lateralFrictionCoefficient;
	contactPoint.m_material.m_staticFriction1 = longitudinalFrictionCoefficient;

	contactPoint.m_material.m_dynamicFriction0 = lateralFrictionCoefficient * lateralKinetiFrictionFactor;
	contactPoint.m_material.m_dynamicFriction1 = longitudinalFrictionCoefficient * longitudinalKinetiFrictionFactor;

//dTrace(("(%d %f) ", tireBody->GetId(), contactPoint.m_normal_Force.m_force));
}

void ndMultiBodyVehicle::ApplyTiremodel()
{
	for (dList<ndMultiBodyVehicleTireJoint*>::dNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		dAssert(((ndShape*)tire->GetBody0()->GetCollisionShape().GetShape())->GetAsShapeChamferCylinder());

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
					//dFloat32 maxPenetration = dFloat32(0.0f);
					for (ndContactPointList::dNode* contactNode0 = contactPoints.GetFirst(); contactNode0; contactNode0 = contactNode0->GetNext())
					{
						const ndContactPoint& contactPoint0 = contactNode0->GetInfo();
						//maxPenetration = dMax(contactNode0->GetInfo().m_penetration, maxPenetration);
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

				const dMatrix tireMatrix (tire->GetLocalMatrix1() * tire->GetBody1()->GetMatrix());
				for (ndContactPointList::dNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
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
	m_downForceTable[0].m_aerodynamicDownforceConstant = dFloat32(0.0f);

	m_downForceTable[1].m_speed = dFloat32(60.0f) * dFloat32(0.27f);
	m_downForceTable[1].m_forceFactor = 1.0f;
	dFloat32 num = m_downForceTable[1].m_forceFactor - m_downForceTable[0].m_forceFactor;
	dFloat32 den = m_downForceTable[1].m_speed - m_downForceTable[0].m_speed;
	m_downForceTable[1].m_aerodynamicDownforceConstant = num / (den * den);

	m_downForceTable[2].m_speed = dFloat32(200.0f) * dFloat32(0.27f);
	m_downForceTable[2].m_forceFactor = 3.0f;
	num = m_downForceTable[2].m_forceFactor - m_downForceTable[1].m_forceFactor;
	den = m_downForceTable[2].m_speed - m_downForceTable[1].m_speed;
	m_downForceTable[2].m_aerodynamicDownforceConstant = num / (den * den);
}

dFloat32 ndMultiBodyVehicle::ndDownForce::GetDownforceFactor(dFloat32 speed) const
{
	dAssert(speed >= dFloat32(0.0f));
	dFloat32 downForceFactor = m_downForceTable[2].m_forceFactor;
	if (speed < m_downForceTable[0].m_speed)
	{
		downForceFactor = m_downForceTable[0].m_forceFactor;
	}
	else if (speed < m_downForceTable[1].m_speed)
	{
		dFloat32 deltaSpeed = speed - m_downForceTable[0].m_forceFactor;
		downForceFactor = m_downForceTable[0].m_forceFactor + m_downForceTable[1].m_aerodynamicDownforceConstant * deltaSpeed * deltaSpeed;
	}
	else if (speed < m_downForceTable[2].m_speed)
	{
		dFloat32 deltaSpeed = speed - m_downForceTable[1].m_forceFactor;
		downForceFactor = m_downForceTable[1].m_forceFactor + m_downForceTable[2].m_aerodynamicDownforceConstant * deltaSpeed * deltaSpeed;
	}
	return downForceFactor;
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
	ApplyTiremodel();
}
