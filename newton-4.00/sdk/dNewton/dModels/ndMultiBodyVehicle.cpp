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
//#include "ndJointHinge.h"
#include "ndJointWheel.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"

ndMultiBodyVehicle::ndMultiBodyVehicle(const dVector& frontDir, const dVector& upDir)
	:ndModel()
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
	,m_tireShape(new ndShapeChamferCylinder(dFloat32(0.75f), dFloat32(0.5f)))
	,m_tiresList()
	,m_brakeTires()
	,m_steeringTires()
	,m_brakeTorque(dFloat32(0.0f))
	,m_steeringAngle(dFloat32 (0.0f))
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
	,m_tiresList()
	,m_steeringTires()
	,m_brakeTorque(dFloat32(0.0f))
	,m_steeringAngle(dFloat32(0.0f))
	,m_steeringAngleMemory(dFloat32(0.0f))
{
	m_tireShape->AddRef();
}

ndMultiBodyVehicle::~ndMultiBodyVehicle()
{
	m_tireShape->Release();
}

void ndMultiBodyVehicle::AddChassis(ndBodyDynamic* const chassis)
{
	m_chassis = chassis;
}

void ndMultiBodyVehicle::SetSteeringAngle(dFloat32 angleInRadians)
{
	m_steeringAngle = angleInRadians;
}

ndJointWheel* ndMultiBodyVehicle::AddTire(ndWorld* const world, ndBodyDynamic* const tire)
{
	dMatrix tireFrame(dGetIdentityMatrix());
	tireFrame.m_front = dVector(0.0f, 0.0f, 1.0f, 0.0f);
	tireFrame.m_up    = dVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = dVector(-1.0f, 0.0f, 0.0f, 0.0f);
	dMatrix matrix (tireFrame * m_localFrame * m_chassis->GetMatrix());
	matrix.m_posit = tire->GetMatrix().m_posit;

	// make tire inertia spherical
	dVector inertia(tire->GetMassMatrix());
	dFloat32 maxInertia(dMax(dMax(inertia.m_x, inertia.m_y), inertia.m_z));
	inertia.m_x = maxInertia;
	inertia.m_y = maxInertia;
	inertia.m_z = maxInertia;
	tire->SetMassMatrix(inertia);

	ndJointWheel* const tireJoint = new ndJointWheel(matrix, tire, m_chassis);
	m_tiresList.Append(tireJoint);
	world->AddJoint(tireJoint);
	return tireJoint;
}

ndShapeInstance ndMultiBodyVehicle::CreateTireShape(dFloat32 radius, dFloat32 width) const
{
	ndShapeInstance tireCollision(m_tireShape);
	dVector scale(2.0f * width, radius, radius, 0.0f);
	tireCollision.SetScale(scale);
	return tireCollision;
}

void ndMultiBodyVehicle::SetBrakeTorque(dFloat32 brakeToqrue)
{
	m_brakeTorque = dAbs(brakeToqrue);
}

void ndMultiBodyVehicle::SetAsSteering(ndJointWheel* const tire)
{
	m_steeringTires.Append(tire);
}

void ndMultiBodyVehicle::SetAsBrake(ndJointWheel* const tire)
{
	m_brakeTires.Append(tire);
}

void ndMultiBodyVehicle::Update(const ndWorld* const world, dFloat32 timestep)
{
	ApplyAligmentAndBalancing();
	ApplyBrakes();
	ApplySteering();
}

void ndMultiBodyVehicle::ApplyAligmentAndBalancing()
{
	const dVector chassisOmega(m_chassis->GetOmega());
	const dVector upDir(m_chassis->GetMatrix().RotateVector(m_localFrame.m_up));
	for (dList<ndJointWheel*>::dListNode* node = m_tiresList.GetFirst(); node; node = node->GetNext())
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

void ndMultiBodyVehicle::ApplyBrakes()
{
	for (dList<ndJointWheel*>::dListNode* node = m_brakeTires.GetFirst(); node; node = node->GetNext())
	{
		ndJointWheel* const tire = node->GetInfo();
		tire->SetBrakeTorque(m_brakeTorque);
	}
}