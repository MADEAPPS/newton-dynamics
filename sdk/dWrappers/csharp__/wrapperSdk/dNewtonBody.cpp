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
#include "dNewtonBody.h"
#include "dNewtonWorld.h"
#include "dNewtonCollision.h"


dNewtonBody::ScopeLock::ScopeLock (unsigned int* const lock)
	:m_atomicLock(lock)
{
	const int maxCount = 1024 * 32;
	for (int i = 0; (i < maxCount) && NewtonAtomicSwap((int*)m_atomicLock, 1); i++) {
		NewtonYield();
	}
}

dNewtonBody::ScopeLock::~ScopeLock()
{
	NewtonAtomicSwap((int*)m_atomicLock, 0);
}


dNewtonBody::dNewtonBody(const dMatrix& matrix)
	:dAlloc()
	,m_body(NULL)
	,m_posit0(matrix.m_posit)
	,m_posit1(matrix.m_posit)
	,m_interpolatedPosit(matrix.m_posit)
	,m_rotation0(matrix)
	,m_rotation1(m_rotation0)
	,m_interpolatedRotation(m_rotation0)
	,m_lock(0)
{
}

dNewtonBody::~dNewtonBody()
{
	Destroy();
}


void* dNewtonBody::GetBody() const
{
	return m_body;
}


bool dNewtonBody::GetSleepState() const
{
	return NewtonBodyGetSleepState(m_body) ? true : false;
}

void dNewtonBody::SetSleepState(bool state) const
{
	NewtonBodySetSleepState(m_body, state ? 1 : 0);
}

void* dNewtonBody::GetInterpolatedPosition()
{
	const dNewtonWorld* const world = (dNewtonWorld*)NewtonWorldGetUserData(NewtonBodyGetWorld(m_body));
	ScopeLock scopelock(&m_lock);
	m_interpolatedPosit = m_posit0 + (m_posit1 - m_posit0).Scale(world->m_interpotationParam);
	return &m_interpolatedPosit.m_x;
}

void* dNewtonBody::GetInterpolatedRotation()
{
	const dNewtonWorld* const world = (dNewtonWorld*) NewtonWorldGetUserData(NewtonBodyGetWorld(m_body));
	ScopeLock scopelock(&m_lock);
	m_interpolatedRotation = m_rotation0.Slerp(m_rotation1, world->m_interpotationParam);
	return &m_interpolatedRotation.m_q0;
}

void* dNewtonBody::GetPosition()
{
	return &m_posit1.m_x;
}

void* dNewtonBody::GetRotation()
{
	return &m_rotation1.m_q0;
}

void dNewtonBody::SetPosition(dFloat x, dFloat y, dFloat z)
{
	dQuaternion rot;
	NewtonBodyGetRotation(m_body, &rot.m_q0);
	dMatrix mat(rot, dVector(x, y, z));
	NewtonBodySetMatrix(m_body, &mat[0][0]);
}

void dNewtonBody::SetRotation(dFloat x, dFloat y, dFloat z, dFloat w)
{
	dVector pos(0, 0, 0);
	NewtonBodyGetPosition(m_body, &pos.m_x);
	dMatrix mat(dQuaternion(x, y, z, w), pos);
	NewtonBodySetMatrix(m_body, &mat[0][0]);
}

void dNewtonBody::SetUserData(void* userData)
{
	m_userData = userData;
}

void* dNewtonBody::GetUserData()
{
	return m_userData;
}

void* dNewtonBody::GetVelocity()
{
	NewtonBodyGetVelocity(m_body, &m_velocity.m_x);
	return &m_velocity;
}

void* dNewtonBody::GetOmega()
{
	NewtonBodyGetOmega(m_body, &m_omega.m_x);
	return &m_omega;
}

void dNewtonBody::SetVelocity(dFloat x, dFloat y, dFloat z)
{
	dVector vel(x,y,z);
	NewtonBodySetVelocity(m_body, &vel.m_x);
}

void dNewtonBody::SetOmega(dFloat x, dFloat y, dFloat z)
{
	dVector omg(x, y, z);
	NewtonBodySetOmega(m_body, &omg.m_x);
}

float dNewtonBody::GetLinearDamping()
{
	return NewtonBodyGetLinearDamping(m_body);
}

void dNewtonBody::SetLinearDamping(dFloat x)
{
	NewtonBodySetLinearDamping(m_body, x);
}

void* dNewtonBody::GetAngularDamping()
{
	NewtonBodyGetAngularDamping(m_body, &m_angulardamping.m_x);
	return &m_angulardamping;
}

void dNewtonBody::SetAngularDamping(dFloat x, dFloat y, dFloat z)
{
	dVector damp(x, y, z);
	NewtonBodySetAngularDamping(m_body, &damp.m_x);
}

void* dNewtonBody::GetCenterOfMass()
{
	NewtonBodyGetCentreOfMass(m_body, &m_com.m_x);
	return &m_com;
}

void dNewtonBody::SetCenterOfMass(float com_x, float com_y, float com_z)
{
	dVector com;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass(m_body, &mass, &Ixx, &Iyy, &Izz);
	NewtonCollision* const collision = NewtonBodyGetCollision(m_body);
	NewtonBodySetMassProperties(m_body, mass, NewtonBodyGetCollision(m_body));
	NewtonBodyGetCentreOfMass (m_body, &com[0]);
	com.m_x += com_x;
	com.m_y += com_y;
	com.m_z += com_z;
	NewtonBodySetCentreOfMass(m_body, &com[0]);
}

void dNewtonBody::CalculateBuoyancyForces(const void* plane, void* force, void* torque, float bodyDensity)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass(m_body, &mass, &Ixx, &Iyy, &Izz);

	if (mass > 0.0f) {
		dMatrix matrix;
		dVector cog(0.0f);
		dVector accelPerUnitMass(0.0f);
		dVector torquePerUnitMass(0.0f);
		const dVector gravity(0.0f, -9.8f, 0.0f, 0.0f);

		NewtonBodyGetMatrix(m_body, &matrix[0][0]);
		NewtonBodyGetCentreOfMass(m_body, &cog[0]);
		cog = matrix.TransformVector(cog);
		NewtonCollision* const collision = NewtonBodyGetCollision(m_body);

		dFloat shapeVolume = NewtonConvexCollisionCalculateVolume(collision);
		dFloat fluidDensity = 1.0f / (bodyDensity * shapeVolume);
		dFloat viscosity = 0.995f;

		NewtonConvexCollisionCalculateBuoyancyAcceleration(collision, &matrix[0][0], &cog[0], &gravity[0], (float*)plane, fluidDensity, viscosity, &accelPerUnitMass[0], &torquePerUnitMass[0]);

		dVector finalForce(accelPerUnitMass.Scale(mass));
		dVector finalTorque(torquePerUnitMass.Scale(mass));

		dVector omega(0.0f);
		NewtonBodyGetOmega(m_body, &omega[0]);
		omega = omega.Scale(viscosity);
		NewtonBodySetOmega(m_body, &omega[0]);

		((float*)force)[0] = finalForce.m_x ;
		((float*)force)[1] = finalForce.m_y ;
		((float*)force)[2] = finalForce.m_z ;
		((float*)torque)[0] = finalTorque.m_x;
		((float*)torque)[1] = finalTorque.m_y;
		((float*)torque)[2] = finalTorque.m_z;
	}
}

void dNewtonBody::OnBodyTransform(const dFloat* const matrixPtr, int threadIndex)
{
	dMatrix matrix(matrixPtr);

	ScopeLock scopelock(&m_lock);
	m_posit0 = m_posit1;
	m_rotation0 = m_rotation1;
	m_posit1 = matrix.m_posit;
	m_rotation1 = dQuaternion(matrix);
	dFloat angle = m_rotation0.DotProduct(m_rotation1);
	if (angle < 0.0f) {
		m_rotation1.Scale(-1.0f);
	}
}

void dNewtonBody::OnForceAndTorque(dFloat timestep, int threadIndex)
{
}

void dNewtonBody::OnForceAndTorqueCallback (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dNewtonBody* const me = (dNewtonBody*)NewtonBodyGetUserData(body);
	dAssert(me);
	me->OnForceAndTorque(timestep, threadIndex);
}

void dNewtonBody::OnBodyTransformCallback (const NewtonBody* const body, const dFloat* const matrix, int threadIndex)
{
	dNewtonBody* const me = (dNewtonBody*)NewtonBodyGetUserData(body);
	dAssert(me);
	me->OnBodyTransform(matrix, threadIndex);
}


void dNewtonBody::Destroy()
{
	if (m_body) {
		NewtonWaitForUpdateToFinish(NewtonBodyGetWorld(m_body));
		NewtonBodySetDestructorCallback(m_body, NULL);
		NewtonDestroyBody(m_body);
		m_body = NULL;
	}
}

void dNewtonBody::OnBodyDestroy(const NewtonBody* const body)
{
	dAssert(0);
/*
	dNewtonBody* const me = (dNewtonBody*)NewtonBodyGetUserData(body);
	if (me) {
		NewtonBodySetDestructorCallback(me->m_body, NULL);
		delete me;
	}
*/
}

void dNewtonBody::InitForceAccumulators()
{
}

void dNewtonBody::AddForce(dFloat x, dFloat y, dFloat z)
{
}

void dNewtonBody::AddTorque(dFloat x, dFloat y, dFloat z)
{
}

dNewtonKinematicBody::dNewtonKinematicBody(dNewtonWorld* const world, dNewtonCollision* const collision, dMatrix matrix, dFloat mass)
	:dNewtonBody(matrix)
{
	NewtonWorld* const newton = world->m_world;
	NewtonWaitForUpdateToFinish(newton);

	m_body = NewtonCreateDynamicBody(newton, collision->m_shape, &matrix[0][0]);
	collision->DeleteShape();
	collision->SetShape(NewtonBodyGetCollision(m_body));

	NewtonBodySetMassProperties(m_body, mass, NewtonBodyGetCollision(m_body));

	NewtonBodySetUserData(m_body, this);
	NewtonBodySetTransformCallback(m_body, OnBodyTransformCallback);
	NewtonBodySetForceAndTorqueCallback(m_body, OnForceAndTorqueCallback);
}

dNewtonDynamicBody::dNewtonDynamicBody(dNewtonWorld* const world, dNewtonCollision* const collision, dMatrix matrix, dFloat mass)
	:dNewtonBody(matrix)
{
	NewtonWorld* const newton = world->m_world;

	NewtonWaitForUpdateToFinish(newton);
	m_body = NewtonCreateDynamicBody(newton, collision->m_shape, &matrix[0][0]);
	collision->DeleteShape();
	collision->SetShape(NewtonBodyGetCollision(m_body));

	NewtonBodySetMassProperties(m_body, mass, NewtonBodyGetCollision(m_body));

	NewtonBodySetUserData(m_body, this);
	NewtonBodySetTransformCallback(m_body, OnBodyTransformCallback);
	NewtonBodySetForceAndTorqueCallback(m_body, OnForceAndTorqueCallback);
}

void dNewtonDynamicBody::InitForceAccumulators()
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;

	NewtonBodyGetMass(m_body, &mass, &Ixx, &Iyy, &Izz);
	const dNewtonWorld* const world = (dNewtonWorld*)NewtonWorldGetUserData(NewtonBodyGetWorld(m_body));

	m_externalForce = world->GetGravity().Scale(mass);
	m_externalTorque = dVector(0.0f);
}


void dNewtonDynamicBody::OnForceAndTorque(dFloat timestep, int threadIndex)
{
	NewtonBodySetForce(m_body, &m_externalForce[0]);
	NewtonBodySetTorque(m_body, &m_externalTorque[0]);
}


void dNewtonDynamicBody::AddForce(dFloat x, dFloat y, dFloat z)
{
	m_externalForce += dVector (x, y, z);
}

void dNewtonDynamicBody::AddTorque(dFloat x, dFloat y, dFloat z)
{
	m_externalTorque += dVector(x, y, z);
}


