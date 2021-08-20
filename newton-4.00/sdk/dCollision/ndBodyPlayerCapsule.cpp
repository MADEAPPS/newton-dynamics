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
#include "ndCollisionStdafx.h"
#include "ndBody.h"
#include "ndScene.h"
#include "ndContact.h"
#include "ndShapeCapsule.h"
#include "ndContactSolver.h"
#include "ndBodyPlayerCapsule.h"

#define D_DESCRETE_MOTION_STEPS		4
#define D_PLAYER_MAX_CONTACTS		8
#define D_PLAYER_MAX_ROWS			(3 * D_PLAYER_MAX_CONTACTS)

#define D_MAX_COLLIONSION_STEPS		8
#define D_SLOP_JUMP_ANGLE			dFloat32(0.8f)
#define D_MAX_COLLISION_PENETRATION	dFloat32 (5.0e-3f)

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsuleContactSolver
{
	public:
	ndBodyPlayerCapsuleContactSolver(ndBodyPlayerCapsule* const player);
	void CalculateContacts();

	ndContactPoint m_contactBuffer[D_PLAYER_MAX_ROWS];
	ndBodyPlayerCapsule* m_player;
	dInt32 m_contactCount;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndBodyPlayerCapsuleImpulseSolver
{
	public:
	ndBodyPlayerCapsuleImpulseSolver(ndBodyPlayerCapsule* const controller);

	dVector CalculateImpulse();
	void Reset(ndBodyPlayerCapsule* const controller);

	void AddAngularRows();
	dInt32 AddLinearRow(const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex = -1);
	dInt32 AddContactRow(const ndContactPoint* const contact, const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex = -1);
	void ApplyReaction(dFloat32 timestep);

	dMatrix m_invInertia;
	dVector m_veloc;
	ndJacobianPair m_jacobianPairs[D_PLAYER_MAX_ROWS];
	ndContactPoint* m_contactPoint[D_PLAYER_MAX_ROWS];
	dFloat32 m_rhs[D_PLAYER_MAX_ROWS];
	dFloat32 m_low[D_PLAYER_MAX_ROWS];
	dFloat32 m_high[D_PLAYER_MAX_ROWS];
	dInt32 m_normalIndex[D_PLAYER_MAX_ROWS];
	dFloat32 m_impulseMag[D_PLAYER_MAX_ROWS];
	dFloat32 m_mass;
	dFloat32 m_invMass;
	dInt32 m_rowCount;
} D_GCC_NEWTON_ALIGN_32;


ndBodyPlayerCapsule::ndBodyPlayerCapsule(const dMatrix& localAxis, dFloat32 mass, dFloat32 radius, dFloat32 height, dFloat32 stepHeight)
	:ndBodyKinematic()
{
	m_contactTestOnly = 1;
	m_impulse = dVector::m_zero;
	m_headingAngle = dFloat32(0.0f);
	m_forwardSpeed = dFloat32(0.0f);
	m_lateralSpeed = dFloat32(0.0f);
	m_stepHeight = dFloat32(0.0f);
	m_contactPatch = dFloat32(0.0f);
	m_height = height;
	m_radius = radius;
	m_weistScale = dFloat32(3.0f);
	m_crouchScale = dFloat32(0.5f);
	m_isAirbone = false;
	m_isOnFloor = false;
	m_isCrouched = false;

	dMatrix shapeMatrix(localAxis);
	shapeMatrix.m_posit = shapeMatrix.m_front.Scale(height * 0.5f);
	shapeMatrix.m_posit.m_w = 1.0f;
	
	height = dMax(height - 2.0f * radius / m_weistScale, dFloat32(0.1f));
	ndShapeInstance instance(new ndShapeCapsule(radius / m_weistScale, radius / m_weistScale, height));
	instance.SetLocalMatrix(shapeMatrix);
	instance.SetScale(dVector (dFloat32 (1.0f), m_weistScale, m_weistScale, dFloat32 (0.0f)));
	ndBodyKinematic::SetCollisionShape(instance);
	
	SetMassMatrix(mass, instance);
	m_invMass = GetInvMass();
	m_mass = ndBodyKinematic::m_mass.m_w;
		
	m_localFrame = shapeMatrix;
	m_localFrame.m_posit = dVector::m_wOne;
	m_contactPatch = radius / m_weistScale;
	m_stepHeight = dMax(stepHeight, m_contactPatch * dFloat32(2.0f));
}

ndBodyPlayerCapsule::ndBodyPlayerCapsule(const dLoadSaveBase::dDesc& desc)
	//:ndBodyKinematic(xmlNode->FirstChild("ndBodyKinematic"), shapesCache)
	:ndBodyKinematic(desc)
{
	dAssert(0);
	//m_contactTestOnly = 1;
	//m_impulse = dVector::m_zero;
	//m_headingAngle = dFloat32(0.0f);
	//m_forwardSpeed = dFloat32(0.0f);
	//m_lateralSpeed = dFloat32(0.0f);
	//m_stepHeight = dFloat32(0.0f);
	//m_contactPatch = dFloat32(0.0f);
	//
	//m_mass = xmlGetFloat(xmlNode, "mass");
	//m_height = xmlGetFloat(xmlNode, "height");
	//m_radius = xmlGetFloat(xmlNode, "radius");
	//m_weistScale = xmlGetFloat(xmlNode, "weistScale");
	//m_crouchScale = xmlGetFloat(xmlNode, "crouchScale");
	//m_isAirbone = xmlGetFloat(xmlNode, "isAirbone") ? true : false;
	//m_isOnFloor = xmlGetFloat(xmlNode, "isOnFloor") ? true : false;
	//m_isCrouched = xmlGetFloat(xmlNode, "isCrouched") ? true : false;
	//m_localFrame = xmlGetMatrix(xmlNode, "localFrame");
	//
	//SetMassMatrix(m_mass, GetCollisionShape());
	//m_invMass = GetInvMass();
	//m_contactPatch = m_radius / m_weistScale;
	//m_stepHeight = xmlGetFloat(xmlNode, "stepHeight");
	//m_headingAngle = xmlGetFloat(xmlNode, "headingAngle");
}

ndBodyPlayerCapsule::~ndBodyPlayerCapsule()
{
}

void ndBodyPlayerCapsule::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeHash, dInt32 nodeHash) const
{
	dAssert(0);
	//nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodyPlayerCapsule", nodeid);
	//ndBodyKinematic::Save(paramNode, assetPath, nodeid, shapesCache);
	//
	//xmlSaveParam(paramNode, "localFrame", m_localFrame);
	//xmlSaveParam(paramNode, "mass", m_mass);
	//xmlSaveParam(paramNode, "m_raheight", m_height);
	//xmlSaveParam(paramNode, "height", m_height);
	//xmlSaveParam(paramNode, "radius", m_radius);
	//xmlSaveParam(paramNode, "headingAngle", m_headingAngle);
	//xmlSaveParam(paramNode, "stepHeight", m_stepHeight);
	//xmlSaveParam(paramNode, "weistScale", m_weistScale);
	//xmlSaveParam(paramNode, "crouchScale", m_crouchScale);
	//xmlSaveParam(paramNode, "isAirbone", m_isAirbone ? 1 : 0);
	//xmlSaveParam(paramNode, "isOnFloor", m_isOnFloor ? 1 : 0);
	//xmlSaveParam(paramNode, "isCrouched", m_isCrouched ? 1 : 0);
}

void ndBodyPlayerCapsule::ResolveStep(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep)
{
	dMatrix matrix(m_matrix);
	dVector savedVeloc(m_veloc);

	dMatrix startMatrix(matrix);
	ndBodyPlayerCapsuleImpulseSolver impulseSolver(this);

	bool hasStartMatrix = false;
	const dFloat32 invTimeStep = dFloat32 (1.0f) / timestep;
	for (dInt32 j = 0; !hasStartMatrix && (j < 4); j++) 
	{
		hasStartMatrix = true;
		contactSolver.CalculateContacts();
		dInt32 count = contactSolver.m_contactCount;
		for (dInt32 i = count - 1; i >= 0; i--) 
		{
			const dVector& point = contactSolver.m_contactBuffer[i].m_point;
			const dVector localPointpoint(m_localFrame.UntransformVector(startMatrix.UntransformVector(point)));
			if (localPointpoint.m_x < m_stepHeight) 
			{
				count--;
				contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
			}
		}
		
		if (count) 
		{
			hasStartMatrix = false;
		
			SetVelocity(dVector::m_zero);
			impulseSolver.Reset(this);

			dVector com(startMatrix.TransformVector(GetCentreOfMass()));
			for (dInt32 i = 0; i < count; i++) 
			{
				dVector point(contactSolver.m_contactBuffer[i].m_point);
				dVector normal(contactSolver.m_contactBuffer[i].m_normal);
				dFloat32 speed = dMin((contactSolver.m_contactBuffer[i].m_penetration + D_MAX_COLLISION_PENETRATION), dFloat32(0.25f)) * invTimeStep;
				impulseSolver.AddLinearRow(normal, point - com, speed, 0.0f, 1.0e12f);
			}

			impulseSolver.AddAngularRows();
			dVector veloc(impulseSolver.CalculateImpulse().Scale(m_invMass));
			SetVelocity(veloc);
			ndBodyKinematic::IntegrateVelocity(timestep);
			startMatrix = GetMatrix();
		}
	}

	if (hasStartMatrix) 
	{
		// clip player velocity along the high contacts
		dMatrix coodinateMatrix(m_localFrame * startMatrix);
		dFloat32 scaleSpeedFactor = 1.5f;
		dFloat32 forwardSpeed = m_forwardSpeed * scaleSpeedFactor;
		dFloat32 lateralSpeed = m_lateralSpeed * scaleSpeedFactor;
		dFloat32 maxSpeed = dMax(dAbs(forwardSpeed), dAbs(lateralSpeed));
		dFloat32 stepFriction = 1.0f + m_mass * maxSpeed;
		
		SetVelocity(savedVeloc);
		impulseSolver.Reset(this);
		dInt32 index = impulseSolver.AddLinearRow(coodinateMatrix[0], dVector::m_zero, dFloat32 (0.0f), dFloat32(0.0f), dFloat32(1.0e12f));
		impulseSolver.AddLinearRow(coodinateMatrix[1], dVector::m_zero, -forwardSpeed, -stepFriction, stepFriction, index);
		impulseSolver.AddLinearRow(coodinateMatrix[2], dVector::m_zero, lateralSpeed, -stepFriction, stepFriction, index);
		dVector veloc(savedVeloc + impulseSolver.CalculateImpulse().Scale(m_invMass));
		
		bool advanceIsBlocked = true;
		for (dInt32 j = 0; advanceIsBlocked && (j < 4); j++) 
		{
			advanceIsBlocked = false;
			SetVelocity(veloc);
			ndBodyKinematic::IntegrateVelocity(timestep);
			contactSolver.CalculateContacts();
			if (contactSolver.m_contactCount) 
			{
				dMatrix stepMatrix (GetMatrix());
				dInt32 count = contactSolver.m_contactCount;

				// filter by position
				for (dInt32 i = count - 1; i >= 0; i--) 
				{
					const dVector& point = contactSolver.m_contactBuffer[i].m_point;
					dVector localPointpoint(m_localFrame.UntransformVector(stepMatrix.UntransformVector(point)));
					if (localPointpoint.m_x < m_stepHeight) 
					{
						count--;
						contactSolver.m_contactBuffer[i] = contactSolver.m_contactBuffer[count];
					}
				}
				
				if (count) 
				{
					dVector com(stepMatrix.TransformVector(GetCentreOfMass()));
					advanceIsBlocked = true;
					
					impulseSolver.Reset(this);
					for (dInt32 i = 0; i < count; i++) 
					{
						dVector point(contactSolver.m_contactBuffer[i].m_point);
						dVector normal(contactSolver.m_contactBuffer[i].m_normal);
						impulseSolver.AddLinearRow(normal, point - com, 0.0f, 0.0f, 1.0e12f);
					}
					
					impulseSolver.AddAngularRows();
					veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
					SetMatrix(startMatrix);
				}
			}	
		}
		
		SetVelocity(veloc);
		SetMatrix(startMatrix);
		ndBodyKinematic::IntegrateVelocity(timestep);
		contactSolver.CalculateContacts();
		if (contactSolver.m_contactCount) 
		{
			dMatrix stepMatrix(GetMatrix());
			
			dFloat32 maxHigh = 0.0f;
			for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
			{
				dVector point (contactSolver.m_contactBuffer[i].m_point);
				point = m_localFrame.UntransformVector(stepMatrix.UntransformVector(point));
				if ((point.m_x < m_stepHeight) && (point.m_x > m_contactPatch)) 
				{
					dVector normal(contactSolver.m_contactBuffer[i].m_normal);
					dFloat32 relSpeed = normal.DotProduct(veloc).GetScalar();
					if (relSpeed < dFloat32(-1.0e-2f)) 
					{
						maxHigh = dMax(point.m_x, maxHigh);
					}
				}
			}
			
			if (maxHigh > 0.0f) 
			{
				dVector step(stepMatrix.RotateVector(m_localFrame.RotateVector(dVector(maxHigh, dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)))));
				matrix.m_posit += step;
			}
		}
	}
	
	SetVelocity(savedVeloc);
	SetMatrix(matrix);
}

void ndBodyPlayerCapsule::UpdatePlayerStatus(ndBodyPlayerCapsuleContactSolver& contactSolver)
{
	m_isAirbone = true;
	m_isOnFloor = false;
	contactSolver.CalculateContacts();
	dMatrix matrix(m_localFrame * GetMatrix());
	for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
	{
		m_isAirbone = false;
		const ndContactPoint* const contact = &contactSolver.m_contactBuffer[i];
		dVector point(contact->m_point);
		dVector localPoint(matrix.UntransformVector(point));
		if (localPoint.m_x < m_contactPatch) 
		{
			dVector normal(contact->m_normal);
			dVector localNormal(matrix.UnrotateVector(normal));
			//if (localNormal.m_x > dFloat32 (0.95f)) 
			if (localNormal.m_x > D_SLOP_JUMP_ANGLE)
			{
				m_isOnFloor = true;
			}
		}
	}

	if (m_isAirbone || !m_isOnFloor)
	{
		m_equilibriumOverride = 1;
	}
}

ndBodyPlayerCapsule::dCollisionState ndBodyPlayerCapsule::TestPredictCollision(const ndBodyPlayerCapsuleContactSolver& contactSolver, const dVector& veloc) const
{
	for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
	{
		const ndContactPoint* const contact = &contactSolver.m_contactBuffer[i];
		if (contact->m_penetration >= D_MAX_COLLISION_PENETRATION) 
		{
			return m_deepPenetration;
		}
	}

	for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
	{
		const ndContactPoint* const contact = &contactSolver.m_contactBuffer[i];
		dFloat32 projecSpeed = veloc.DotProduct(contact->m_normal).GetScalar();
		if (projecSpeed < dFloat32(0.0f)) 
		{
			return m_colliding;
		}
	}
	return m_freeMovement;
}

dFloat32 ndBodyPlayerCapsule::PredictTimestep(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep)
{
	dMatrix matrix(m_matrix);
	dVector veloc(GetVelocity());
	ndBodyKinematic::IntegrateVelocity(timestep);
	dCollisionState playerCollide = TestPredictCollision(contactSolver, veloc);
	SetMatrix(matrix);

	if (playerCollide == m_deepPenetration) 
	{
		dFloat32 savedTimeStep = timestep;
		timestep *= 0.5f;
		dFloat32 dt = timestep;
		for (dInt32 i = 0; i < D_MAX_COLLIONSION_STEPS; i++) 
		{
			ndBodyKinematic::IntegrateVelocity(timestep);
			contactSolver.CalculateContacts();
			SetMatrix(matrix);
		
			dt *= 0.5f;
			playerCollide = TestPredictCollision(contactSolver, veloc);
			if (playerCollide == m_colliding) 
			{
				return timestep;
			}
			if (playerCollide == m_deepPenetration) 
			{
				timestep -= dt;
			}
			else 
			{
				timestep += dt;
			}
		}
		if (timestep > dt * 2.0f) 
		{
			return timestep;
		}
		
		dt = savedTimeStep / D_MAX_COLLIONSION_STEPS;
		timestep = dt;
		for (dInt32 i = 1; i < D_MAX_COLLIONSION_STEPS; i++) 
		{
			ndBodyKinematic::IntegrateVelocity(timestep);
			contactSolver.CalculateContacts();
			SetMatrix(matrix);
			playerCollide = TestPredictCollision(contactSolver, veloc);
			if (playerCollide != m_freeMovement) 
			{
				return timestep;
			}
			timestep += dt;
		}
		dAssert(0);
	}

	return timestep;
}

void ndBodyPlayerCapsule::ResolveInterpenetrations(ndBodyPlayerCapsuleContactSolver& contactSolver, ndBodyPlayerCapsuleImpulseSolver& impulseSolver)
{
	dVector savedVeloc(GetVelocity());
	
	dFloat32 timestep = dFloat32 (0.1f);
	dFloat32 invTimestep = dFloat32(1.0f) / timestep;
	
	dFloat32 penetration = D_MAX_COLLISION_PENETRATION * 10.0f;
	for (dInt32 j = 0; (j < 8) && (penetration > D_MAX_COLLISION_PENETRATION); j++) 
	{
		SetVelocity(dVector::m_zero);
		dMatrix matrix(GetMatrix());
		dVector com(matrix.TransformVector(GetCentreOfMass()));
	
		impulseSolver.Reset(this);
		for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
		{
			ndContactPoint* const contact = &contactSolver.m_contactBuffer[i];
			dVector point(contact->m_point);
			dVector normal(contact->m_normal);
			dInt32 index = impulseSolver.AddContactRow(contact, normal, point - com, 0.0f, 0.0f, 1.0e12f);
	
			dFloat32 impulse = invTimestep * dClamp(contact->m_penetration - D_MAX_COLLISION_PENETRATION * 0.5f, dFloat32(0.0f), dFloat32(0.5f));
			impulseSolver.m_rhs[index] = impulse;
		}

		impulseSolver.AddAngularRows();
	
		dVector veloc(impulseSolver.CalculateImpulse().Scale(m_invMass));
		SetVelocity(veloc);
		ndBodyKinematic::IntegrateVelocity(timestep);
	
		penetration = 0.0f;
		contactSolver.CalculateContacts();
		for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
		{
			penetration = dMax(contactSolver.m_contactBuffer[i].m_penetration, penetration);
		}
	}
	
	SetVelocity(savedVeloc);
}

void ndBodyPlayerCapsule::ResolveCollision(ndBodyPlayerCapsuleContactSolver& contactSolver, dFloat32 timestep)
{
	contactSolver.CalculateContacts();
	if (!contactSolver.m_contactCount) 
	{
		return;
	}

	dFloat32 maxPenetration = 0.0f;
	for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
	{
		maxPenetration = dMax(contactSolver.m_contactBuffer[i].m_penetration, maxPenetration);
	}

	dMatrix matrix(GetMatrix());
	ndBodyPlayerCapsuleImpulseSolver impulseSolver(this);
	if (maxPenetration > D_MAX_COLLISION_PENETRATION) 
	{
		ResolveInterpenetrations(contactSolver, impulseSolver);
		matrix = GetMatrix();
	}

	dVector veloc (GetVelocity());
	const dMatrix frameMatrix(m_localFrame * matrix);
	dVector com(matrix.TransformVector(GetCentreOfMass()));

	impulseSolver.Reset(this);
	dVector surfaceVeloc(0.0f);
	const dFloat32 contactPatchHigh = m_contactPatch * dFloat32(0.995f);

	for (dInt32 i = 0; i < contactSolver.m_contactCount; i++) 
	{
		ndContactPoint* const contact = &contactSolver.m_contactBuffer[i];
		dVector point(contact->m_point);
		dVector normal(contact->m_normal);
		const dInt32 normalIndex = impulseSolver.AddContactRow(contact, normal, point - com, dFloat32 (0.0f), dFloat32(0.0f), dFloat32(1.0e12f));
		dVector localPoint(frameMatrix.UntransformVector(point));
		
		if (localPoint.m_x < contactPatchHigh) 
		{
			dFloat32 friction = ContactFrictionCallback(point, normal, 0, contact->m_body1);
			if (friction > 0.0f) 
			{
				// add lateral traction friction
				dVector sideDir(frameMatrix.m_up.CrossProduct(normal).Normalize());
				impulseSolver.AddContactRow(contact, sideDir, point - com, -m_lateralSpeed, -friction, friction, normalIndex);
		
				// add longitudinal  traction friction
				dVector frontDir(normal.CrossProduct(sideDir));
				impulseSolver.AddContactRow(contact, frontDir, point - com, -m_forwardSpeed, -friction, friction, normalIndex);
			}
		}
	}

	impulseSolver.AddAngularRows();

	veloc += impulseSolver.CalculateImpulse().Scale(m_invMass);
	impulseSolver.ApplyReaction(timestep);

	SetVelocity(veloc);
}

ndBodyPlayerCapsuleImpulseSolver::ndBodyPlayerCapsuleImpulseSolver(ndBodyPlayerCapsule* const controller)
{
	m_mass = controller->GetMassMatrix().m_w;
	m_invMass = controller->GetInvMass();
	m_invInertia = controller->GetInvInertiaMatrix();
	Reset(controller);
}

void ndBodyPlayerCapsuleImpulseSolver::Reset(ndBodyPlayerCapsule* const controller)
{
	m_rowCount = 0;
	m_veloc = controller->GetVelocity();
}

ndBodyPlayerCapsuleContactSolver::ndBodyPlayerCapsuleContactSolver(ndBodyPlayerCapsule* const player)
	:m_player(player)
	,m_contactCount(0)
{
}

void ndBodyPlayerCapsuleContactSolver::CalculateContacts()
{
	m_contactCount = 0;
	ndScene* const scene = m_player->GetScene();
	ndBodyKinematic::ndContactMap::Iterator it(m_player->GetContactMap());
	for (it.Begin(); it; it++)
	{
		const ndContact* const srcContact = *it;
		if (srcContact->IsActive())
		{
			ndBodyKinematic* body0 = srcContact->GetBody0();
			ndBodyKinematic* body1 = srcContact->GetBody1();
			if (body1 == m_player)
			{
				dSwap(body0, body1);
			}
			ndContact contact;
			contact.SetBodies(body0, body1);

			ndContactPoint contactBuffer[D_MAX_CONTATCS];
			//ndContactSolver contactSolver(&contact, scene);
			ndContactSolver contactSolver(&contact, scene->GetContactNotify(), dFloat32(1.0f));
			contactSolver.m_instance0.SetGlobalMatrix(contactSolver.m_instance0.GetLocalMatrix() * body0->GetMatrix());
			contactSolver.m_instance1.SetGlobalMatrix(contactSolver.m_instance1.GetLocalMatrix() * body1->GetMatrix());
			contactSolver.m_separatingVector = srcContact->m_separatingVector;
			//contactSolver.m_timestep = dFloat32 (1.0f);
			//contactSolver.m_ccdMode = 0;
			contactSolver.m_intersectionTestOnly = 0;
			contactSolver.m_contactBuffer = contactBuffer;
			const dInt32 count = contactSolver.CalculateContactsDiscrete ();
			for (dInt32 i = 0; i < count; i++)
			{
				m_contactBuffer[m_contactCount] = contactBuffer[i];
				m_contactCount++;
				dAssert(m_contactCount < dInt32 (sizeof(m_contactBuffer) / sizeof(m_contactBuffer[0])));
			}
		}
	}
}

dInt32 ndBodyPlayerCapsuleImpulseSolver::AddLinearRow(const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex)
{
	m_contactPoint[m_rowCount] = nullptr;
	m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear = dir;
	m_jacobianPairs[m_rowCount].m_jacobianM0.m_angular = r.CrossProduct(dir);
	m_jacobianPairs[m_rowCount].m_jacobianM1.m_linear = dVector::m_zero;
	m_jacobianPairs[m_rowCount].m_jacobianM1.m_angular = dVector::m_zero;

	m_low[m_rowCount] = low;
	m_high[m_rowCount] = high;
	m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;
	m_rhs[m_rowCount] = speed - m_veloc.DotProduct(m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear).GetScalar();
	m_rowCount++;
	dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	return m_rowCount - 1;
}

dVector ndBodyPlayerCapsuleImpulseSolver::CalculateImpulse()
{
	dFloat32 massMatrix[D_PLAYER_MAX_ROWS][D_PLAYER_MAX_ROWS];
	const ndBodyKinematic* bodyArray[D_PLAYER_MAX_ROWS];
	for (dInt32 i = 0; i < m_rowCount; i++) 
	{
		bodyArray[i] = m_contactPoint[i] ? m_contactPoint[i]->m_body0 : nullptr;
	}

	for (dInt32 i = 0; i < m_rowCount; i++) 
	{
		ndJacobianPair jInvMass(m_jacobianPairs[i]);

		jInvMass.m_jacobianM0.m_linear = jInvMass.m_jacobianM0.m_linear.Scale(m_invMass);
		jInvMass.m_jacobianM0.m_angular = m_invInertia.RotateVector(jInvMass.m_jacobianM0.m_angular);
		if (bodyArray[i]) 
		{
			dFloat32 invMass = bodyArray[i]->GetInvMass();
			dMatrix invInertia(bodyArray[i]->GetInvInertiaMatrix());
			jInvMass.m_jacobianM1.m_linear = jInvMass.m_jacobianM1.m_linear.Scale(invMass);
			jInvMass.m_jacobianM1.m_angular = invInertia.RotateVector(jInvMass.m_jacobianM1.m_angular);
		}
		else 
		{
			jInvMass.m_jacobianM1.m_linear = dVector::m_zero;
			jInvMass.m_jacobianM1.m_angular = dVector::m_zero;
		}

		dVector tmp(
			jInvMass.m_jacobianM0.m_linear * m_jacobianPairs[i].m_jacobianM0.m_linear +
			jInvMass.m_jacobianM0.m_angular * m_jacobianPairs[i].m_jacobianM0.m_angular +
			jInvMass.m_jacobianM1.m_linear * m_jacobianPairs[i].m_jacobianM1.m_linear +
			jInvMass.m_jacobianM1.m_angular * m_jacobianPairs[i].m_jacobianM1.m_angular);
		dFloat32 a00 = (tmp.m_x + tmp.m_y + tmp.m_z) * dFloat32 (1.0004f);

		massMatrix[i][i] = a00;

		m_impulseMag[i] = 0.0f;
		for (dInt32 j = i + 1; j < m_rowCount; j++) 
		{
			dVector tmp1(
				jInvMass.m_jacobianM0.m_linear * m_jacobianPairs[j].m_jacobianM0.m_linear +
				jInvMass.m_jacobianM0.m_angular * m_jacobianPairs[j].m_jacobianM0.m_angular);
			if (bodyArray[i] == bodyArray[j]) 
			{
				tmp1 += jInvMass.m_jacobianM1.m_linear * m_jacobianPairs[j].m_jacobianM1.m_linear;
				tmp1 += jInvMass.m_jacobianM1.m_angular * m_jacobianPairs[j].m_jacobianM1.m_angular;
			}

			dFloat32 a01 = tmp1.m_x + tmp1.m_y + tmp1.m_z;
			massMatrix[i][j] = a01;
			massMatrix[j][i] = a01;
		}
	}

	dAssert(dTestPSDmatrix(m_rowCount, D_PLAYER_MAX_ROWS, &massMatrix[0][0]));
	dGaussSeidelLcpSor(m_rowCount, D_PLAYER_MAX_ROWS, &massMatrix[0][0], m_impulseMag, m_rhs, m_normalIndex, m_low, m_high, dFloat32(1.0e-6f), 32, dFloat32(1.1f));

	dVector netImpulse(0.0f);
	for (dInt32 i = 0; i < m_rowCount; i++) 
	{
		netImpulse += m_jacobianPairs[i].m_jacobianM0.m_linear.Scale(m_impulseMag[i]);
	}
	return netImpulse;
}

dInt32 ndBodyPlayerCapsuleImpulseSolver::AddContactRow(const ndContactPoint* const contact, const dVector& dir, const dVector& r, dFloat32 speed, dFloat32 low, dFloat32 high, dInt32 normalIndex)
{
	dAssert(contact->m_body1);
	if (contact->m_body1->GetInvMass() == dFloat32 (0.0f)) 
	{
		return AddLinearRow(dir, r, speed, low, high, normalIndex);
	}
	
	dVector omega(contact->m_body1->GetOmega());
	dVector veloc(contact->m_body1->GetVelocity());
	dMatrix matrix(contact->m_body1->GetMatrix());
	dVector com(matrix.TransformVector(contact->m_body1->GetCentreOfMass()));
	
	dVector p1(contact->m_point);
	dVector r1(p1 - com);
	dVector dir1(dir.Scale(-1.0f));
	
	m_contactPoint[m_rowCount] = (ndContactPoint*)contact;
	m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear = dir;
	m_jacobianPairs[m_rowCount].m_jacobianM0.m_angular = r.CrossProduct(dir);
	m_jacobianPairs[m_rowCount].m_jacobianM1.m_linear = dir1;
	m_jacobianPairs[m_rowCount].m_jacobianM1.m_angular = r1.CrossProduct(dir1);
	
	m_low[m_rowCount] = low;
	m_high[m_rowCount] = high;
	m_normalIndex[m_rowCount] = (normalIndex == -1) ? 0 : normalIndex - m_rowCount;
	
	dVector reactionSpeed(
		m_veloc * m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear +
		veloc * m_jacobianPairs[m_rowCount].m_jacobianM1.m_linear +
		omega * m_jacobianPairs[m_rowCount].m_jacobianM1.m_angular);
	m_rhs[m_rowCount] = speed - reactionSpeed.AddHorizontal().GetScalar();
	
	m_rowCount++;
	dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	return m_rowCount - 1;
}

void ndBodyPlayerCapsuleImpulseSolver::AddAngularRows()
{
	for (dInt32 i = 0; i < 3; i++) 
	{
		m_contactPoint[m_rowCount] = nullptr;
		m_jacobianPairs[m_rowCount].m_jacobianM1.m_linear = dVector::m_zero;
		m_jacobianPairs[m_rowCount].m_jacobianM1.m_angular = dVector::m_zero;
		m_jacobianPairs[m_rowCount].m_jacobianM0.m_linear = dVector::m_zero;
		m_jacobianPairs[m_rowCount].m_jacobianM0.m_angular = dVector::m_zero;
		m_jacobianPairs[m_rowCount].m_jacobianM0.m_angular[i] = dFloat32(1.0f);
		m_rhs[m_rowCount] = dFloat32(0.0f);
		m_low[m_rowCount] = dFloat32(-1.0e12f);
		m_high[m_rowCount] = dFloat32(1.0e12f);
		m_impulseMag[m_rowCount] = dFloat32(0.0f);
		m_normalIndex[m_rowCount] = 0;
		m_rowCount++;
		dAssert(m_rowCount < D_PLAYER_MAX_ROWS);
	}
}

void ndBodyPlayerCapsuleImpulseSolver::ApplyReaction(dFloat32 timestep)
{
	dFloat32 invTimeStep = 0.1f / timestep;
	for (dInt32 i = 0; i < m_rowCount; i++) 
	{
		if (m_contactPoint[i]) 
		{
			ndBodyKinematic* const body0 = ((ndBodyKinematic*)m_contactPoint[i]->m_body0);
			ndBodyKinematic* const body1 = ((ndBodyKinematic*)m_contactPoint[i]->m_body1);
			dVector force(m_jacobianPairs[i].m_jacobianM1.m_linear.Scale(m_impulseMag[i] * invTimeStep));
			dVector torque(m_jacobianPairs[i].m_jacobianM1.m_angular.Scale(m_impulseMag[i] * invTimeStep));
			body1->SetForce(force + body1->GetForce());
			body1->SetTorque(torque + body1->GetTorque());
			body0->m_equilibriumOverride = 1;
		}
	}
}

void ndBodyPlayerCapsule::IntegrateExternalForce(dFloat32 timestep)
{
	ndBodyPlayerCapsuleContactSolver contactSolver(this);
	dFloat32 timeLeft = timestep;
	const dFloat32 timeEpsilon = timestep * (1.0f / 16.0f);

	m_equilibriumOverride = 0;
	m_impulse = dVector::m_zero;
	ApplyInputs(timestep);

#if 0
	#if 1
		static FILE* file = fopen("log.bin", "wb");
		if (file) {
			fwrite(&m_headingAngle, sizeof(m_headingAngle), 1, file);
			fwrite(&m_forwardSpeed, sizeof(m_forwardSpeed), 1, file);
			fwrite(&m_lateralSpeed, sizeof(m_lateralSpeed), 1, file);
			fflush(file);
		}
	#else 
		static FILE* file = fopen("log.bin", "rb");
		if (file) {
			fread(&m_headingAngle, sizeof(m_headingAngle), 1, file);
			fread(&m_forwardSpeed, sizeof(m_forwardSpeed), 1, file);
			fread(&m_lateralSpeed, sizeof(m_lateralSpeed), 1, file);
		}
	#endif
#endif

	// set player orientation
	dMatrix matrix(dYawMatrix(GetHeadingAngle()));
	matrix.m_posit = m_matrix.m_posit;
	SetMatrix(matrix);

	// set play desired velocity
	dVector veloc(GetVelocity() + m_impulse.Scale(m_invMass));
	SetVelocity(veloc);

	// determine if player has to step over obstacles lower than step hight
	ResolveStep(contactSolver, timestep);

	// advance player until it hit a collision point, until there is not more time left
	for (dInt32 i = 0; (i < D_DESCRETE_MOTION_STEPS) && (timeLeft > timeEpsilon); i++)
	{
		if (timeLeft > timeEpsilon)
		{
			ResolveCollision(contactSolver, timestep);
		}

		dFloat32 predicetdTime = PredictTimestep(contactSolver, timeLeft);
		ndBodyKinematic::IntegrateVelocity(predicetdTime);
		timeLeft -= predicetdTime;
	}

	UpdatePlayerStatus(contactSolver);
}

