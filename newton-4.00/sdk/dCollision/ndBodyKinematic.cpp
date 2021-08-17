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
#include "ndContact.h"
#include "ndShapeNull.h"
#include "ndRayCastNotify.h"
#include "ndBodyKinematic.h"
#include "ndShapeCompound.h"
#include "ndJointBilateralConstraint.h"

#define D_MINIMUM_MASS	dFloat32(1.0e-5f)
#define D_INFINITE_MASS	dFloat32(1.0e15f)

class ndDummyCollision: public ndShapeNull
{
	public:
	ndDummyCollision()
		:ndShapeNull()
	{
		m_refCount.fetch_add(1);
	}

	~ndDummyCollision()
	{
		m_refCount.fetch_add(-1);
	}

	static ndShapeNull* GetNullShape()
	{
		static ndDummyCollision nullShape;
		return &nullShape;
	}
};


ndBodyKinematic::ndContactMap::ndContactMap()
	:dTree<ndContact*, ndContactkey, dContainersFreeListAlloc<ndContact*>>()
{
}

ndContact* ndBodyKinematic::ndContactMap::FindContact(const ndBody* const body0, const ndBody* const body1) const
{
	ndContactkey key(body0->GetId(), body1->GetId());
	dNode* const node = Find(key);
	return node ? node->GetInfo() : nullptr;
}

void ndBodyKinematic::ndContactMap::AttachContact(ndContact* const contact)
{
	ndBody* const body0 = contact->GetBody0();
	ndBody* const body1 = contact->GetBody1();
	ndContactkey key(body0->GetId(), body1->GetId());
	dAssert(!Find(key));
	Insert(contact, key);
}

void ndBodyKinematic::ndContactMap::DetachContact(ndContact* const contact)
{
	ndBody* const body0 = contact->GetBody0();
	ndBody* const body1 = contact->GetBody1();
	ndContactkey key(body0->GetId(), body1->GetId());
	dAssert(Find(key));
	Remove(key);
}

ndBodyKinematic::ndBodyKinematic()
	:ndBody()
	,m_invWorldInertiaMatrix(dGetZeroMatrix())
	,m_shapeInstance(ndDummyCollision::GetNullShape())
	,m_mass(dVector::m_zero)
	,m_invMass(dVector::m_zero)
	,m_residualVeloc(dVector::m_zero)
	,m_residualOmega(dVector::m_zero)
	,m_gyroAlpha(dVector::m_zero)
	,m_gyroTorque(dVector::m_zero)
	,m_gyroRotation()
	,m_jointList()
	,m_contactList()
	,m_lock()
	,m_scene(nullptr)
	,m_islandParent(nullptr)
	,m_sceneNode(nullptr)
	,m_sceneBodyBodyNode(nullptr)
	,m_skeletonContainer(nullptr)
	,m_maxAngleStep(dFloat32 (90.0f) * dDegreeToRad)
	,m_maxLinearSpeed(dFloat32 (100.0f))
	,m_weigh(dFloat32 (0.0f))
	,m_rank(0)
	,m_index(0)
	,m_sleepingCounter(0)
{
	m_invWorldInertiaMatrix[3][3] = dFloat32(1.0f);
	m_shapeInstance.m_ownerBody = this;
	SetMassMatrix(dVector::m_zero);
}

ndBodyKinematic::ndBodyKinematic(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache)
	:ndBody(xmlNode->FirstChild("ndBody"), shapesCache)
	,m_invWorldInertiaMatrix(dGetZeroMatrix())
	,m_shapeInstance(ndDummyCollision::GetNullShape())
	,m_mass(dVector::m_zero)
	,m_invMass(dVector::m_zero)
	,m_residualVeloc(dVector::m_zero)
	,m_residualOmega(dVector::m_zero)
	,m_gyroAlpha(dVector::m_zero)
	,m_gyroTorque(dVector::m_zero)
	,m_gyroRotation()
	,m_jointList()
	,m_contactList()
	,m_lock()
	,m_scene(nullptr)
	,m_islandParent(nullptr)
	,m_sceneNode(nullptr)
	,m_sceneBodyBodyNode(nullptr)
	,m_skeletonContainer(nullptr)
	,m_weigh(dFloat32(0.0f))
	,m_rank(0)
	,m_index(0)
	,m_sleepingCounter(0)
{
	m_invWorldInertiaMatrix[3][3] = dFloat32(1.0f);
	ndShapeInstance instance(xmlNode->FirstChild("ndShapeInstance"), shapesCache);
	SetCollisionShape(instance);

	dFloat32 invMass = xmlGetFloat(xmlNode, "invMass");
	SetMassMatrix(dVector::m_zero);
	if (invMass > dFloat32 (0.0f))
	{
		dVector invInertia(xmlGetVector3(xmlNode, "invPrincipalInertia"));
		invInertia.m_w = dFloat32 (1.0f);
		dVector mass (invInertia.Scale(dFloat32(1.0f) / invMass));
		SetMassMatrix(mass.m_x, mass.m_y, mass.m_z, mass.m_w);
	}
}

ndBodyKinematic::~ndBodyKinematic()
{
	dAssert(m_scene == nullptr);
	dAssert(m_sceneNode == nullptr);
}

void ndBodyKinematic::SetSleepState(bool state)
{
	m_equilibrium = state ? 1 : 0;
	if ((m_invMass.m_w > dFloat32(0.0f)) && (m_veloc.DotProduct(m_veloc).GetScalar() < dFloat32(1.0e-10f)) && (m_omega.DotProduct(m_omega).GetScalar() < dFloat32(1.0e-10f))) 
	{
		dVector invalidateVeloc(dFloat32(10.0f));
		ndContactMap::Iterator it(m_contactList);
		for (it.Begin(); it; it ++)
		{
			ndContact* const contactJoint = *it;
			contactJoint->m_positAcc = invalidateVeloc;
		}
	}
}

void ndBodyKinematic::SetCollisionShape(const ndShapeInstance& shapeInstance)
{
	m_shapeInstance = shapeInstance;
	m_shapeInstance.m_ownerBody = this;
	if (m_shapeInstance.GetShape()->GetAsShapeCompound())
	{
		m_shapeInstance.GetShape()->GetAsShapeCompound()->SetSubShapeOwner(this);
	}
}

void ndBodyKinematic::ReleaseMemory()
{
	ndContactMap::FlushFreeList();
}

ndContact* ndBodyKinematic::FindContact(const ndBody* const otherBody) const
{
#ifndef D_USE_GLOBAL_LOCK
	dScopeSpinLock lock(m_lock);
#endif
	return m_contactList.FindContact(this, otherBody);
}

void ndBodyKinematic::AttachContact(ndContact* const contact)
{
#ifndef D_USE_GLOBAL_LOCK
	dScopeSpinLock lock(m_lock);
#endif
	dAssert((this == contact->GetBody0()) || (this == contact->GetBody1()));
	if (m_invMass.m_w > dFloat32(0.0f))
	{
		m_equilibrium = 0;
	}

	m_contactList.AttachContact(contact);
}

void ndBodyKinematic::DetachContact(ndContact* const contact)
{
#ifndef D_USE_GLOBAL_LOCK
	dScopeSpinLock lock(m_lock);
#endif
	dAssert((this == contact->GetBody0()) || (this == contact->GetBody1()));
	m_equilibrium = contact->m_body0->m_equilibrium & contact->m_body1->m_equilibrium;
	m_contactList.DetachContact(contact);
}

ndJointList::dNode* ndBodyKinematic::AttachJoint(ndJointBilateralConstraint* const joint)
{
	m_equilibrium = 0;
	return m_jointList.Append(joint);
}

void ndBodyKinematic::DetachJoint(ndJointList::dNode* const node)
{
	m_equilibrium = 0;
#ifdef _DEBUG
	bool found = false;
	for (ndJointList::dNode* nodeptr = m_jointList.GetFirst(); nodeptr; nodeptr = nodeptr->GetNext())
	{
		found = found || nodeptr;
	}
	dAssert(found);
#endif
	m_jointList.Remove(node);
}

void ndBodyKinematic::SetMassMatrix(dFloat32 mass, const dMatrix& inertia)
{
	mass = dAbs(mass);

	ndShape* const shape = m_shapeInstance.GetShape();
	//if ((mass < D_MINIMUM_MASS) || shape->GetAsShapeNull() || !shape->GetAsShapeConvex())
	if ((mass < D_MINIMUM_MASS) || shape->GetAsShapeNull() || shape->GetAsShapeStaticMesh())
	{
		mass = D_INFINITE_MASS * 2.0f;
	}

	if (mass >= D_INFINITE_MASS)
	{
		m_mass.m_x = D_INFINITE_MASS;
		m_mass.m_y = D_INFINITE_MASS;
		m_mass.m_z = D_INFINITE_MASS;
		m_mass.m_w = D_INFINITE_MASS;
		m_invMass = dVector::m_zero;
	}
	else
	{
		dFloat32 Ixx = dAbs(inertia[0][0]);
		dFloat32 Iyy = dAbs(inertia[1][1]);
		dFloat32 Izz = dAbs(inertia[2][2]);

		dFloat32 Ixx1 = dClamp(Ixx, dFloat32(0.0001f) * mass, dFloat32(10000.0f) * mass);
		dFloat32 Iyy1 = dClamp(Iyy, dFloat32(0.0001f) * mass, dFloat32(10000.0f) * mass);
		dFloat32 Izz1 = dClamp(Izz, dFloat32(0.0001f) * mass, dFloat32(10000.0f) * mass);

		dAssert(Ixx1 > dFloat32(0.0f));
		dAssert(Iyy1 > dFloat32(0.0f));
		dAssert(Izz1 > dFloat32(0.0f));

		m_mass.m_x = Ixx1;
		m_mass.m_y = Iyy1;
		m_mass.m_z = Izz1;
		m_mass.m_w = mass;

		m_invMass.m_x = dFloat32(1.0f) / Ixx1;
		m_invMass.m_y = dFloat32(1.0f) / Iyy1;
		m_invMass.m_z = dFloat32(1.0f) / Izz1;
		m_invMass.m_w = dFloat32(1.0f) / mass;
	}

	//#ifdef _DEBUG
#if 0
	dgBodyMasterList& me = *m_world;
	for (dgBodyMasterList::dNode* refNode = me.GetFirst(); refNode; refNode = refNode->GetNext()) {
		dgBody* const body0 = refNode->GetInfo().GetBody();
		dVector invMass(body0->GetInvMass());
		if (invMass.m_w != 0.0f) {
			for (; refNode; refNode = refNode->GetNext()) {
				dgBody* const body1 = refNode->GetInfo().GetBody();
				dVector invMass1(body1->GetInvMass());
				dAssert(invMass1.m_w != 0.0f);
			}
			break;
		}
	}
#endif
}

bool ndBodyKinematic::RayCast(ndRayCastNotify& callback, const dFastRay& ray, dFloat32 maxT) const
{
	dVector l0(ray.m_p0);
	dVector l1(ray.m_p0 + ray.m_diff.Scale(dMin(maxT, dFloat32(1.0f))));

	bool state = false;
	if (dRayBoxClip(l0, l1, m_minAabb, m_maxAabb))
	{
		const dMatrix& globalMatrix = m_shapeInstance.GetGlobalMatrix();
		dVector localP0(globalMatrix.UntransformVector(l0) & dVector::m_triplexMask);
		dVector localP1(globalMatrix.UntransformVector(l1) & dVector::m_triplexMask);
		dVector p1p0(localP1 - localP0);
		dAssert(p1p0.m_w == dFloat32(0.0f));
		if (p1p0.DotProduct(p1p0).GetScalar() > dFloat32(1.0e-12f))
		{
			if (m_shapeInstance.GetCollisionMode())
			{
				ndContactPoint contactOut;
				dFloat32 t = m_shapeInstance.RayCast(callback, localP0, localP1, this, contactOut);
				if (t < dFloat32(1.0f))
				{
					dAssert(localP0.m_w == localP1.m_w);
					dVector p(globalMatrix.TransformVector(localP0 + (localP1 - localP0).Scale(t)));
					t = ray.m_diff.DotProduct(p - ray.m_p0).GetScalar() / ray.m_diff.DotProduct(ray.m_diff).GetScalar();
					if (t < maxT)
					{
						dAssert(t >= dFloat32(0.0f));
						dAssert(t <= dFloat32(1.0f));
						contactOut.m_body0 = this;
						contactOut.m_body1 = this;
						contactOut.m_point = p;
						contactOut.m_normal = globalMatrix.RotateVector(contactOut.m_normal);
						state = callback.OnRayCastAction(contactOut, t) < dFloat32 (1.0f);
					}
				}
			}
		}
	}
	return state;
}

void ndBodyKinematic::UpdateCollisionMatrix()
{
	m_transformIsDirty = 1;
	m_shapeInstance.SetGlobalMatrix(m_shapeInstance.GetLocalMatrix() * m_matrix);
	m_shapeInstance.CalculateAabb(m_shapeInstance.GetGlobalMatrix(), m_minAabb, m_maxAabb);
}

dMatrix ndBodyKinematic::CalculateInvInertiaMatrix() const
{
	const dVector invIxx(m_invMass[0]);
	const dVector invIyy(m_invMass[1]);
	const dVector invIzz(m_invMass[2]);
	return dMatrix(
		m_matrix.m_front.Scale(m_matrix.m_front[0]) * invIxx +
		m_matrix.m_up.Scale(m_matrix.m_up[0])		* invIyy +
		m_matrix.m_right.Scale(m_matrix.m_right[0]) * invIzz,

		m_matrix.m_front.Scale(m_matrix.m_front[1]) * invIxx +
		m_matrix.m_up.Scale(m_matrix.m_up[1])		* invIyy +
		m_matrix.m_right.Scale(m_matrix.m_right[1]) * invIzz,

		m_matrix.m_front.Scale(m_matrix.m_front[2]) * invIxx +
		m_matrix.m_up.Scale(m_matrix.m_up[2])		* invIyy +
		m_matrix.m_right.Scale(m_matrix.m_right[2]) * invIzz,
		dVector::m_wOne);
}

dMatrix ndBodyKinematic::CalculateInertiaMatrix() const
{
	const dVector Ixx(m_mass[0]);
	const dVector Iyy(m_mass[1]);
	const dVector Izz(m_mass[2]);
	return dMatrix(
		m_matrix.m_front.Scale(m_matrix.m_front[0]) * Ixx +
		m_matrix.m_up.Scale(m_matrix.m_up[0])		* Iyy +
		m_matrix.m_right.Scale(m_matrix.m_right[0]) * Izz,

		m_matrix.m_front.Scale(m_matrix.m_front[1]) * Ixx +
		m_matrix.m_up.Scale(m_matrix.m_up[1])       * Iyy +
		m_matrix.m_right.Scale(m_matrix.m_right[1]) * Izz,

		m_matrix.m_front.Scale(m_matrix.m_front[2]) * Ixx +
		m_matrix.m_up.Scale(m_matrix.m_up[2])       * Iyy +
		m_matrix.m_right.Scale(m_matrix.m_right[2]) * Izz,
		dVector::m_wOne);

}

dVector ndBodyKinematic::CalculateLinearMomentum() const
{
	return m_veloc.Scale(m_mass.m_w);
}

dVector ndBodyKinematic::CalculateAngularMomentum() const
{
	const dVector localOmega(m_matrix.UnrotateVector(m_omega));
	const dVector localAngularMomentum(m_mass * localOmega);
	return m_matrix.RotateVector(localAngularMomentum);
}

dFloat32 ndBodyKinematic::TotalEnergy() const
{
	dVector energy (m_veloc.DotProduct(CalculateLinearMomentum()) + m_veloc.DotProduct(CalculateAngularMomentum()));
	return energy.AddHorizontal().GetScalar()* dFloat32(0.5f);
}

void ndBodyKinematic::UpdateInvInertiaMatrix()
{
	dAssert(m_invWorldInertiaMatrix[0][3] == dFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[1][3] == dFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[2][3] == dFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[3][3] == dFloat32(1.0f));

	m_invWorldInertiaMatrix = CalculateInvInertiaMatrix();

	dAssert(m_invWorldInertiaMatrix[0][3] == dFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[1][3] == dFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[2][3] == dFloat32(0.0f));
	dAssert(m_invWorldInertiaMatrix[3][3] == dFloat32(1.0f));
}

void ndBodyKinematic::IntegrateVelocity(dFloat32 timestep)
{
	dAssert(m_veloc.m_w == dFloat32(0.0f));
	dAssert(m_omega.m_w == dFloat32(0.0f));
	m_globalCentreOfMass += m_veloc.Scale(timestep);
	dFloat32 omegaMag2 = m_omega.DotProduct(m_omega).GetScalar();
#ifdef _DEBUG
	const dFloat32 err2 = m_maxAngleStep * m_maxAngleStep;
	const dFloat32 step2 = omegaMag2 * timestep * timestep;
	const dFloat32 speed2 = m_veloc.DotProduct(m_veloc).GetScalar() * timestep * timestep;;
	if ((step2 > err2) || (speed2 > m_maxLinearSpeed))
	{
		dTrace(("warning bodies %d w(%f %f %f) v(%f %f %f) with very high velocity or angular velocity, may be unstable\n", m_uniqueId,
			m_omega.m_x, m_omega.m_y, m_omega.m_z, m_veloc.m_x, m_veloc.m_y, m_veloc.m_z));
		//dAssert(0);
	}
#endif

	// this is correct
	const dFloat32 tol = (dFloat32(0.0125f) * dDegreeToRad);
	if (omegaMag2 > (tol * tol))
	{
		dFloat32 invOmegaMag = dRsqrt(omegaMag2);
		dVector omegaAxis(m_omega.Scale(invOmegaMag));
		dFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep;
		dQuaternion rotationStep(omegaAxis, omegaAngle);
		//m_rotation = m_rotation * rotationStep;
		//m_rotation.Scale(dRsqrt(m_rotation.DotProduct(m_rotation)));
		m_rotation = (m_rotation * rotationStep).Normalize();
		m_matrix = dMatrix(m_rotation, m_matrix.m_posit);
	}

	m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);
	dAssert(m_matrix.TestOrthogonal());

	m_residualVeloc = m_veloc;
	m_residualOmega = m_omega;
}

void ndBodyKinematic::IntegrateExternalForce(dFloat32 timestep)
{
	if (!m_equilibrium && (m_invMass.m_w > dFloat32(0.0f)))
	{
		const dVector accel(GetForce().Scale(m_invMass.m_w));
		const dVector torque(GetTorque());

		// using simple backward Euler or implicit integration, this is. 
		// f'(t + dt) = (f(t + dt) - f(t)) / dt  
		
		// therefore: 
		// f(t + dt) = f(t) + f'(t + dt) * dt
		
		// approximate f'(t + dt) by expanding the Taylor of f(w + dt)
		// f(w + dt) = f(w) + f'(w) * dt + f''(w) * dt^2 / 2! + ....
		
		// assume dt^2 is negligible, therefore we can truncate the expansion to
		// f(w + dt) ~= f(w) + f'(w) * dt
		
		// calculating dw as the  f'(w) = d(wx, wy, wz) | dt
		// dw/dt = a = (Tl - (wl x (wl * Il)) * Il^1
		
		// expanding f(w) 
		// f'(wx) = Ix * ax = Tx - (Iz - Iy) * wy * wz 
		// f'(wy) = Iy * ay = Ty - (Ix - Iz) * wz * wx
		// f'(wz) = Iz * az = Tz - (Iy - Ix) * wx * wy
		//
		// calculation the expansion 
		// Ix * ax = (Tx - (Iz - Iy) * wy * wz) - ((Iz - Iy) * wy * az + (Iz - Iy) * ay * wz) * dt
		// Iy * ay = (Ty - (Ix - Iz) * wz * wx) - ((Ix - Iz) * wz * ax + (Ix - Iz) * az * wx) * dt
		// Iz * az = (Tz - (Iy - Ix) * wx * wy) - ((Iy - Ix) * wx * ay + (Iy - Ix) * ax * wy) * dt   
		//
		// factorizing a we get
		// Ix * ax + (Iz - Iy) * dwy * az + (Iz - Iy) * dwz * ay = Tx - (Iz - Iy) * wy * wz
		// Iy * ay + (Ix - Iz) * dwz * ax + (Ix - Iz) * dwx * az = Ty - (Ix - Iz) * wz * wx
		// Iz * az + (Iy - Ix) * dwx * ay + (Iy - Ix) * dwy * ax = Tz - (Iy - Ix) * wx * wy
		
		dVector localOmega(m_matrix.UnrotateVector(m_omega));
		const dVector localAngularMomentum(m_mass * localOmega);
		const dVector angularMomentum(m_matrix.RotateVector(localAngularMomentum));
		const dVector gyroTorque(m_omega.CrossProduct(angularMomentum));
		const dVector localTorque(m_matrix.UnrotateVector(torque - gyroTorque));
		
		// and solving for alpha we get the angular acceleration at t + dt
		// calculate gradient at a full time step
		dVector gradientStep(localTorque.Scale(timestep));
		
		// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
		const dVector dw(localOmega.Scale(dFloat32(0.5f) * timestep));
		//dVector dw(localOmega.Scale(dFloat32(1.0f) * timestep));
		
		// calculates Jacobian matrix (
		//		dWx / dwx, dWx / dwy, dWx / dwz
		//		dWy / dwx, dWy / dwy, dWy / dwz
		//		dWz / dwx, dWz / dwy, dWz / dwz
		//		
		//		dWx / dwx = Ix, dWx / dwy = (Iz - Iy) * wz * dt, dWx / dwz = (Iz - Iy) * wy * dt)
		//		dWy / dwx = (Ix - Iz) * wz * dt, dWy / dwy = Iy, dWy / dwz = (Ix - Iz) * wx * dt
		//		dWz / dwx = (Iy - Ix) * wy * dt, dWz / dwy = (Iy - Ix) * wx * dt, dWz / dwz = Iz
		const dMatrix jacobianMatrix(
			dVector(m_mass[0], (m_mass[2] - m_mass[1]) * dw[2], (m_mass[2] - m_mass[1]) * dw[1], dFloat32(0.0f)),
			dVector((m_mass[0] - m_mass[2]) * dw[2], m_mass[1], (m_mass[0] - m_mass[2]) * dw[0], dFloat32(0.0f)),
			dVector((m_mass[1] - m_mass[0]) * dw[1], (m_mass[1] - m_mass[0]) * dw[0], m_mass[2], dFloat32(0.0f)),
			dVector::m_wOne);
		
		gradientStep = jacobianMatrix.SolveByGaussianElimination(gradientStep);
		
		localOmega += gradientStep;
		
		const dVector alpha(m_matrix.RotateVector(localTorque * m_invMass));
		
		SetAccel(accel);
		SetAlpha(alpha);
		m_veloc += accel.Scale(timestep);
		m_omega = m_matrix.RotateVector(localOmega);
	}
	else
	{
		SetAccel(dVector::m_zero);
		SetAlpha(dVector::m_zero);
	}
}

void ndBodyKinematic::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeId, dInt32 bodyId) const
{
	nd::TiXmlElement* const paramNode = new nd::TiXmlElement(ClassName());
	rootNode->LinkEndChild(paramNode);

	ndBody::Save(paramNode, assetPath, shapeId, bodyId);

	m_shapeInstance.Save(paramNode, shapeId);
	xmlSaveParam(paramNode, "invMass", m_invMass.m_w);
	dVector invInertia(m_invMass & dVector::m_triplexMask);
	xmlSaveParam(paramNode, "invPrincipalInertia", invInertia);
}
