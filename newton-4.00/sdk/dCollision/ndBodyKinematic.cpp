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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndShapeNull.h"
#include "ndRayCastNotify.h"
#include "ndBodyKinematic.h"
#include "ndShapeCompound.h"
#include "ndJointBilateralConstraint.h"

#define D_MINIMUM_MASS	ndFloat32(1.0e-5f)
#define D_INFINITE_MASS	ndFloat32(1.0e15f)
D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndBodyKinematic);

//class ndDummyCollision: public ndShapeNull
//{
//	public:
//	ndDummyCollision()
//		:ndShapeNull()
//	{
//		m_refCount.fetch_add(1);
//	}
//
//	~ndDummyCollision()
//	{
//		m_refCount.fetch_add(-1);
//	}
//
//	static ndShapeNull* GetNullShape()
//	{
//		static ndDummyCollision nullShape;
//		return &nullShape;
//	}
//};

ndBodyKinematic::ndContactMap::ndContactMap()
	:ndTree<ndContact*, ndContactkey, ndContainersFreeListAlloc<ndContact*>>()
{
}

ndContact* ndBodyKinematic::ndContactMap::FindContact(const ndBody* const body0, const ndBody* const body1) const
{
	ndContactkey key(body0->GetId(), body1->GetId());
	ndNode* const node = Find(key);
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
	,m_shapeInstance(new ndShapeNull)
	,m_mass(ndVector::m_zero)
	,m_invMass(ndVector::m_zero)
	,m_accel(ndVector::m_zero)
	,m_alpha(ndVector::m_zero)
	,m_gyroAlpha(ndVector::m_zero)
	,m_gyroTorque(ndVector::m_zero)
	,m_gyroRotation()
	,m_jointList()
	,m_contactList()
	,m_lock()
	,m_scene(nullptr)
	,m_islandParent(nullptr)
	,m_sceneNode(nullptr)
	,m_sceneBodyBodyNode(nullptr)
	,m_skeletonContainer(nullptr)
	,m_maxAngleStep(ndFloat32 (90.0f) * ndDegreeToRad)
	,m_maxLinearSpeed(ndFloat32 (100.0f))
	,m_weigh(ndFloat32 (0.0f))
	,m_rank(0)
	,m_index(0)
	,m_sleepingCounter(0)
{
	m_invWorldInertiaMatrix[3][3] = ndFloat32(1.0f);
	m_shapeInstance.m_ownerBody = this;
	SetMassMatrix(ndVector::m_zero);
}

ndBodyKinematic::ndBodyKinematic(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndBody(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_invWorldInertiaMatrix(dGetZeroMatrix())
	,m_shapeInstance(new ndShapeNull)
	,m_mass(ndVector::m_zero)
	,m_invMass(ndVector::m_zero)
	,m_accel(ndVector::m_zero)
	,m_alpha(ndVector::m_zero)
	,m_gyroAlpha(ndVector::m_zero)
	,m_gyroTorque(ndVector::m_zero)
	,m_gyroRotation()
	,m_jointList()
	,m_contactList()
	,m_lock()
	,m_scene(nullptr)
	,m_islandParent(nullptr)
	,m_sceneNode(nullptr)
	,m_sceneBodyBodyNode(nullptr)
	,m_skeletonContainer(nullptr)
	,m_weigh(ndFloat32(0.0f))
	,m_rank(0)
	,m_index(0)
	,m_sleepingCounter(0)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	m_invWorldInertiaMatrix[3][3] = ndFloat32(1.0f);
	ndShapeInstance instance(xmlNode->FirstChild("ndShapeInstance"), *desc.m_shapeMap);
	SetCollisionShape(instance);
	
	ndFloat32 invMass = xmlGetFloat(xmlNode, "invMass");
	SetMassMatrix(ndVector::m_zero);
	if (invMass > ndFloat32 (0.0f))
	{
		ndVector invInertia(xmlGetVector3(xmlNode, "invPrincipalInertia"));
		SetMassMatrix(ndFloat32 (1.0f)/invInertia.m_x, ndFloat32(1.0f) / invInertia.m_y, ndFloat32(1.0f) / invInertia.m_z, ndFloat32(1.0f) / invMass);
	}

	m_maxAngleStep = xmlGetFloat(xmlNode, "maxAngleStep");
	m_maxLinearSpeed = xmlGetFloat(xmlNode, "maxLinearSpeed");
}

ndBodyKinematic::~ndBodyKinematic()
{
	dAssert(m_scene == nullptr);
	dAssert(m_sceneNode == nullptr);
}

void ndBodyKinematic::SetSleepState(bool state)
{
	m_equilibrium = state ? 1 : 0;
	if ((m_invMass.m_w > ndFloat32(0.0f)) && (m_veloc.DotProduct(m_veloc).GetScalar() < ndFloat32(1.0e-10f)) && (m_omega.DotProduct(m_omega).GetScalar() < ndFloat32(1.0e-10f))) 
	{
		ndVector invalidateVeloc(ndFloat32(10.0f));
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

ndContact* ndBodyKinematic::FindContact(const ndBody* const otherBody) const
{
	ndScopeSpinLock lock(m_lock);
	return m_contactList.FindContact(this, otherBody);
}

void ndBodyKinematic::AttachContact(ndContact* const contact)
{
	ndScopeSpinLock lock(m_lock);
	dAssert((this == contact->GetBody0()) || (this == contact->GetBody1()));
	if (m_invMass.m_w > ndFloat32(0.0f))
	{
		m_equilibrium = 0;
	}

	m_contactList.AttachContact(contact);
}

void ndBodyKinematic::DetachContact(ndContact* const contact)
{
	ndScopeSpinLock lock(m_lock);
	dAssert((this == contact->GetBody0()) || (this == contact->GetBody1()));
	m_equilibrium = contact->m_body0->m_equilibrium & contact->m_body1->m_equilibrium;
	m_contactList.DetachContact(contact);
}

ndJointList::ndNode* ndBodyKinematic::AttachJoint(ndJointBilateralConstraint* const joint)
{
	m_equilibrium = 0;
	return m_jointList.Append(joint);
}

void ndBodyKinematic::DetachJoint(ndJointList::ndNode* const node)
{
	m_equilibrium = 0;
#ifdef _DEBUG
	bool found = false;
	for (ndJointList::ndNode* nodeptr = m_jointList.GetFirst(); nodeptr; nodeptr = nodeptr->GetNext())
	{
		found = found || nodeptr;
	}
	dAssert(found);
#endif
	m_jointList.Remove(node);
}

void ndBodyKinematic::SetMassMatrix(ndFloat32 mass, const ndMatrix& inertia)
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
		m_invMass = ndVector::m_zero;
	}
	else
	{
		ndFloat32 Ixx = dAbs(inertia[0][0]);
		ndFloat32 Iyy = dAbs(inertia[1][1]);
		ndFloat32 Izz = dAbs(inertia[2][2]);

		ndFloat32 Ixx1 = dClamp(Ixx, ndFloat32(0.0001f) * mass, ndFloat32(10000.0f) * mass);
		ndFloat32 Iyy1 = dClamp(Iyy, ndFloat32(0.0001f) * mass, ndFloat32(10000.0f) * mass);
		ndFloat32 Izz1 = dClamp(Izz, ndFloat32(0.0001f) * mass, ndFloat32(10000.0f) * mass);

		dAssert(Ixx1 > ndFloat32(0.0f));
		dAssert(Iyy1 > ndFloat32(0.0f));
		dAssert(Izz1 > ndFloat32(0.0f));

		m_mass.m_x = Ixx1;
		m_mass.m_y = Iyy1;
		m_mass.m_z = Izz1;
		m_mass.m_w = mass;

		m_invMass.m_x = ndFloat32(1.0f) / Ixx1;
		m_invMass.m_y = ndFloat32(1.0f) / Iyy1;
		m_invMass.m_z = ndFloat32(1.0f) / Izz1;
		m_invMass.m_w = ndFloat32(1.0f) / mass;
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

bool ndBodyKinematic::RayCast(ndRayCastNotify& callback, const ndFastRay& ray, ndFloat32 maxT) const
{
	ndVector l0(ray.m_p0);
	ndVector l1(ray.m_p0 + ray.m_diff.Scale(dMin(maxT, ndFloat32(1.0f))));

	bool state = false;
	if (dRayBoxClip(l0, l1, m_minAabb, m_maxAabb))
	{
		const ndMatrix& globalMatrix = m_shapeInstance.GetGlobalMatrix();
		ndVector localP0(globalMatrix.UntransformVector(l0) & ndVector::m_triplexMask);
		ndVector localP1(globalMatrix.UntransformVector(l1) & ndVector::m_triplexMask);
		ndVector p1p0(localP1 - localP0);
		dAssert(p1p0.m_w == ndFloat32(0.0f));
		if (p1p0.DotProduct(p1p0).GetScalar() > ndFloat32(1.0e-12f))
		{
			if (m_shapeInstance.GetCollisionMode())
			{
				ndContactPoint contactOut;
				ndFloat32 t = m_shapeInstance.RayCast(callback, localP0, localP1, this, contactOut);
				if (t < ndFloat32(1.0f))
				{
					dAssert(localP0.m_w == localP1.m_w);
					ndVector p(globalMatrix.TransformVector(localP0 + (localP1 - localP0).Scale(t)));
					t = ray.m_diff.DotProduct(p - ray.m_p0).GetScalar() / ray.m_diff.DotProduct(ray.m_diff).GetScalar();
					if (t < maxT)
					{
						dAssert(t >= ndFloat32(0.0f));
						dAssert(t <= ndFloat32(1.0f));
						contactOut.m_body0 = this;
						contactOut.m_body1 = this;
						contactOut.m_point = p;
						contactOut.m_normal = globalMatrix.RotateVector(contactOut.m_normal);
						state = callback.OnRayCastAction(contactOut, t) < ndFloat32 (1.0f);
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

ndMatrix ndBodyKinematic::CalculateInvInertiaMatrix() const
{
	const ndVector invIxx(m_invMass[0]);
	const ndVector invIyy(m_invMass[1]);
	const ndVector invIzz(m_invMass[2]);
	return ndMatrix(
		m_matrix.m_front.Scale(m_matrix.m_front[0]) * invIxx +
		m_matrix.m_up.Scale(m_matrix.m_up[0])	* invIyy +
		m_matrix.m_right.Scale(m_matrix.m_right[0]) * invIzz,

		m_matrix.m_front.Scale(m_matrix.m_front[1]) * invIxx +
   		m_matrix.m_up.Scale(m_matrix.m_up[1])	* invIyy +
		m_matrix.m_right.Scale(m_matrix.m_right[1]) * invIzz,

		m_matrix.m_front.Scale(m_matrix.m_front[2]) * invIxx +
		m_matrix.m_up.Scale(m_matrix.m_up[2])	* invIyy +
		m_matrix.m_right.Scale(m_matrix.m_right[2]) * invIzz,
		ndVector::m_wOne);
}

ndMatrix ndBodyKinematic::CalculateInertiaMatrix() const
{
	const ndVector Ixx(m_mass.m_x);
	const ndVector Iyy(m_mass.m_y);
	const ndVector Izz(m_mass.m_z);
	return ndMatrix(
		m_matrix.m_front.Scale(m_matrix.m_front[0]) * Ixx +
		m_matrix.m_up.Scale(m_matrix.m_up[0]) 	* Iyy +
		m_matrix.m_right.Scale(m_matrix.m_right[0]) * Izz,

		m_matrix.m_front.Scale(m_matrix.m_front[1]) * Ixx +
		m_matrix.m_up.Scale(m_matrix.m_up[1])       * Iyy +
		m_matrix.m_right.Scale(m_matrix.m_right[1]) * Izz,

		m_matrix.m_front.Scale(m_matrix.m_front[2]) * Ixx +
		m_matrix.m_up.Scale(m_matrix.m_up[2])       * Iyy +
		m_matrix.m_right.Scale(m_matrix.m_right[2]) * Izz,
		ndVector::m_wOne);

}

ndVector ndBodyKinematic::CalculateLinearMomentum() const
{
	return m_veloc.Scale(m_mass.m_w);
}

ndVector ndBodyKinematic::CalculateAngularMomentum() const
{
	const ndVector localOmega(m_matrix.UnrotateVector(m_omega));
	const ndVector localAngularMomentum(m_mass * localOmega);
	return m_matrix.RotateVector(localAngularMomentum);
}

ndFloat32 ndBodyKinematic::TotalEnergy() const
{
	ndVector energy (m_veloc.DotProduct(CalculateLinearMomentum()) + m_veloc.DotProduct(CalculateAngularMomentum()));
	return energy.AddHorizontal().GetScalar()* ndFloat32(0.5f);
}

void ndBodyKinematic::IntegrateVelocity(ndFloat32 timestep)
{
	dAssert(m_veloc.m_w == ndFloat32(0.0f));
	dAssert(m_omega.m_w == ndFloat32(0.0f));
	m_globalCentreOfMass += m_veloc.Scale(timestep);

	//const ndFloat32 omegaMag2 = dMax(m_omega.DotProduct(m_omega).GetScalar(), tol2);
	const ndFloat32 omegaMag2 = m_omega.DotProduct(m_omega).GetScalar();

#ifdef _DEBUG
	const ndFloat32 err2 = m_maxAngleStep * m_maxAngleStep;
	const ndFloat32 step2 = omegaMag2 * timestep * timestep;
	const ndFloat32 speed2 = m_veloc.DotProduct(m_veloc).GetScalar() * timestep * timestep;;
	if ((step2 > err2) || (speed2 > m_maxLinearSpeed))
	{
		dTrace(("warning bodies %d w(%f %f %f) v(%f %f %f) with very high velocity or angular velocity, may be unstable\n", m_uniqueId,
			m_omega.m_x, m_omega.m_y, m_omega.m_z, m_veloc.m_x, m_veloc.m_y, m_veloc.m_z));
		//dAssert(0);
	}
#endif

	const ndFloat32 tol = (ndFloat32(0.0125f) * ndDegreeToRad);
	const ndFloat32 tol2 = tol * tol;
	if (omegaMag2 > tol2)
	{
		// this is correct
		const ndFloat32 invOmegaMag = ndRsqrt(omegaMag2);
		const ndFloat32 omegaAngle = invOmegaMag * omegaMag2 * timestep;
		const ndVector omegaAxis(m_omega.Scale(invOmegaMag));
		const ndQuaternion rotationStep(omegaAxis, omegaAngle);
		const ndQuaternion rotation(m_rotation * rotationStep);
		m_rotation = rotation.Normalize();
		dAssert((m_rotation.DotProduct(m_rotation).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-5f));
		m_matrix = ndMatrix(m_rotation, m_matrix.m_posit);
	}

	m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);
	dAssert(m_matrix.TestOrthogonal());
}

void ndBodyKinematic::IntegrateExternalForce(ndFloat32 timestep)
{
	if (!m_equilibrium && (m_invMass.m_w > ndFloat32(0.0f)))
	{
		const ndVector accel(GetForce().Scale(m_invMass.m_w));
		const ndVector torque(GetTorque());

		// using simple backward Euler or implicit integration, this is. 
		// f'(t + dt) = (f(t + dt) - f(t)) / dt  
		
		// therefore: 
		// f(t + dt) = f(t) + f'(t + dt) * dt
		
		// approximate f'(t + dt) by expanding the Taylor of f(w + dt)
		// f(w + dt) = f(w) + f'(w) * dt + f''(w) * dt^2 / 2! + ....
		
		// assume dt^2 is negligible, therefore we can truncate the expansion to
		// f(w + dt) ~= f(w) + f'(w) * dt
		
		// calculating dw as the  f'(w) = d(wx, wy, wz) | dt
		// dw/dt = a = (Tl - (wl x (wl * Il)) * Il^-1
		
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
		
		ndVector localOmega(m_matrix.UnrotateVector(m_omega));
		const ndVector localAngularMomentum(m_mass * localOmega);
		const ndVector angularMomentum(m_matrix.RotateVector(localAngularMomentum));
		const ndVector gyroTorque(m_omega.CrossProduct(angularMomentum));
		const ndVector localTorque(m_matrix.UnrotateVector(torque - gyroTorque));
		
		// and solving for alpha we get the angular acceleration at t + dt
		// calculate gradient at a full time step
		ndVector gradientStep(localTorque.Scale(timestep));
		
		// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
		const ndVector dw(localOmega.Scale(ndFloat32(0.5f) * timestep));
		//ndVector dw(localOmega.Scale(ndFloat32(1.0f) * timestep));
		
		// calculates Jacobian matrix (
		//		dWx / dwx, dWx / dwy, dWx / dwz
		//		dWy / dwx, dWy / dwy, dWy / dwz
		//		dWz / dwx, dWz / dwy, dWz / dwz
		//		
		//		dWx / dwx = Ix, dWx / dwy = (Iz - Iy) * wz * dt, dWx / dwz = (Iz - Iy) * wy * dt)
		//		dWy / dwx = (Ix - Iz) * wz * dt, dWy / dwy = Iy, dWy / dwz = (Ix - Iz) * wx * dt
		//		dWz / dwx = (Iy - Ix) * wy * dt, dWz / dwy = (Iy - Ix) * wx * dt, dWz / dwz = Iz
		const ndMatrix jacobianMatrix(
			ndVector(m_mass.m_x, (m_mass.m_z - m_mass.m_y) * dw.m_z, (m_mass.m_z - m_mass.m_y) * dw.m_y, ndFloat32(0.0f)),
			ndVector((m_mass.m_x - m_mass.m_z) * dw.m_z, m_mass.m_y, (m_mass.m_x - m_mass.m_z) * dw.m_x, ndFloat32(0.0f)),
			ndVector((m_mass.m_y - m_mass.m_x) * dw.m_y, (m_mass.m_y - m_mass.m_x) * dw.m_x, m_mass.m_z, ndFloat32(0.0f)),
			ndVector::m_wOne);
		
		gradientStep = jacobianMatrix.SolveByGaussianElimination(gradientStep);
		
		localOmega += gradientStep;
		
		const ndVector alpha(m_matrix.RotateVector(localTorque * m_invMass));
		
		SetAccel(accel);
		SetAlpha(alpha);
		m_veloc += accel.Scale(timestep);
		m_omega = m_matrix.RotateVector(localOmega);
	}
	else
	{
		SetAccel(ndVector::m_zero);
		SetAlpha(ndVector::m_zero);
	}
}

void ndBodyKinematic::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndBody::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
	
	xmlSaveParam(childNode, "invMass", m_invMass.m_w);
	ndVector invInertia(m_invMass & ndVector::m_triplexMask);
	xmlSaveParam(childNode, "invPrincipalInertia", invInertia);

	xmlSaveParam(childNode, "maxAngleStep", m_maxAngleStep);
	xmlSaveParam(childNode, "maxLinearSpeed", m_maxLinearSpeed);

	m_shapeInstance.Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
}
