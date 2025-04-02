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
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndContact.h"
#include "ndShapeNull.h"
#include "ndRayCastNotify.h"
#include "ndBodyKinematic.h"
#include "ndShapeCompound.h"
#include "ndJointBilateralConstraint.h"

#define D_MINIMUM_MASS	ndFloat32(1.0e-5f)
#define D_INFINITE_MASS	ndFloat32(1.0e15f)

ndVector ndBodyKinematic::m_velocTol(ndVector(ndFloat32(1.0e-8f)) & ndVector::m_triplexMask);

ndBodyKinematic::ndContactkey::ndContactkey(ndUnsigned32 tag0, ndUnsigned32 tag1)
	:m_tagLow(ndMin(tag0, tag1))
	,m_tagHigh(ndMax(tag0, tag1))
{
	ndAssert(m_tagLow < m_tagHigh);
}

bool ndBodyKinematic::ndContactkey::operator== (const ndContactkey& key) const
{
	return m_tag == key.m_tag;
}

bool ndBodyKinematic::ndContactkey::operator< (const ndContactkey& key) const
{
	return m_tag < key.m_tag;
}

bool ndBodyKinematic::ndContactkey::operator> (const ndContactkey& key) const
{
	return m_tag > key.m_tag;
}

ndBodyKinematic::ndContactMap::ndContactMap()
	:ndTree<ndContact*, ndContactkey, ndContainersFreeListAlloc<ndContact*>>()
{
}

ndBodyKinematic::ndContactMap::~ndContactMap()
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
	ndAssert(!Find(key));
	Insert(contact, key);
}

void ndBodyKinematic::ndContactMap::DetachContact(ndContact* const contact)
{
	ndBody* const body0 = contact->GetBody0();
	ndBody* const body1 = contact->GetBody1();
	ndContactkey key(body0->GetId(), body1->GetId());
	ndAssert(Find(key));
	Remove(key);
}

ndBodyKinematic::ndBodyKinematic()
	:ndBody()
	,m_inertiaPrincipalAxis(ndGetIdentityMatrix())
	,m_invWorldInertiaMatrix(ndGetZeroMatrix())
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
	,m_skeletonContainer(nullptr)
	,m_spetialUpdateNode(nullptr)
	,m_maxAngleStep(ndFloat32(90.0f)* ndDegreeToRad)
	,m_maxLinearStep(ndFloat32(1.0f))
	,m_weigh(ndFloat32(0.0f))
	,m_index(0)
	,m_bodyNodeIndex(-1)
	,m_buildSkelIndex(0)
	,m_sceneNodeIndex(-1)
	,m_buildBodyNodeIndex(-1)
	,m_buildSceneNodeIndex(-1)
{
	m_invWorldInertiaMatrix[3][3] = ndFloat32(1.0f);
	m_shapeInstance.m_ownerBody = this;
	SetMassMatrix(ndVector::m_zero);
}

ndBodyKinematic::ndBodyKinematic(const ndBodyKinematic& src)
	:ndBody(src)
	,m_inertiaPrincipalAxis(src.m_inertiaPrincipalAxis)
	,m_invWorldInertiaMatrix(src.m_invWorldInertiaMatrix)
	,m_shapeInstance(src.m_shapeInstance)
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
	,m_skeletonContainer(nullptr)
	,m_spetialUpdateNode(nullptr)
	,m_maxAngleStep(ndFloat32(90.0f)* ndDegreeToRad)
	,m_maxLinearStep(ndFloat32(1.0f))
	,m_weigh(ndFloat32(0.0f))
	,m_index(0)
	,m_bodyNodeIndex(-1)
	,m_buildSkelIndex(0)
	,m_sceneNodeIndex(-1)
	,m_buildBodyNodeIndex(-1)
	,m_buildSceneNodeIndex(-1)
{
}

ndBodyKinematic::~ndBodyKinematic()
{
	ndAssert(m_scene == nullptr);
	ndAssert(m_sceneNode == nullptr);
	ndAssert(m_spetialUpdateNode == nullptr);
}

ndUnsigned32 ndBodyKinematic::GetIndex() const
{
	return ndUnsigned32(m_index);
}

ndFloat32 ndBodyKinematic::GetInvMass() const
{
	return m_invMass.m_w;
}

const ndVector ndBodyKinematic::GetInvInertia() const
{
	return m_invMass & ndVector::m_triplexMask;
}

const ndVector& ndBodyKinematic::GetMassMatrix() const
{
	return m_mass;
}

const ndMatrix& ndBodyKinematic::GetInvInertiaMatrix() const
{
	return m_invWorldInertiaMatrix;
}

ndVector ndBodyKinematic::GetGyroAlpha() const
{
	return m_gyroAlpha;
}

ndVector ndBodyKinematic::GetGyroTorque() const
{
	return m_gyroTorque;
}

void ndBodyKinematic::GetMassMatrix(ndFloat32& Ixx, ndFloat32& Iyy, ndFloat32& Izz, ndFloat32& mass)
{
	Ixx = m_mass.m_x;
	Iyy = m_mass.m_y;
	Izz = m_mass.m_z;
	mass = m_mass.m_w;
}

void ndBodyKinematic::SetMassMatrix(const ndVector& massMatrix)
{
	ndMatrix inertia(ndGetIdentityMatrix());
	inertia[0][0] = massMatrix.m_x;
	inertia[1][1] = massMatrix.m_y;
	inertia[2][2] = massMatrix.m_z;
	SetMassMatrix(massMatrix.m_w, inertia);
}

void ndBodyKinematic::SetMassMatrix(ndFloat32 Ixx, ndFloat32 Iyy, ndFloat32 Izz, ndFloat32 mass)
{
	SetMassMatrix(ndVector(Ixx, Iyy, Izz, mass));
}

ndMatrix ndBodyKinematic::GetPrincipalAxis() const
{
	return m_inertiaPrincipalAxis;
}

ndBodyKinematic* ndBodyKinematic::GetAsBodyKinematic()
{
	return this;
}

ndScene* ndBodyKinematic::GetScene() const
{
	return m_scene;
}

void ndBodyKinematic::SetSceneNodes(ndScene* const scene, ndBodyListView::ndNode* const node)
{
	m_scene = scene;
	m_sceneNode = node;
}

ndVector ndBodyKinematic::GetForce() const
{
	return ndVector::m_zero;
}

ndVector ndBodyKinematic::GetTorque() const
{
	return ndVector::m_zero;
}

void ndBodyKinematic::SetForce(const ndVector&)
{
}

void ndBodyKinematic::SetTorque(const ndVector&)
{
}

ndVector ndBodyKinematic::GetAccel() const
{
	return m_accel;
}

void ndBodyKinematic::SetAccel(const ndVector& accel)
{
	m_accel = accel;
}

ndVector ndBodyKinematic::GetAlpha() const
{
	return m_alpha;
}

void ndBodyKinematic::SetAlpha(const ndVector& alpha)
{
	m_alpha = alpha;
}

void ndBodyKinematic::AddDampingAcceleration(ndFloat32)
{
}

void ndBodyKinematic::SetAccel(const ndJacobian& accel)
{
	SetAccel(accel.m_linear);
	SetAlpha(accel.m_angular);
}

void ndBodyKinematic::PrepareStep(ndInt32 index)
{
	m_index = index;
	m_isJointFence0 = 1;
	m_isJointFence1 = 1;
	m_isConstrained = 0;
	m_buildSkelIndex = 0;
	m_islandParent = this;
	m_weigh = ndFloat32(0.0f);
	m_isStatic = ndUnsigned8(m_invMass.m_w == ndFloat32(0.0f));
	m_equilibrium = ndUnsigned8(m_isStatic | m_equilibrium);
	m_equilibrium0 = m_equilibrium;
}

ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap()
{
	return m_contactList;
}

const ndBodyKinematic::ndContactMap& ndBodyKinematic::GetContactMap() const
{
	return m_contactList;
}

ndBodyKinematic::ndJointList& ndBodyKinematic::GetJointList()
{
	return m_jointList;
}

const ndBodyKinematic::ndJointList& ndBodyKinematic::GetJointList() const
{
	return m_jointList;
}

ndShapeInstance& ndBodyKinematic::GetCollisionShape()
{
	return (ndShapeInstance&)m_shapeInstance;
}

const ndShapeInstance& ndBodyKinematic::GetCollisionShape() const
{
	return m_shapeInstance;
}

bool ndBodyKinematic::GetAutoSleep() const
{
	return m_autoSleep ? true : false;
}

bool ndBodyKinematic::GetSleepState() const
{
	return m_equilibrium ? true : false;
}

void ndBodyKinematic::RestoreSleepState(bool state)
{
	m_equilibrium = ndUnsigned8(state ? 1 : 0);
}

void ndBodyKinematic::SetAutoSleep(bool state)
{
	m_autoSleep = ndUnsigned8(state ? 1 : 0);
	SetSleepState(false);
}

ndSkeletonContainer* ndBodyKinematic::GetSkeleton() const
{
	return m_skeletonContainer;
}

void ndBodyKinematic::SetSkeleton(ndSkeletonContainer* const skeleton)
{
	m_skeletonContainer = skeleton;
}

ndFloat32 ndBodyKinematic::GetMaxLinearStep() const
{
	return m_maxLinearStep;
}

ndFloat32 ndBodyKinematic::GetMaxAngularStep() const
{
	return m_maxAngleStep;
}

void ndBodyKinematic::SetDebugMaxLinearAndAngularIntegrationStep(ndFloat32 angleInRadian, ndFloat32 stepInUnitPerSeconds)
{
	m_maxLinearStep = ndMax(ndAbs(stepInUnitPerSeconds), ndFloat32(1.0f));
	m_maxAngleStep = ndMax(ndAbs(angleInRadian), ndFloat32(90.0f) * ndDegreeToRad);
}

void ndBodyKinematic::SetLinearDamping(ndFloat32)
{
}

ndFloat32 ndBodyKinematic::GetLinearDamping() const
{
	return ndFloat32(0.0f);
}

void ndBodyKinematic::SetAngularDamping(const ndVector&)
{
}

ndVector ndBodyKinematic::GetAngularDamping() const
{
	return ndVector::m_zero;
}

ndVector ndBodyKinematic::GetCachedDamping() const
{
	return ndVector::m_one;
}

void ndBodyKinematic::UpdateInvInertiaMatrix()
{
	ndAssert(m_invWorldInertiaMatrix[0][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[1][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[2][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[3][3] == ndFloat32(1.0f));

	m_invWorldInertiaMatrix = CalculateInvInertiaMatrix();

	ndAssert(m_invWorldInertiaMatrix[0][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[1][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[2][3] == ndFloat32(0.0f));
	ndAssert(m_invWorldInertiaMatrix[3][3] == ndFloat32(1.0f));
}

void ndBodyKinematic::IntegrateGyroSubstep(const ndVector&)
{
}

ndJacobian ndBodyKinematic::IntegrateForceAndToque(const ndVector&, const ndVector&, const ndVector&) const
{
	ndJacobian step;
	step.m_linear = ndVector::m_zero;
	step.m_angular = ndVector::m_zero;
	return step;
}

void ndBodyKinematic::AddImpulse(const ndVector&, const ndVector&, ndFloat32)
{
}

void ndBodyKinematic::ApplyImpulsePair(const ndVector&, const ndVector&, ndFloat32)
{
}

void ndBodyKinematic::ApplyImpulsesAtPoint(ndInt32, const ndVector* const, const ndVector* const, ndFloat32)
{
}

void ndBodyKinematic::SpecialUpdate(ndFloat32)
{
	ndAssert(0);
}

void ndBodyKinematic::ApplyExternalForces(ndInt32, ndFloat32)
{
}

void ndBodyKinematic::SetAcceleration(const ndVector&, const ndVector&)
{
	m_accel = ndVector::m_zero;
	m_alpha = ndVector::m_zero;
}


void ndBodyKinematic::SetSleepState(bool state)
{
	m_equilibrium = ndUnsigned8 (state ? 1 : 0);
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
	ndAssert((this == contact->GetBody0()) || (this == contact->GetBody1()));
	if (m_invMass.m_w > ndFloat32(0.0f))
	{
		m_equilibrium = 0;
	}

	m_contactList.AttachContact(contact);
}

void ndBodyKinematic::DetachContact(ndContact* const contact)
{
	ndScopeSpinLock lock(m_lock);
	ndAssert((this == contact->GetBody0()) || (this == contact->GetBody1()));
	//m_equilibrium = ndUnsigned8(contact->m_body0->m_equilibrium & contact->m_body1->m_equilibrium);
	if (contact->IsActive() && m_invMass.m_w > ndFloat32(0.0f))
	{
		m_equilibrium = 0;
	}
	m_contactList.DetachContact(contact);
}

ndBodyKinematic::ndJointList::ndNode* ndBodyKinematic::AttachJoint(ndJointBilateralConstraint* const joint)
{
	m_equilibrium = 0;
	#ifdef _DEBUG
	ndBody* const body0 = joint->GetBody0();
	ndBody* const body1 = joint->GetBody1();
	for (ndJointList::ndNode* node = m_jointList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const bodyJoint = node->GetInfo();
		ndBody* const bodyInJoint0 = bodyJoint->GetBody0();
		ndBody* const bodyInJoint1 = bodyJoint->GetBody1();
		bool test = (body0 == bodyInJoint0) && (body1 == bodyInJoint1);
		test = test || ((body1 == bodyInJoint0) && (body0 == bodyInJoint1));
		test = test && (body1->GetInvMass() > ndFloat32(0.0f));
		test = test && bodyJoint->IsActive();
		if (test)
		{
			ndTrace(("warning body %d and body %d already connected by a biletaral joint\n", body0->GetId(), body1->GetId()));
			ndAssert(0);
		}
	}
	#endif
	
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
	ndAssert(found);
#endif
	m_jointList.Remove(node);
}

void ndBodyKinematic::SetMassMatrix(ndFloat32 mass, const ndShapeInstance& shapeInstance, bool fullInertia)
{
	ndMatrix inertia(shapeInstance.CalculateInertia());

	ndVector origin(inertia.m_posit);
	for (ndInt32 i = 0; i < 3; ++i)
	{
		inertia[i] = inertia[i].Scale(mass);
	}

	// although the engine fully supports asymmetric inertia, I will ignore cross inertia for now
	SetCentreOfMass(origin);

	if (!fullInertia)
	{
		ndMatrix matrix (inertia);
		ndVector eigenValues(matrix.EigenVectors());
		inertia = ndGetIdentityMatrix();
		inertia[0][0] = eigenValues[0];
		inertia[1][1] = eigenValues[1];
		inertia[2][2] = eigenValues[2];
	}
	SetMassMatrix(mass, inertia);
}

void ndBodyKinematic::SetIntrinsicMassMatrix(ndFloat32 mass, const ndShapeInstance& shapeInstance, bool fullInertia)
{
	const ndMatrix inertia(shapeInstance.CalculateInertia());
	const ndVector saveCom(inertia.m_posit);
	//SetCentreOfMass(inertia.m_posit);

	ndShapeInstance instance(shapeInstance);
	ndMatrix matrix(instance.GetLocalMatrix());
	matrix.m_posit = ndVector::m_wOne;
	if (instance.GetShape()->GetAsShapeConvexHull())
	{
		matrix.m_posit = saveCom * ndVector::m_negOne;
		matrix.m_posit.m_w = ndFloat32 (1.0f);
	}
	instance.SetLocalMatrix(matrix);
	SetMassMatrix(mass, instance, fullInertia);
	SetCentreOfMass(saveCom);
}

void ndBodyKinematic::SetMassMatrix(ndFloat32 mass, const ndMatrix& inertia)
{
	mass = ndAbs(mass);
	ndShape* const shape = m_shapeInstance.GetShape();

	m_inertiaPrincipalAxis = ndGetIdentityMatrix();
	if ((mass < D_MINIMUM_MASS) || shape->GetAsShapeStaticMesh())
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
		ndVector eigenValues(inertia[0][0], inertia[1][1], inertia[2][2], ndFloat32(0.0f));
		if (!((inertia[0][1] == ndFloat32(0.0f)) && (inertia[0][2] == ndFloat32(0.0f)) && (inertia[1][2] == ndFloat32(0.0f))))
		{
			m_inertiaPrincipalAxis = inertia;
			eigenValues = m_inertiaPrincipalAxis.EigenVectors();
		}

		ndFloat32 Ixx = ndAbs(eigenValues[0]);
		ndFloat32 Iyy = ndAbs(eigenValues[1]);
		ndFloat32 Izz = ndAbs(eigenValues[2]);

		ndFloat32 Ixx1 = ndClamp(Ixx, ndFloat32(0.0001f) * mass, ndFloat32(10000.0f) * mass);
		ndFloat32 Iyy1 = ndClamp(Iyy, ndFloat32(0.0001f) * mass, ndFloat32(10000.0f) * mass);
		ndFloat32 Izz1 = ndClamp(Izz, ndFloat32(0.0001f) * mass, ndFloat32(10000.0f) * mass);

		ndAssert(Ixx1 > ndFloat32(0.0f));
		ndAssert(Iyy1 > ndFloat32(0.0f));
		ndAssert(Izz1 > ndFloat32(0.0f));

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
	ndVector l1(ray.m_p0 + ray.m_diff.Scale(ndMin(maxT, ndFloat32(1.0f))));

	bool state = false;
	if (ndRayBoxClip(l0, l1, m_minAabb, m_maxAabb))
	{
		const ndMatrix& globalMatrix = m_shapeInstance.GetGlobalMatrix();
		ndVector localP0(globalMatrix.UntransformVector(l0) & ndVector::m_triplexMask);
		ndVector localP1(globalMatrix.UntransformVector(l1) & ndVector::m_triplexMask);
		ndVector p1p0(localP1 - localP0);
		ndAssert(p1p0.m_w == ndFloat32(0.0f));
		if (p1p0.DotProduct(p1p0).GetScalar() > ndFloat32(1.0e-12f))
		{
			if (m_shapeInstance.GetCollisionMode())
			{
				ndContactPoint contactOut;
				ndFloat32 t = m_shapeInstance.RayCast(callback, localP0, localP1, this, contactOut);
				if (t < ndFloat32(1.0f))
				{
					ndAssert(localP0.m_w == localP1.m_w);
					ndVector p(globalMatrix.TransformVector(localP0 + (localP1 - localP0).Scale(t)));
					t = ray.m_diff.DotProduct(p - ray.m_p0).GetScalar() / ray.m_diff.DotProduct(ray.m_diff).GetScalar();
					if (t < maxT)
					{
						ndAssert(t >= ndFloat32(0.0f));
						ndAssert(t <= ndFloat32(1.0f));
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

void ndBodyKinematic::SetMatrixUpdateScene(const ndMatrix& matrix)
{
	SetMatrix(matrix);
	ndScene* const scene = GetScene();
	if (scene)
	{
		ndUnsigned8 sceneEquilibrium = 1;
		ndUnsigned8 sceneForceUpdate = m_sceneForceUpdate;

		ndContactMap::Iterator it(GetContactMap());
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = it.GetNode()->GetInfo();
			contact->ClearSeparatingDistance();
		}

		if (ndUnsigned8(!m_equilibrium) | sceneForceUpdate)
		{
			ndBvhLeafNode* const bodyNode = scene->m_bvhSceneManager.GetLeafNode(this);
			ndAssert(bodyNode->GetAsSceneBodyNode());
			ndAssert(!bodyNode->GetLeft());
			ndAssert(!bodyNode->GetRight());
			ndAssert(!GetCollisionShape().GetShape()->GetAsShapeNull());
		
			UpdateCollisionMatrix();
			const ndInt32 test = ndBoxInclusionTest(m_minAabb, m_maxAabb, bodyNode->m_minBox, bodyNode->m_maxBox);
			if (!test)
			{
				bodyNode->SetAabb(m_minAabb, m_maxAabb);
			}
			sceneEquilibrium = ndUnsigned8(!sceneForceUpdate & (test != 0));
		}
		m_sceneForceUpdate = 0;
		m_sceneEquilibrium = sceneEquilibrium;
	}
}

ndMatrix ndBodyKinematic::CalculateInvInertiaMatrix() const
{
	const ndVector invIxx(m_invMass[0]);
	const ndVector invIyy(m_invMass[1]);
	const ndVector invIzz(m_invMass[2]);

	const ndMatrix matrix(m_inertiaPrincipalAxis * m_matrix);
	return ndMatrix(
		matrix.m_front.Scale(matrix.m_front[0]) * invIxx +
		matrix.m_up.Scale(matrix.m_up[0])		* invIyy +
		matrix.m_right.Scale(matrix.m_right[0]) * invIzz,

		matrix.m_front.Scale(matrix.m_front[1]) * invIxx +
   		matrix.m_up.Scale(matrix.m_up[1])		* invIyy +
		matrix.m_right.Scale(matrix.m_right[1]) * invIzz,

		matrix.m_front.Scale(matrix.m_front[2]) * invIxx +
		matrix.m_up.Scale(matrix.m_up[2])		* invIyy +
		matrix.m_right.Scale(matrix.m_right[2]) * invIzz,
		ndVector::m_wOne);
}

ndMatrix ndBodyKinematic::CalculateInertiaMatrix() const
{
	const ndVector Ixx(m_mass.m_x);
	const ndVector Iyy(m_mass.m_y);
	const ndVector Izz(m_mass.m_z);

	const ndMatrix matrix(m_inertiaPrincipalAxis * m_matrix);
	return ndMatrix(
		matrix.m_front.Scale(matrix.m_front[0]) * Ixx +
		matrix.m_up.Scale(matrix.m_up[0]) 		* Iyy +
		matrix.m_right.Scale(matrix.m_right[0]) * Izz,

		matrix.m_front.Scale(matrix.m_front[1]) * Ixx +
		matrix.m_up.Scale(matrix.m_up[1])       * Iyy +
		matrix.m_right.Scale(matrix.m_right[1]) * Izz,

		matrix.m_front.Scale(matrix.m_front[2]) * Ixx +
		matrix.m_up.Scale(matrix.m_up[2])       * Iyy +
		matrix.m_right.Scale(matrix.m_right[2]) * Izz,
		ndVector::m_wOne);
}

ndVector ndBodyKinematic::CalculateLinearMomentum() const
{
	return m_veloc.Scale(m_mass.m_w);
}

ndVector ndBodyKinematic::CalculateAngularMomentum() const
{
	const ndVector localOmega(m_inertiaPrincipalAxis.UnrotateVector (m_matrix.UnrotateVector(m_omega)));
	const ndVector localAngularMomentum(m_mass * localOmega);
	return m_matrix.RotateVector(m_inertiaPrincipalAxis.RotateVector(localAngularMomentum));
}

ndJacobian ndBodyKinematic::CalculateNetForce() const
{
	ndJacobian force;

	force.m_linear = GetForce();
	force.m_angular = GetTorque();
	for (ndJointList::ndNode* node = GetJointList().GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		if (joint->GetBody0() == this)
		{
			force.m_linear += joint->GetForceBody0();
			force.m_angular += joint->GetTorqueBody0();
		}
		else
		{
			ndAssert(joint->GetBody1() == this);
			force.m_linear += joint->GetForceBody1();
			force.m_angular += joint->GetTorqueBody1();
		}
	}

	const ndBodyKinematic::ndContactMap& contactMap = GetContactMap();
	ndBodyKinematic::ndContactMap::Iterator it(contactMap);
	for (it.Begin(); it; it++)
	{
		ndContact* const fronterContact = it.GetNode()->GetInfo();
		if (fronterContact->IsActive())
		{
			if (fronterContact->GetBody0() == this)
			{
				force.m_linear += fronterContact->GetForceBody0();
				force.m_angular += fronterContact->GetTorqueBody0();
			}
			else
			{
				ndAssert(fronterContact->GetBody1() == this);
				force.m_linear += fronterContact->GetForceBody1();
				force.m_angular += fronterContact->GetTorqueBody1();
			}
		}
	}
	return force;
}

ndFloat32 ndBodyKinematic::TotalEnergy() const
{
	ndVector energy (m_veloc.DotProduct(CalculateLinearMomentum()) + m_veloc.DotProduct(CalculateAngularMomentum()));
	return energy.AddHorizontal().GetScalar()* ndFloat32(0.5f);
}

void ndBodyKinematic::ClearMemory()
{
}

void ndBodyKinematic::IntegrateVelocity(ndFloat32 timestep)
{
	ndAssert(m_veloc.m_w == ndFloat32(0.0f));
	ndAssert(m_omega.m_w == ndFloat32(0.0f));
	m_globalCentreOfMass += m_veloc.Scale(timestep);

	const ndFloat32 omegaMag2 = m_omega.DotProduct(m_omega).GetScalar();

#ifdef _DEBUG
	const ndFloat32 angular2 = omegaMag2 * timestep * timestep;
	const ndFloat32 linear2 = m_veloc.DotProduct(m_veloc).GetScalar() * timestep * timestep;;
	const ndFloat32 maxAngularStep2 = m_maxAngleStep * m_maxAngleStep;
	const ndFloat32 maxLinearStep2 = m_maxLinearStep * m_maxLinearStep;
	if ((angular2 > maxAngularStep2) || (linear2 > maxLinearStep2))
	{
		ndTrace(("warning body %d w(%f %f %f) v(%f %f %f) with very high velocity or angular velocity, may be unstable\n", 
			m_uniqueId,	
			m_omega.m_x, m_omega.m_y, m_omega.m_z, 
			m_veloc.m_x, m_veloc.m_y, m_veloc.m_z));
		//ndAssert(0);
	}
#endif

	const ndFloat32 tol = (ndFloat32(0.0125f) * ndDegreeToRad);
	const ndFloat32 tol2 = tol * tol;
	if (omegaMag2 > tol2)
	{
		const ndFloat32 omegaAngle = ndSqrt(omegaMag2);
		const ndVector omegaAxis(m_omega.Scale(ndFloat32 (1.0f)/ omegaAngle));
		const ndQuaternion rotationStep(omegaAxis, omegaAngle * timestep);
		const ndQuaternion rotation(m_rotation * rotationStep);
		m_rotation = rotation.Normalize();
		ndAssert((m_rotation.DotProduct(m_rotation).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-5f));
		m_matrix = ndCalculateMatrix(m_rotation, m_matrix.m_posit);
	}

	m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);
	ndAssert(m_matrix.TestOrthogonal());
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
		
		const ndMatrix matrix(m_inertiaPrincipalAxis * m_matrix);

		ndVector localOmega(matrix.UnrotateVector(m_omega));
		const ndVector localAngularMomentum(m_mass * localOmega);
		const ndVector angularMomentum(matrix.RotateVector(localAngularMomentum));
		const ndVector gyroTorque(m_omega.CrossProduct(angularMomentum));
		const ndVector localTorque(matrix.UnrotateVector(torque - gyroTorque));
		// and solving for alpha we get the angular acceleration at t + dt
		
		// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
		const ndVector dw(localOmega.Scale(ndFloat32(0.5f) * timestep));
		
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
		
		const ndVector gradientStep (jacobianMatrix.SolveByGaussianElimination(localTorque.Scale(timestep)));
		localOmega += gradientStep;
		
		const ndVector alpha(matrix.RotateVector(localTorque * m_invMass));
		
		SetAccel(accel);
		SetAlpha(alpha);
		m_veloc += accel.Scale(timestep);
		m_omega = matrix.RotateVector(localOmega);
	}
	else
	{
		SetAccel(ndVector::m_zero);
		SetAlpha(ndVector::m_zero);
	}
}

void ndBodyKinematic::EvaluateSleepState(ndFloat32 freezeSpeed2, ndFloat32)
{
	m_isJointFence0 = 1;
	if (m_isStatic)
	{
		m_equilibrium = 1;
	}
	else
	{
		ndInt32 count = 0;
		ndUnsigned8 equilibrium = (m_invMass.m_w == ndFloat32(0.0f)) ? 1 : (m_autoSleep & ~m_equilibriumOverride);
		const ndVector isMovingMask(m_veloc + m_omega);
		const ndVector mask(isMovingMask.TestZero());
		const ndInt32 test = mask.GetSignMask() & 7;
		if (test != 7)
		{
			const ndFloat32 speed2 = m_veloc.DotProduct(m_veloc).GetScalar();
			const ndFloat32 omega2 = m_omega.DotProduct(m_omega).GetScalar();
			ndUnsigned32 equilibriumTest = ndUnsigned32((speed2 < freezeSpeed2) && (omega2 < freezeSpeed2));

			if (equilibriumTest)
			{
				const ndFloat32 velocityDragCoeff = (count <= 1) ? D_FREEZZING_VELOCITY_DRAG : ndFloat32(0.9999f);
				const ndVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, ndFloat32(0.0f));
				const ndVector veloc(m_veloc * velocDragVect);
				const ndVector omega(m_omega * velocDragVect);
				const ndVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
				const ndVector omegaMask(omega.DotProduct(omega) > m_velocTol);
				m_veloc = velocMask & veloc;
				m_omega = omegaMask & omega;
			}

			equilibrium &= equilibriumTest;
		}
		m_isJointFence0 = equilibrium;
		if (equilibrium & ~m_isConstrained)
		{
			m_equilibrium = equilibrium;
		}
	}
}

void ndBodyKinematic::InitSurrogateBody(ndBodyKinematic* const surrogate) const
{
	surrogate->m_veloc = m_veloc;
	surrogate->m_omega = m_omega;
	surrogate->m_matrix = m_matrix;
	surrogate->m_localCentreOfMass = m_localCentreOfMass;
	surrogate->m_globalCentreOfMass = m_globalCentreOfMass;
	
	surrogate->m_mass = m_mass;
	surrogate->m_accel = m_accel;
	surrogate->m_alpha = m_alpha;
	surrogate->m_invMass = m_invMass;
	surrogate->m_gyroAlpha = m_gyroAlpha;
	surrogate->m_gyroTorque = m_gyroTorque;
	surrogate->m_gyroRotation = m_gyroRotation;
	surrogate->m_invWorldInertiaMatrix = m_invWorldInertiaMatrix;
	surrogate->m_inertiaPrincipalAxis = m_inertiaPrincipalAxis;
}

