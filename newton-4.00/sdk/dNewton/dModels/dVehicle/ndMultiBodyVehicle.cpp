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
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndJointHinge.h"
#include "ndBodyDynamic.h"
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleMotor.h"
#include "ndMultiBodyVehicleGearBox.h"
#include "ndMultiBodyVehicleTireJoint.h"
#include "ndMultiBodyVehicleTorsionBar.h"
#include "ndMultiBodyVehicleDifferential.h"
#include "ndMultiBodyVehicleDifferentialAxle.h"

#define D_MAX_CONTACT_PENETRATION	  ndFloat32 (1.0e-2f)
#define D_MIN_CONTACT_CLOSE_DISTANCE2 ndFloat32 (5.0e-2f * 5.0e-2f)

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndMultiBodyVehicle)

ndMultiBodyVehicle::ndMultiBodyVehicle(const ndVector& frontDir, const ndVector& upDir)
	:ndModel()
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
	,m_motor(nullptr)
	,m_tireShape(new ndShapeChamferCylinder(ndFloat32(0.75f), ndFloat32(0.5f)))
	,m_gearBox(nullptr)
	,m_torsionBar(nullptr)
	,m_tireList()
	,m_extraBodiesAttachmentList()
	,m_axleList()
	,m_differentialList()
	,m_extraJointsAttachmentList()
	,m_downForce()
{
	m_tireShape->AddRef();
	m_localFrame.m_front = frontDir & ndVector::m_triplexMask;
	m_localFrame.m_up = upDir & ndVector::m_triplexMask;
	m_localFrame.m_right = m_localFrame.m_front.CrossProduct(m_localFrame.m_up).Normalize();
	m_localFrame.m_up = m_localFrame.m_right.CrossProduct(m_localFrame.m_front).Normalize();
}

ndMultiBodyVehicle::ndMultiBodyVehicle(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_localFrame(dGetIdentityMatrix())
	,m_chassis(nullptr)
	,m_motor(nullptr)
	,m_tireShape(new ndShapeChamferCylinder(ndFloat32(0.75f), ndFloat32(0.5f)))
	,m_gearBox(nullptr)
	,m_torsionBar(nullptr)
	,m_tireList()
	,m_extraBodiesAttachmentList()
	,m_axleList()
	,m_differentialList()
	,m_extraJointsAttachmentList()
	,m_downForce()
{
	m_tireShape->AddRef();

	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	for (const nd::TiXmlNode* node = xmlNode->FirstChild(); node; node = node->NextSibling())
	{
		const char* const partName = node->Value();

		if (strcmp(partName, "ndModel") == 0)
		{
			// do nothing
		}
		else if (strcmp(partName, "localFrame") == 0)
		{
			m_localFrame = xmlGetMatrix(xmlNode, "localFrame");
		} 
		else if (strcmp(partName, "chassis") == 0)
		{
			ndInt32 hash = xmlGetInt(xmlNode, "chassis");
			const ndBody* const body = desc.m_bodyMap->Find(hash)->GetInfo();
			m_chassis = ((ndBody*)body)->GetAsBodyDynamic();
		}
		else if (strcmp(partName, "tire") == 0)
		{
			ndInt32 hash;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hash);
			ndMultiBodyVehicleTireJoint* const tire = (ndMultiBodyVehicleTireJoint*)desc.m_jointMap->Find(hash)->GetInfo();
			tire->m_vehicle = this;
			m_tireList.Append(tire);
		}
		else if (strcmp(partName, "diff") == 0)
		{
			ndInt32 hash;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hash);
			ndMultiBodyVehicleDifferential* const tire = (ndMultiBodyVehicleDifferential*)desc.m_jointMap->Find(hash)->GetInfo();
			m_differentialList.Append(tire);
		}
		else if (strcmp(partName, "axle") == 0)
		{
			ndInt32 hash;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hash);
			ndMultiBodyVehicleDifferentialAxle* const tire = (ndMultiBodyVehicleDifferentialAxle*)desc.m_jointMap->Find(hash)->GetInfo();
			m_axleList.Append(tire);
		}
		else if (strcmp(partName, "motor") == 0)
		{
			ndInt32 hash = xmlGetInt(xmlNode, "motor");
			m_motor = (ndMultiBodyVehicleMotor*)desc.m_jointMap->Find(hash)->GetInfo();
			m_motor->m_vehicelModel = this;
		}
		else if (strcmp(partName, "gearBox") == 0)
		{
			ndInt32 hash = xmlGetInt(xmlNode, "gearBox");
			m_gearBox = (ndMultiBodyVehicleGearBox*)desc.m_jointMap->Find(hash)->GetInfo();
			m_gearBox->m_chassis = this;
		}
		else if (strcmp(partName, "torsionBar") == 0)
		{
			ndInt32 hash = xmlGetInt(xmlNode, "torsionBar");
			m_torsionBar = (ndMultiBodyVehicleTorsionBar*)desc.m_jointMap->Find(hash)->GetInfo();

			const nd::TiXmlNode* barNode = node->FirstChild();
			for (ndInt32 i = 0; i < m_torsionBar->m_axleCount; i++)
			{
				ndInt32 bodyHash0 = xmlGetInt(barNode, "bodyHash0");
				ndInt32 bodyHash1 = xmlGetInt(barNode, "bodyHash1");
				ndBody* const body0 = (ndBody*)desc.m_bodyMap->Find(bodyHash0)->GetInfo();
				ndBody* const body1 = (ndBody*)desc.m_bodyMap->Find(bodyHash1)->GetInfo();
				m_torsionBar->m_axles[i].m_leftTire = body0->GetAsBodyKinematic();
				m_torsionBar->m_axles[i].m_rightTire = body1->GetAsBodyKinematic();
				barNode = barNode->NextSibling();
			}
		}
		else if (strcmp(partName, "aerodynamics") == 0)
		{
			m_downForce.Load(node);
		}
		else if (strcmp(partName, "extraBody") == 0)
		{
			ndInt32 hash;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hash);
			const ndBody* const body = desc.m_bodyMap->Find(hash)->GetInfo();
			m_extraBodiesAttachmentList.Append(((ndBody*)body)->GetAsBodyDynamic());
		}
		else if (strcmp(partName, "extraJoint") == 0)
		{
			ndInt32 hash;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hash);
			const ndJointBilateralConstraint* const joint = desc.m_jointMap->Find(hash)->GetInfo();
			m_extraJointsAttachmentList.Append((ndJointBilateralConstraint*)joint);
		}
		else
		{
			dAssert(0);
		}
	}
}

ndMultiBodyVehicle::~ndMultiBodyVehicle()
{
	m_tireShape->Release();

	for (ndList<ndJointBilateralConstraint*>::ndNode* node = m_extraJointsAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		delete joint;
	}

	for (ndList<ndBodyDynamic*>::ndNode* node = m_extraBodiesAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyDynamic* const body = node->GetInfo();
		delete body;
	}

	if (m_torsionBar)
	{
		delete(m_torsionBar);
	}
	if (m_gearBox)
	{
		delete(m_gearBox);
	}
	if (m_motor)
	{
		ndBodyKinematic* const motorBody = m_motor->GetBody0();
		delete(m_motor);
		delete(motorBody);
	}
	for (ndList<ndMultiBodyVehicleDifferentialAxle*>::ndNode* node = m_axleList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferentialAxle* const joint = node->GetInfo();
		delete (joint);
	}
	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferential* const joint = node->GetInfo();
		ndBodyKinematic* const diffBody = joint->GetBody0();
		delete(joint);
		delete(diffBody);
	}
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const joint = node->GetInfo();
		ndBodyKinematic* const tireBody = joint->GetBody0();
		delete(joint);
		delete(tireBody);
	}
	delete(m_chassis);
}

void ndMultiBodyVehicle::AddToWorld(ndWorld* const world)
{
	world->AddBody(m_chassis);
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const joint = node->GetInfo();
		world->AddBody(joint->GetBody0());
		world->AddJoint(joint);
	}

	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferential* const joint = node->GetInfo();
		world->AddBody(joint->GetBody0());
		world->AddJoint(joint);
	}

	for (ndList<ndMultiBodyVehicleDifferentialAxle*>::ndNode* node = m_axleList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferentialAxle* const joint = node->GetInfo();
		world->AddJoint(joint);
	}

	for (ndList<ndBodyDynamic*>::ndNode* node = m_extraBodiesAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyDynamic* const body = node->GetInfo();
		world->AddBody(body);
	}

	for (ndList<ndJointBilateralConstraint*>::ndNode* node = m_extraJointsAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		world->AddJoint(joint);
	}

	if (m_motor)
	{
		world->AddBody(m_motor->GetBody0());
		world->AddJoint(m_motor);
	}

	if (m_gearBox)
	{
		world->AddJoint(m_gearBox);
	}

	if (m_torsionBar)
	{
		world->AddJoint(m_torsionBar);
	}
}

void ndMultiBodyVehicle::RemoveFromToWorld(ndWorld* const world)
{
	for (ndList<ndJointBilateralConstraint*>::ndNode* node = m_extraJointsAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		world->RemoveJoint(joint);
	}

	for (ndList<ndBodyDynamic*>::ndNode* node = m_extraBodiesAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyDynamic* const body = node->GetInfo();
		world->RemoveBody(body);
	}

	if (m_motor)
	{
		world->RemoveBody(m_motor->GetBody0());
	}

	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleDifferential* const joint = node->GetInfo();
		world->RemoveBody(joint->GetBody0());
	}
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const joint = node->GetInfo();
		world->RemoveBody(joint->GetBody0());
	}
	world->RemoveBody(m_chassis);
}

ndFloat32 ndMultiBodyVehicle::GetSpeed() const
{
	const ndVector dir(m_chassis->GetMatrix().RotateVector(m_localFrame.m_front));
	const ndFloat32 speed = dAbs(m_chassis->GetVelocity().DotProduct(dir).GetScalar());
	return speed;
}

void ndMultiBodyVehicle::AddChassis(ndBodyDynamic* const chassis)
{
	m_chassis = chassis;
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddAxleTire(const ndWheelDescriptor& desc, ndBodyDynamic* const tire, ndBodyDynamic* const axleBody)
{
	ndMatrix tireFrame(dGetIdentityMatrix());
	tireFrame.m_front = ndVector(0.0f, 0.0f, 1.0f, 0.0f);
	tireFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	tireFrame.m_right = ndVector(-1.0f, 0.0f, 0.0f, 0.0f);
	ndMatrix matrix(tireFrame * m_localFrame * axleBody->GetMatrix());
	matrix.m_posit = tire->GetMatrix().m_posit;

	// make tire inertia spherical
	ndVector inertia(tire->GetMassMatrix());
	ndFloat32 maxInertia(dMax(dMax(inertia.m_x, inertia.m_y), inertia.m_z));
	inertia.m_x = maxInertia;
	inertia.m_y = maxInertia;
	inertia.m_z = maxInertia;
	tire->SetMassMatrix(inertia);

	ndMultiBodyVehicleTireJoint* const tireJoint = new ndMultiBodyVehicleTireJoint(matrix, tire, axleBody, desc, this);
	m_tireList.Append(tireJoint);

	tire->SetDebugMaxAngularIntegrationSteepAndLinearSpeed(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(100.0f));
	return tireJoint;
}

ndMultiBodyVehicleTireJoint* ndMultiBodyVehicle::AddTire(const ndWheelDescriptor& desc, ndBodyDynamic* const tire)
{
	return AddAxleTire(desc, tire, m_chassis);
}

ndBodyDynamic* ndMultiBodyVehicle::CreateInternalBodyPart(ndFloat32 mass, ndFloat32 radius) const
{
	ndShapeInstance diffCollision(new ndShapeSphere(radius));
	diffCollision.SetCollisionMode(false);

	dAssert(m_chassis);
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetMatrix(m_localFrame * m_chassis->GetMatrix());
	body->SetCollisionShape(diffCollision);
	body->SetMassMatrix(mass, diffCollision);
	body->SetDebugMaxAngularIntegrationSteepAndLinearSpeed(ndFloat32(2.0f * 360.0f) * ndDegreeToRad, ndFloat32(100.0f));
	return body;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, ndFloat32 slipOmegaLock)
{
	ndBodyDynamic* const differentialBody = CreateInternalBodyPart(mass, radius);

	ndMultiBodyVehicleDifferential* const differential = new ndMultiBodyVehicleDifferential(differentialBody, m_chassis, slipOmegaLock);
	m_differentialList.Append(differential);

	ndVector pin0(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_front));
	ndVector upPin(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_up));
	ndVector leftPin1(leftTire->GetBody0()->GetMatrix().RotateVector(leftTire->GetLocalMatrix0().m_front));

	ndMultiBodyVehicleDifferentialAxle* const leftAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin, differentialBody, leftPin1, leftTire->GetBody0());
	m_axleList.Append(leftAxle);

	ndMultiBodyVehicleDifferentialAxle* const rightAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin.Scale (ndFloat32 (-1.0f)), differentialBody, leftPin1, rightTire->GetBody0());
	m_axleList.Append(rightAxle);

	return differential;
}

ndMultiBodyVehicleDifferential* ndMultiBodyVehicle::AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, ndFloat32 slipOmegaLock)
{
	ndBodyDynamic* const differentialBody = CreateInternalBodyPart(mass, radius);

	ndMultiBodyVehicleDifferential* const differential = new ndMultiBodyVehicleDifferential(differentialBody, m_chassis, slipOmegaLock);
	m_differentialList.Append(differential);

	ndVector pin0(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_front));
	ndVector upPin(differentialBody->GetMatrix().RotateVector(differential->GetLocalMatrix0().m_up));
	ndVector leftPin1(leftDifferential->GetBody0()->GetMatrix().RotateVector(leftDifferential->GetLocalMatrix0().m_front));
	leftPin1 = leftPin1.Scale(ndFloat32(-1.0f));

	ndMultiBodyVehicleDifferentialAxle* const leftAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin, differentialBody, leftPin1, leftDifferential->GetBody0());
	m_axleList.Append(leftAxle);

	ndMultiBodyVehicleDifferentialAxle* const rightAxle = new ndMultiBodyVehicleDifferentialAxle(pin0, upPin.Scale(ndFloat32(-1.0f)), differentialBody, leftPin1, rightDifferential->GetBody0());
	m_axleList.Append(rightAxle);

	return differential;
}

void ndMultiBodyVehicle::AddExtraBody(ndBodyDynamic* const body)
{
	m_extraBodiesAttachmentList.Append(body);
}

void ndMultiBodyVehicle::AddExtraJoint(ndJointBilateralConstraint* const joint)
{
	m_extraJointsAttachmentList.Append(joint);
}

ndMultiBodyVehicleMotor* ndMultiBodyVehicle::AddMotor(ndFloat32 mass, ndFloat32 radius)
{
	ndBodyDynamic* const motorBody = CreateInternalBodyPart(mass, radius);
	m_motor = new ndMultiBodyVehicleMotor(motorBody, this);
	return m_motor;
}

ndMultiBodyVehicleGearBox* ndMultiBodyVehicle::AddGearBox(ndMultiBodyVehicleMotor* const motor, ndMultiBodyVehicleDifferential* const differential)
{
	dAssert(m_motor == motor);
	m_gearBox = new ndMultiBodyVehicleGearBox(motor->GetBody0(), differential->GetBody0(), this);
	return m_gearBox;
}

ndMultiBodyVehicleTorsionBar* ndMultiBodyVehicle::AddTorsionBar(ndBodyDynamic* const sentinel)
{
	m_torsionBar = new ndMultiBodyVehicleTorsionBar(this, sentinel);
	return m_torsionBar;
}

ndShapeInstance ndMultiBodyVehicle::CreateTireShape(ndFloat32 radius, ndFloat32 width) const
{
	ndShapeInstance tireCollision(m_tireShape);
	ndVector scale(2.0f * width, radius, radius, 0.0f);
	tireCollision.SetScale(scale);
	return tireCollision;
}

void ndMultiBodyVehicle::ApplyAerodynamics()
{
	m_downForce.m_suspensionStiffnessModifier = ndFloat32(1.0f);
	ndFloat32 gravity = m_downForce.GetDownforceFactor(GetSpeed());
	if (dAbs (gravity) > ndFloat32(1.0e-2f))
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
			ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
			const ndVector tireWeight(tireBody->GetForce());
			const ndVector tireDownForce(up.Scale(gravity * tireBody->GetMassMatrix().m_w));
			tireBody->SetForce(tireWeight + tireDownForce);
		}
	}
}

void ndMultiBodyVehicle::SetVehicleSolverModel(bool hardJoint)
{
	ndJointBilateralSolverModel openLoopMode = hardJoint ? m_jointkinematicOpenLoop : m_jointIterativeSoft;

	dAssert(m_chassis);
	const ndJointList& chassisJoints = m_chassis->GetJointList();
	for (ndJointList::ndNode* node = chassisJoints.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		const char* const className = joint->ClassName();
		if (!strcmp(className, "ndMultiBodyVehicleTireJoint") ||
			!strcmp(className, "ndMultiBodyVehicleDifferential") ||
			!strcmp(className, "ndMultiBodyVehicleMotor"))
		{
			joint->SetSolverModel(openLoopMode);
		}
	}
	
	ndJointBilateralSolverModel driveTrainMode = hardJoint ? m_jointkinematicCloseLoop : m_jointIterativeSoft;
	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		ndJointBilateralConstraint* const joint = node->GetInfo();
		const ndJointList& jointList = joint->GetBody0()->GetJointList();
		for (ndJointList::ndNode* node1 = jointList.GetFirst(); node1; node1 = node1->GetNext())
		{
			ndJointBilateralConstraint* const axle = node1->GetInfo();
			const char* const clasName = axle->ClassName();
			if (strcmp(clasName, "ndMultiBodyVehicleDifferential"))
			{
				axle->SetSolverModel(driveTrainMode);
			}
		}
	}

	if (m_torsionBar)
	{
		m_torsionBar->SetSolverModel(driveTrainMode);
	}
}

void ndMultiBodyVehicle::ApplyAligmentAndBalancing()
{
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
		ndBodyDynamic* const chassisBody = tire->GetBody1()->GetAsBodyDynamic();

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
	// draw vehicle cordinade system;
	ndMatrix chassisMatrix(m_chassis->GetMatrix());
	chassisMatrix.m_posit = chassisMatrix.TransformVector(m_chassis->GetCentreOfMass());
	//context.DrawFrame(chassisMatrix);

	ndFloat32 totalMass = m_chassis->GetMassMatrix().m_w;
	ndVector effectiveCom(chassisMatrix.m_posit.Scale(totalMass));

	// draw front direction for side slip angle reference
	ndVector p0(chassisMatrix.m_posit + m_localFrame.m_up.Scale(1.0f));
	ndVector p1(p0 + chassisMatrix.RotateVector(m_localFrame.m_front).Scale(0.5f));
	context.DrawLine(p0, p1, ndVector(1.0f, 1.0f, 1.0f, 0.0f));

	// draw velocity vector
	ndVector veloc(m_chassis->GetVelocity());
	ndVector p2(p0 + veloc.Scale (0.25f));
	context.DrawLine(p0, p2, ndVector(1.0f, 1.0f, 0.0f, 0.0f));

	// draw body acceleration
	//ndVector accel(m_chassis->GetAccel());
	//ndVector p3(p0 + accel.Scale(0.5f));
	//context.DrawLine(p0, p3, ndVector(0.0f, 1.0f, 1.0f, 0.0f));

	ndVector weight(m_chassis->GetForce());
	ndFloat32 scale = ndSqrt(weight.DotProduct(weight).GetScalar());
	weight = weight.Normalize().Scale(-2.0f);

	// draw vehicle weight;
	ndVector forceColor(ndFloat32 (0.8f), ndFloat32(0.8f), ndFloat32(0.8f), ndFloat32(0.0f));
	ndVector lateralColor(ndFloat32(0.3f), ndFloat32(0.7f), ndFloat32(0.0f), ndFloat32(0.0f));
	ndVector longitudinalColor(ndFloat32(0.7f), ndFloat32(0.3f), ndFloat32(0.0f), ndFloat32(0.0f));
	context.DrawLine(chassisMatrix.m_posit, chassisMatrix.m_posit + weight, forceColor);

	const ndFloat32 tireForceScale = ndFloat32(3.0f);
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tireJoint = node->GetInfo();
		ndBodyDynamic* const tireBody = tireJoint->GetBody0()->GetAsBodyDynamic();

		// draw upper bumper
		ndMatrix upperBumberMatrix(tireJoint->CalculateUpperBumperMatrix());
		//context.DrawFrame(tireJoint->CalculateUpperBumperMatrix());

		// show tire center of mass;
		ndMatrix tireFrame(tireBody->GetMatrix());
		//context.DrawFrame(tireFrame);
		upperBumberMatrix.m_posit = tireFrame.m_posit;
		//context.DrawFrame(upperBumberMatrix);

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
				for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					ndMatrix frame(contactPoint.m_normal, contactPoint.m_dir0, contactPoint.m_dir1, contactPoint.m_point);

					ndVector localPosit(m_localFrame.UntransformVector(chassisMatrix.UntransformVector(contactPoint.m_point)));
					ndFloat32 offset = (localPosit.m_z > ndFloat32(0.0f)) ? ndFloat32(0.2f) : ndFloat32(-0.2f);
					frame.m_posit += contactPoint.m_dir0.Scale(offset);
					frame.m_posit += contactPoint.m_normal.Scale(0.1f);

					// normal force
					ndFloat32 normalForce = tireForceScale * contactPoint.m_normal_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_normal.Scale (normalForce), forceColor);

					// lateral force
					ndFloat32 lateralForce = -tireForceScale * contactPoint.m_dir0_Force.m_force / scale;
					context.DrawLine(frame.m_posit, frame.m_posit + contactPoint.m_dir0.Scale(lateralForce), lateralColor);

					// longitudinal force
					ndFloat32 longitudinalForce = -tireForceScale * contactPoint.m_dir1_Force.m_force / scale;
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

void ndMultiBodyVehicle::BrushTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const
{
	// calculate longitudinal slip ratio
	const ndBodyDynamic* const tireBody = tire->GetBody0()->GetAsBodyDynamic();
	const ndBodyDynamic* const otherBody = (contactPoint.m_body0 == tireBody) ? ((ndBodyKinematic*)contactPoint.m_body1)->GetAsBodyDynamic() : ((ndBodyKinematic*)contactPoint.m_body0)->GetAsBodyDynamic();
	dAssert(tireBody != otherBody);
	dAssert((tireBody == contactPoint.m_body0) || (tireBody == contactPoint.m_body1));

	const ndVector tireVeloc(tireBody->GetVelocity());
	const ndFloat32 tireSpeed = dAbs(tireVeloc.DotProduct(contactPoint.m_dir1).GetScalar());
	// tire non linear brush model is only considered 
	// when is moving faster than 3 mph 
	// (this is just an arbitrary limit, based of the model 
	// not been defined for stationary tires.
	if (dAbs(tireSpeed) > ndFloat32(0.9f))
	{
		// apply brush tire model only when center travels faster that 10 miles per hours
		const ndVector contactVeloc0(tireBody->GetVelocityAtPoint(contactPoint.m_point) - tireBody->GetVelocity() + tireVeloc);
		const ndVector contactVeloc1(otherBody->GetVelocityAtPoint(contactPoint.m_point));
		const ndVector relVeloc(contactVeloc0 - contactVeloc1);

		const ndFloat32 relSpeed = dAbs(relVeloc.DotProduct(contactPoint.m_dir1).GetScalar());
		const ndFloat32 longitudialSlip = relSpeed / tireSpeed;

		// calculate side slip ratio
		const ndFloat32 sideSpeed = dAbs(relVeloc.DotProduct(contactPoint.m_dir0).GetScalar());
		const ndFloat32 lateralSlip = sideSpeed / (relSpeed + ndFloat32(1.0f));

		const ndFloat32 den = ndFloat32(1.0f) / (ndFloat32(1.0f) + longitudialSlip);
		const ndFloat32 v = lateralSlip * den;
		const ndFloat32 u = longitudialSlip * den;

		//if (u > 0.5f || v > 0.5f)
		//dTrace(("(%d %f %f)\n", tireBody->GetId(), u, v));
		tire->m_lateralSlip = dMax (tire->m_lateralSlip, v);
		tire->m_longitudinalSlip = dMax(tire->m_longitudinalSlip, u);

		const ndWheelDescriptor& info = tire->GetInfo();
		const ndFloat32 cz = info.m_laterialStiffness  * v;
		const ndFloat32 cx = info.m_longitudinalStiffness  * u;
		const ndFloat32 gamma = dMax (ndSqrt(cx * cx + cz * cz), ndFloat32(1.0e-8f));

		const ndFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
		const ndFloat32 lateralFrictionCoefficient = frictionCoefficient * cz / gamma;
		const ndFloat32 longitudinalFrictionCoefficient = frictionCoefficient * cx / gamma;
		//dTrace(("(%d %f %f) ", tireBody->GetId(), lateralFrictionCoefficient, longitudinalFrictionCoefficient));

		contactPoint.m_material.m_restitution = ndFloat32(0.1f);
		contactPoint.m_material.m_staticFriction0 = lateralFrictionCoefficient;
		contactPoint.m_material.m_dynamicFriction0 = lateralFrictionCoefficient;
		contactPoint.m_material.m_staticFriction1 = longitudinalFrictionCoefficient;
		contactPoint.m_material.m_dynamicFriction1 = longitudinalFrictionCoefficient;
	}
	else
	{
		// at low speed the tire act like a normal rigid body
		const ndFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
		contactPoint.m_material.m_restitution = ndFloat32(0.1f);
		contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
		contactPoint.m_material.m_staticFriction1 = frictionCoefficient;

		contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient;
		contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient;
	}
}

void ndMultiBodyVehicle::ApplyTireModel()
{
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		ndMultiBodyVehicleTireJoint* const tire = node->GetInfo();
		dAssert(((ndShape*)tire->GetBody0()->GetCollisionShape().GetShape())->GetAsShapeChamferCylinder());

		tire->m_lateralSlip = ndFloat32(0.0f);
		tire->m_longitudinalSlip = ndFloat32(0.0f);

		const ndBodyKinematic::ndContactMap& contactMap = tire->GetBody0()->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				ndContactPointList& contactPoints = contact->GetContactPoints();
				const ndBodyKinematic* const otherBody = contact->GetBody1();

				if (((ndShape*)otherBody->GetCollisionShape().GetShape())->GetAsShapeStaticMesh())
				{
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
				}

				ndMatrix tireBasisMatrix (tire->GetLocalMatrix1() * tire->GetBody1()->GetMatrix());
				tireBasisMatrix.m_posit = tire->GetBody0()->GetMatrix().m_posit;
				for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					ndContactMaterial& contactPoint = contactNode->GetInfo();
					ndFloat32 contactPathLocation = dAbs (contactPoint.m_normal.DotProduct(tireBasisMatrix.m_front).GetScalar());
					// contact are consider on the contact patch strip only if the are less than 
					// 45 degree angle from the tire axle
					if (contactPathLocation < ndFloat32 (0.71f))
					{
						// align tire friction direction
						const ndVector fronDir(contactPoint.m_normal.CrossProduct(tireBasisMatrix.m_front));
						contactPoint.m_dir1 = fronDir.Normalize();
						contactPoint.m_dir0 = contactPoint.m_dir1.CrossProduct(contactPoint.m_normal);
						
						// check if the contact is in the contact patch,
						// the is the 45 degree point around the tire vehicle axis. 
						ndVector dir(contactPoint.m_point - tireBasisMatrix.m_posit);
						dAssert(dir.DotProduct(dir).GetScalar() > ndFloat32(0.0f));
						ndFloat32 contactPatch = tireBasisMatrix.m_up.DotProduct(dir.Normalize()).GetScalar();
						if (contactPatch < ndFloat32(-0.71f))
						{
							BrushTireModel(tire, contactPoint);
						}
						else
						{
							const ndFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
							contactPoint.m_material.m_restitution = ndFloat32(0.1f);
							contactPoint.m_material.m_staticFriction0 = frictionCoefficient;
							contactPoint.m_material.m_staticFriction1 = frictionCoefficient;

							contactPoint.m_material.m_dynamicFriction0 = frictionCoefficient;
							contactPoint.m_material.m_dynamicFriction1 = frictionCoefficient;
						}
					}
					else
					{
						const ndFloat32 frictionCoefficient = GetFrictionCoeficient(tire, contactPoint);
						contactPoint.m_material.m_restitution = ndFloat32(0.1f);
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
	:m_gravity(ndFloat32(-10.0f))
	, m_suspensionStiffnessModifier(ndFloat32(1.0f))
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

#if 0
	dFloat32 speed = 0;
	for (dInt32 i = 0; i < 100; i++)
	{
		dTrace(("%f %f\n", speed, GetDownforceFactor(speed) / m_gravity));
		speed += (150.0f / 100.0f) * 0.27f;
	}
#endif
}

ndFloat32 ndMultiBodyVehicle::ndDownForce::CalculateFactor(const ndSpeedForcePair* const entry0) const
{
	const ndSpeedForcePair* const entry1 = entry0 + 1;
	ndFloat32 num = dMax(entry1->m_forceFactor - entry0->m_forceFactor, ndFloat32(0.0f));
	ndFloat32 den = dMax(dAbs(entry1->m_speed - entry0->m_speed), ndFloat32(1.0f));
	return num / (den * den);
}

ndFloat32 ndMultiBodyVehicle::ndDownForce::GetDownforceFactor(ndFloat32 speed) const
{
	dAssert(speed >= ndFloat32(0.0f));
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

void ndMultiBodyVehicle::ndDownForce::Save(nd::TiXmlNode* const parentNode) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement("aerodynamics");
	parentNode->LinkEndChild(childNode);

	xmlSaveParam(childNode, "gravity", m_gravity);
	for (ndInt32 i = 0; i < ndInt32 (sizeof(m_downForceTable) / sizeof(m_downForceTable[0])); i++)
	{
		ndVector nod(m_downForceTable[i].m_speed, m_downForceTable[i].m_forceFactor, m_downForceTable[i].m_aerodynamicDownforceConstant, ndFloat32(0.0f));
		xmlSaveParam(childNode, "downforceCurve", nod);
	}
}

void ndMultiBodyVehicle::ndDownForce::Load(const nd::TiXmlNode* const xmlNode)
{
	m_gravity = xmlGetFloat(xmlNode, "gravity");
	const nd::TiXmlNode* node = xmlNode->FirstChild();
	for (ndInt32 i = 0; i < ndInt32 (sizeof(m_downForceTable) / sizeof(m_downForceTable[0])); i++)
	{
		node = node->NextSibling();
		const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
		const char* const data = element->Attribute("float3");

		ndFloat64 fx;
		ndFloat64 fy;
		ndFloat64 fz;
		sscanf(data, "%lf %lf %lf", &fx, &fy, &fz);
		m_downForceTable[i].m_speed = ndFloat32(fx);
		m_downForceTable[i].m_forceFactor = ndFloat32(fy);
		m_downForceTable[i].m_aerodynamicDownforceConstant = ndFloat32(fz);
	}
}

void ndMultiBodyVehicle::PostUpdate(ndWorld* const, ndFloat32)
{
	ApplyAligmentAndBalancing();
}

void ndMultiBodyVehicle::Update(ndWorld* const world, ndFloat32 timestep)
{
	ApplyInputs(world, timestep);

	// apply down force
	ApplyAerodynamics();
	// apply tire model
	ApplyTireModel();

	// Apply Vehicle Dynamics controls
	// no implemented yet
}

void ndMultiBodyVehicle::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndModel::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "localFrame", m_localFrame);
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("chassis");
		childNode->LinkEndChild(paramNode);

		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode = desc.m_bodyMap->Find(m_chassis);
		if (!bodyNode)
		{
			bodyNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_chassis);
		}
		dAssert(bodyNode);
		paramNode->SetAttribute("int32", bodyNode->GetInfo());
	}

	// save all wheels
	for (ndList<ndMultiBodyVehicleTireJoint*>::ndNode* node = m_tireList.GetFirst(); node; node = node->GetNext())
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("tire");
		childNode->LinkEndChild(paramNode);

		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode = desc.m_bodyMap->Find(node->GetInfo()->GetBody0());
		if (!bodyNode)
		{
			bodyNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), node->GetInfo()->GetBody0());
		}

		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(node->GetInfo());
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), node->GetInfo());
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());
	}

	// save all differentials
	for (ndList<ndMultiBodyVehicleDifferential*>::ndNode* node = m_differentialList.GetFirst(); node; node = node->GetNext())
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("diff");
		childNode->LinkEndChild(paramNode);

		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode = desc.m_bodyMap->Find(node->GetInfo()->GetBody0());
		if (!bodyNode)
		{
			bodyNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), node->GetInfo()->GetBody0());
		}

		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(node->GetInfo());
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), node->GetInfo());
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());
	}

	// save all axles
	for (ndList<ndMultiBodyVehicleDifferentialAxle*>::ndNode* node = m_axleList.GetFirst(); node; node = node->GetNext())
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("axle");
		childNode->LinkEndChild(paramNode);
		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(node->GetInfo());
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), node->GetInfo());
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());
	}

	if (m_motor)
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("motor");
		childNode->LinkEndChild(paramNode);

		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode = desc.m_bodyMap->Find(m_motor->GetBody0());
		if (!bodyNode)
		{
			bodyNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_motor->GetBody0());
		}

		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(m_motor);
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_motor);
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());
	}

	for (ndList<ndBodyDynamic*>::ndNode* node = m_extraBodiesAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("extraBody");
		childNode->LinkEndChild(paramNode);

		const ndBodyKinematic* const body = node->GetInfo();
		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode = desc.m_bodyMap->Find(body);
		if (!bodyNode)
		{
			bodyNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), body);
		}
		dAssert(bodyNode);
		paramNode->SetAttribute("int32", bodyNode->GetInfo());
	}

	for (ndList<ndJointBilateralConstraint*>::ndNode* node = m_extraJointsAttachmentList.GetFirst(); node; node = node->GetNext())
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("extraJoint");
		childNode->LinkEndChild(paramNode);

		const ndBodyKinematic* const body0 = node->GetInfo()->GetBody0();
		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode0 = desc.m_bodyMap->Find(body0);
		if (!bodyNode0)
		{
			desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), body0);
		}

		const ndBodyKinematic* const body1 = node->GetInfo()->GetBody1();
		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* bodyNode1 = desc.m_bodyMap->Find(body1);
		if (!bodyNode1)
		{
			desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), body1);
		}

		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(node->GetInfo());
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), node->GetInfo());
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());
	}

	if (m_gearBox)
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("gearBox");
		childNode->LinkEndChild(paramNode);

		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(m_gearBox);
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_gearBox);
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());
	}

	if (m_torsionBar)
	{
		nd::TiXmlElement* const paramNode = new nd::TiXmlElement("torsionBar");
		childNode->LinkEndChild(paramNode);

		dAssert(m_torsionBar->GetBody1()->GetAsBodySentinel());
		ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* jointNode = desc.m_jointMap->Find(m_torsionBar);
		if (!jointNode)
		{
			jointNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_torsionBar);
		}
		dAssert(jointNode);
		paramNode->SetAttribute("int32", jointNode->GetInfo());

		for (ndInt32 i = 0; i < m_torsionBar->m_axleCount; i++)
		{
			nd::TiXmlElement* const barNode = new nd::TiXmlElement("barAxle");
			paramNode->LinkEndChild(barNode);

			ndMultiBodyVehicleTorsionBar::ndAxles& axle = m_torsionBar->m_axles[i];
			ndInt32 bodyHash0 = ndInt32(desc.m_bodyMap->Find(axle.m_leftTire)->GetInfo());
			ndInt32 bodyHash1 = ndInt32(desc.m_bodyMap->Find(axle.m_rightTire)->GetInfo());
			xmlSaveParam(barNode, "bodyHash0", bodyHash0);
			xmlSaveParam(barNode, "bodyHash1", bodyHash1);
		}
	}

	m_downForce.Save(childNode);
}