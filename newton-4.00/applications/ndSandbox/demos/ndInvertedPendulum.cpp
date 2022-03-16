/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequencePlayer.h"

#define D_USE_FORWARD_DYNAMICS

class dInvertedPendulum : public ndModel
{
	public:
	D_CLASS_REFLECTION(dInvertedPendulum);

	dInvertedPendulum(ndDemoEntityManager* const scene, const ndMatrix& location)
		:ndModel()
		,m_gravityDir(0.0f, -1.0f, 0.0f, 0.0f)
		,m_solver()
		,m_bodies()
		,m_effector(nullptr)
		,m_contactSensor(nullptr)
	{
		ndPhysicsWorld* const world = scene->GetWorld();
		ndVector floor(FindFloor(*world, location.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		ndMatrix matrix(location);
		matrix.m_posit.m_y += floor.m_y + 1.05f;
		//matrix.m_posit.m_y += floor.m_y + 1.1f;
		
		ndFloat32 mass = 1.0f;
		ndFloat32 size = 0.5f;
		ndFloat32 radius = 0.125f * size;
		ndFloat32 lenght = 2.0f * size;
		
		ndBodyKinematic* const box = AddBox(scene, matrix, mass, size, size, size);
		//ndBodyKinematic* const leg = AddCapsule(scene, matrix, mass / 20.0f, radius, radius, 2.0f * size);
		ndBodyKinematic* const sph = AddSphere(scene, matrix, mass / 30.0f, radius);

		// add a dummy object to force to make a skeleton
		//ndBodyKinematic* const dummySphere = AddSphere(scene, matrix, mass / 30.0f, radius);
		//dummySphere->SetMatrix(matrix);
		//world->AddJoint(new ndJointFix6dof(matrix, dummySphere, box));

		//ndMatrix legMatrix(dRollMatrix(90.0f * ndDegreeToRad) * box->GetMatrix());
		//legMatrix.m_posit.m_y -= lenght * 0.5f;
		//leg->SetMatrix(legMatrix);

		ndMatrix sphMatrix(box->GetMatrix());
		sphMatrix.m_posit.m_y -= lenght;

		// try offsetting the effector.
		sphMatrix.m_posit.m_z -= (size * 0.5f) * 0.0f;

		sph->SetMatrix(sphMatrix);
		sph->GetNotifyCallback()->OnTransform(0, sphMatrix);
		sph->GetNotifyCallback()->OnTransform(0, sphMatrix);
		//sph->GetCollisionShape().SetCollisionMode(false);
		//ndIkJointSpherical* const feetJoint = new ndIkJointSpherical(sphMatrix, sph, leg);
		//world->AddJoint(feetJoint);

		//ndMatrix legSocketMatrix(legMatrix);
		//legSocketMatrix.m_posit = matrix.m_posit;
		//ndIkJointSpherical* const socketJoint = new ndIkJointSpherical(legSocketMatrix, leg, box);
		//world->AddJoint(socketJoint);

		ndMatrix sphMatrixPivot(box->GetMatrix());
		sphMatrixPivot.m_posit.m_z = sphMatrix.m_posit.m_z;
		m_effector = new ndIk6DofEffector(sphMatrix, sphMatrixPivot, sph, box);
		ndFloat32 regularizer = 1.0e-2f;
		m_effector->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
		m_effector->SetLinearSpringDamper(regularizer, 1500.0f, 100.0f);
		m_effector->SetAngularSpringDamper(regularizer, 1500.0f, 100.0f);
		m_effector->SetSolverModel(ndJointBilateralSolverModel::m_jointkinematicOpenLoop);

		//feetJoint->SetIkMode(false);
		//socketJoint->SetIkMode(false);
		//world->AddJoint(new ndJointPlane(matrix.m_posit, matrix.m_front, box, world->GetSentinelBody()));

		m_bodies.PushBack(box->GetAsBodyDynamic());
		m_bodies.PushBack(sph->GetAsBodyDynamic());
		//m_rootBody = box->GetAsBodyDynamic();
		m_contactSensor = sph->GetAsBodyDynamic();

		#ifdef D_USE_FORWARD_DYNAMICS
			world->AddJoint(m_effector);
		#endif
	}

	dInvertedPendulum(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
	{
		dAssert(0);
	}

	~dInvertedPendulum()
	{
		if (m_effector && !m_effector->IsInWorld())
		{
			delete m_effector;
		}
	}

	//void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
	void Save(const ndLoadSaveBase::ndSaveDescriptor&) const
	{
		dAssert(0);
	}

	void Debug(ndConstraintDebugCallback& context) const
	{
		ndJointBilateralConstraint* const joint = m_effector;
		joint->DebugJoint(context);
		ndMatrix rootMatrix(dGetIdentityMatrix());

		ndVector com(CalculateCenterOfMass());
		rootMatrix.m_posit = com;
		//context.DrawFrame(rootMatrix);

		ndVector p1(com + m_gravityDir.Scale (1.0f));
		context.DrawLine(com, p1, ndVector(0.0f, 1.0f, 1.0f, 0.0f));
	}

	void PostUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostUpdate(world, timestep);
	}

	void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostTransformUpdate(world, timestep);
	}

	//void CalculateCenterOfMass(ndVector& com, ndVector& comVeloc) const
	//{
	//	ndFloat32 toltalMass = 0.0f;
	//	com = ndVector::m_zero;
	//	comVeloc = ndVector::m_zero;
	//	for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
	//	{
	//		ndBodyDynamic* const body = m_bodies[i];
	//		ndFloat32 mass = body->GetMassMatrix().m_w;
	//		ndVector comMass(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
	//		com += comMass.Scale(mass);
	//		comVeloc += body->GetVelocity().Scale(mass);
	//		toltalMass += mass;
	//	}
	//	com = com.Scale(1.0f / toltalMass) & ndVector::m_triplexMask;
	//	comVeloc = comVeloc.Scale(1.0f / toltalMass) & ndVector::m_triplexMask;;
	//}

	ndVector CalculateCenterOfMass() const
	{
		ndFloat32 toltalMass = 0.0f;
		ndVector com (ndVector::m_zero);
		//comVeloc = ndVector::m_zero;
		for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
		{
			ndBodyDynamic* const body = m_bodies[i];
			ndFloat32 mass = body->GetMassMatrix().m_w;
			ndVector comMass(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
			com += comMass.Scale(mass);
			//comVeloc += body->GetVelocity().Scale(mass);
			toltalMass += mass;
		}
		com = com.Scale(1.0f / toltalMass) & ndVector::m_triplexMask;
		//comVeloc = comVeloc.Scale(1.0f / toltalMass) & ndVector::m_triplexMask;;
		return com | ndVector::m_wOne;
	}

	void Update(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::Update(world, timestep);
		if (m_contactSensor)
		{
			#ifdef D_USE_FORWARD_DYNAMICS
			UpdateFK(world, timestep);
			#else
			UpdateIK(world, timestep);
			#endif
		}
	}

	//void UpdateIK(ndWorld* const world, ndFloat32 timestep)
	void UpdateIK(ndWorld* const, ndFloat32)
	{
		//ndSkeletonContainer* const skeleton = m_rootBody->GetSkeleton();
		//dAssert(skeleton);
		//
		//m_rootBody->SetSleepState(false);
		//
		//ndJointBilateralConstraint* effector = m_effector;
		//m_solver.SolverBegin(skeleton, &effector, 1, world, timestep);
		//m_solver.Solve();
		//m_solver.SolverEnd();
	}

	//void UpdateFK(ndWorld* const world, ndFloat32 timestep)
	void UpdateFK(ndWorld* const, ndFloat32)
	{
		// step 1: see if we have a support contacts
		ndBodyKinematic::ndContactMap::Iterator it(m_contactSensor->GetContactMap());
		bool hasSupport = false;
		for (it.Begin(); !hasSupport && it; it++)
		{
			const ndContact* const contact = it.GetNode()->GetInfo();
			hasSupport = hasSupport | contact->IsActive();
		}

		// step 2: we apply correction only if there is a support contact
		if (!hasSupport)
		{
			return;
		}

		// step 3: have support contacts, find the projection of the com over the ground
		//const ndVector com(CalculateCenterOfMass());

		//// step 4: with the com find the projection to the ground
		//class rayCaster : public ndConvexCastNotify
		//{
		//	public:
		//	rayCaster(dInvertedPendulum* const owner)
		//		:m_owner(owner)
		//	{
		//	}
		//
		//	ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const)
		//	{
		//		for (ndInt32 i = 0; i < m_owner->m_bodies.GetCount(); ++i)
		//		{
		//			if (body == m_owner->m_bodies[i])
		//			{
		//				return 0;
		//			}
		//		}
		//		return 1;
		//	}
		//
		//	dInvertedPendulum* m_owner;
		//};
		//
		//rayCaster caster(this);
		//const ndShapeInstance& shape = m_contactSensor->GetCollisionShape();
		//ndMatrix matrix(m_bodies[0]->GetMatrix());
		//matrix.m_posit = com | ndVector::m_wOne;
		//bool hit = world->ConvexCast(caster, shape, matrix, matrix.m_posit - ndVector(0.0f, 1.0f, 0.0f, 0.0f));
		//if (!hit)
		//{
		//	return;
		//}
		
		// step 5: her we have a com support point, the com, com velocity
		// need calculate translation distance from current contact to the com support contact
		//ndSkeletonContainer* const skeleton = m_bodies[0]->GetSkeleton();
		
static int xxx;
xxx++;

		ndJointBilateralConstraint* joint = m_effector;

//if (xxx >= 17)
if (xxx == 200)
{
m_bodies[0]->SetVelocity(ndVector(0.0f, 0.0f, 0.5f, 0.0f));
}

		ndMatrix matrix0;
		ndMatrix matrix1;
		joint->CalculateGlobalMatrix(matrix0, matrix1);

		ndVector localGravity(matrix1.UnrotateVector(m_gravityDir));
		ndMatrix targetMatrix(m_effector->GetOffsetMatrix());
		//ndMatrix targetMatrix(dGetIdentityMatrix() * matrix1.Inverse());
		targetMatrix.m_posit = localGravity.Scale(1.0f) | ndVector::m_wOne;
if (xxx > 200)
{
	xxx *= 1;
}

		m_effector->SetOffsetMatrix(targetMatrix);

		//m_solver.SolverBegin(skeleton, &joint, 1, world, timestep);
		//m_solver.Solve();
		//m_solver.SolverEnd();
	}

	ndVector m_gravityDir;
	ndIkSolver m_solver;
	ndFixSizeArray<ndBodyDynamic*, 16> m_bodies;
	ndBodyDynamic* m_contactSensor;
	ndIk6DofEffector* m_effector;
};
D_CLASS_REFLECTION_IMPLEMENT_LOADER(dInvertedPendulum);

void ndInvertedPendulum(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(dYawMatrix(-0.0f * ndDegreeToRad));

	dInvertedPendulum* const robot0 = new dInvertedPendulum(scene, matrix);
	scene->SetSelectedModel(robot0);
	world->AddModel(robot0);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//dInvertedPendulum* const robot1 = new dInvertedPendulum(scene, robotEntity, matrix);
	//world->AddModel(robot1);

	//ndVector posit(matrix.m_posit);
	//posit.m_x += 1.5f;
	//posit.m_z += 1.5f;
	//AddBox(scene, posit, 2.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, posit, 1.0f, 0.3f, 0.4f, 0.7f);

	//posit.m_x += 0.6f;
	//posit.m_z += 0.2f;
	//AddBox(scene, posit, 8.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, posit, 4.0f, 0.3f, 0.4f, 0.7f);

	//world->AddJoint(new ndJointFix6dof(robot0->GetRoot()->GetMatrix(), robot0->GetRoot(), world->GetSentinelBody()));

	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
