/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndUIEntity.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndController_0
{
	#define ND_ALPHA_TOL ndFloat32 (1.0e-3f)
	class ndModelUnicycle : public ndModelArticulation
	{
		public:
		ndModelUnicycle()
			:ndModelArticulation()
			,m_invInertia(ndGetZeroMatrix())
			,m_com(ndVector::m_zero)
			,m_comVel(ndVector::m_zero)
			,m_gyroTorque(ndVector::m_zero)
			,m_comDist()
			,m_bodies()
			,m_ballBody(nullptr)
			,m_controlJoint(nullptr)
			,m_totalMass(ndFloat32 (0.0f))
			,m_hasSupport(false)
		{
			xxx = 0;
		}

		void Init()
		{
			m_totalMass = ndFloat32(0.0f);
			for (ndNode* node = m_rootNode->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				m_bodies.PushBack(body);
				m_totalMass += body->GetMassMatrix().m_w;
			}
			m_comDist.SetCount(m_bodies.GetCount());
		}

		void InitState(ndWorld* const world)
		{
			//a) Mt = sum(M(i))
			//b) cg = sum(p(i) * M(i)) / Mt
			//c) Vcg = sum(v(i) * M(i)) / Mt
			//d) Icg = sum(I(i) + covarianMatrix(p(i) - cg) * m(i))
			//e) T0 = sum[w(i) x (I(i) * w(i)) - Vcg x(m(i) * V(i))]
			//f) T1 = sum[(p(i) - cg) x Fext(i) + Text(i)]
			//g) Bcg = (Icg ^ -1) * (T0 + T1)

			//a) Mt = sum(m(i))
			//b) cg = sum(p(i) * m(i)) / Mt
			//d) Icg = sum(I(i) + covarianMatrix(p(i) - cg) * m(i))
			//e) T0 = sum(w(i) x (I(i) * w(i))
			//f) T1 = sum[(p(i) - cg) x Fext(i) + Text(i)]
			//g) Bcg = (Icg ^ -1) * (T0 + T1)

			ndVector cg(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				const ndBodyDynamic* const body = m_bodies[i];

				const ndMatrix matrix (body->GetMatrix());
				ndVector veloc(body->GetVelocity());
				cg += matrix.m_posit.Scale (body->GetMassMatrix().m_w);
			}
			m_com = cg.Scale(ndFloat32(1.0f) / m_totalMass);

			ndMatrix inertia(ndGetZeroMatrix());
			ndVector gyroTorque(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				const ndBodyDynamic* const body = m_bodies[i];

				const ndMatrix matrix(body->GetMatrix());
				const ndVector omega(body->GetOmega());
				const ndVector comDist(matrix.m_posit - m_com);
				ndFloat32 mass = body->GetMassMatrix().m_w;

				m_comDist[i] = comDist;
				ndMatrix covariance(comDist, comDist);
				ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				
				inertia.m_front += (bodyInertia.m_front + covariance.m_front.Scale (mass));
				inertia.m_up += (bodyInertia.m_up + covariance.m_up.Scale(mass));
				inertia.m_right += (bodyInertia.m_right + covariance.m_right.Scale(mass));

				gyroTorque += omega.CrossProduct(bodyInertia.RotateVector(omega));
			}

			m_gyroTorque = gyroTorque;
			inertia.m_posit = ndVector::m_wOne;
			m_invInertia = inertia.Inverse4x4();

			m_hasSupport = false;
			ndBodyKinematic::ndContactMap::Iterator it(m_ballBody->GetContactMap());
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = it.GetNode()->GetInfo();
				if (contact->IsActive())
				{
					m_hasSupport = true;
					world->CalculateJointContacts(contact);
					//bool newContact = true;
					//for (ndInt32 j = contacts.GetCount() - 1; j >= 0; --j)
					//{
					//	newContact = newContact && (contacts[j] != contact);
					//}
					//if (newContact)
					//{
					//	contacts.PushBack(contact);
					//}
				}
			}

			//for (ndInt32 i = contacts.GetCount() - 1; i >= 0; --i)
			//{
			//	world->CalculateJointContacts(contacts[i]);
			//}
		}

		//e) T0 = sum(w(i) x (I(i) * w(i))
		//f) T1 = sum[(p(i) - cg) x Fext(i) + Text(i)]
		//g) Bcg = (Icg ^ -1) * (T0 + T1)
		ndVector CalculateAlpha()
		{
			//ndVector torque(ndVector::m_zero);
			ndVector torque(m_gyroTorque);

			m_invDynamicsSolver.Solve();
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				ndBodyDynamic* const body = m_bodies[i];
				
				ndVector r(m_comDist[i]);
				ndVector f(m_invDynamicsSolver.GetBodyForce(body));
				ndVector t(m_invDynamicsSolver.GetBodyTorque(body));
				torque += (f + r.CrossProduct(f));
			}
			return m_invInertia.RotateVector(torque);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndSkeletonContainer* const skeleton = m_bodies[0]->GetSkeleton();
			ndAssert(skeleton);
			m_invDynamicsSolver.SolverBegin(skeleton, nullptr, 0, world, timestep);

			ndVector alpha(CalculateAlpha());
			if (ndAbs(alpha.m_z) > ND_ALPHA_TOL)
			{
				ndTrace(("%d alpha(%f) angle(%f)\n", xxx, alpha.m_z, m_controlJoint->GetOffsetAngle() * ndRadToDegree));
			}
			m_invDynamicsSolver.SolverEnd();
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			InitState(world);
			if (m_hasSupport)
			{
				ndSkeletonContainer* const skeleton = m_bodies[0]->GetSkeleton();
				ndAssert(skeleton);

				m_invDynamicsSolver.SolverBegin(skeleton, nullptr, 0, world, timestep);
				ndVector alpha(CalculateAlpha());
				if (ndAbs(alpha.m_z) > ND_ALPHA_TOL)
				{
					ndFloat32 angle = m_controlJoint->GetOffsetAngle();
					ndTrace(("%d alpha(%f) angle(%f)  deltaAngle(%f)\n", xxx, alpha.m_z, angle * ndRadToDegree, 0.0f));

					ndInt32 passes = 128;
					ndFloat32 angleLimit = ndFloat32(45.0f * ndDegreeToRad);
					do
					{
						passes--;
						ndFloat32 deltaAngle = alpha.m_z * 0.001f;
						angle += deltaAngle;
						
						angle = ndClamp(angle + deltaAngle, -angleLimit, angleLimit);
						m_controlJoint->SetOffsetAngle(angle);
						m_invDynamicsSolver.UpdateJointAcceleration(m_controlJoint);
						alpha = CalculateAlpha();
						ndTrace(("%d alpha(%f) angle(%f)  deltaAngle(%f)\n", xxx, alpha.m_z, angle * ndRadToDegree, deltaAngle));
					} while ((ndAbs(alpha.m_z) > ND_ALPHA_TOL) && passes);
					ndTrace(("\n"));
				}

				m_crossValidation____ = CalculateAlpha();
				m_invDynamicsSolver.SolverEnd();
			}

			xxx++;
		}

		ndMatrix m_invInertia;
		ndVector m_com;
		ndVector m_comVel;
		ndVector m_gyroTorque;

		ndFixSizeArray<ndVector, 8> m_comDist;
		ndFixSizeArray<ndBodyDynamic*, 8> m_bodies;
		ndBodyDynamic* m_ballBody;
		ndJointHinge* m_controlJoint;
		ndFloat32 m_totalMass;
		bool m_hasSupport;

		ndVector m_crossValidation____;
		int xxx;
	};

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndModelUnicycle* const model = new ndModelUnicycle();

		ndFloat32 mass = 10.0f;
		ndFloat32 limbMass = 1.0f;
		ndFloat32 wheelMass = 1.0f;

		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.40f;
		ndFloat32 zSize = 0.30f;
		ndPhysicsWorld* const world = scene->GetWorld();
		
		// add hip body
		ndSharedPtr<ndBody> hipBody(world->GetBody(AddBox(scene, location, mass, xSize, ySize, zSize, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(hipBody);

		ndMatrix matrix(hipBody->GetMatrix());
		matrix.m_posit.m_y += 0.5f;
		hipBody->SetMatrix(matrix);

		ndMatrix limbLocation(matrix);
		limbLocation.m_posit.m_z += zSize * 0.0f;
		limbLocation.m_posit.m_y -= ySize * 0.5f;
		limbLocation.m_posit.m_x += xSize * 0.5f * 0.0f;

		// make single leg
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadio = 0.025f;

		ndSharedPtr<ndBody> legBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga")));
		ndMatrix legLocation(ndRollMatrix(-90.0f * ndDegreeToRad) * limbLocation);
		legLocation.m_posit.m_y -= limbLength * 0.5f;
		legBody->SetMatrix(legLocation);
		ndMatrix legPivot(ndYawMatrix(90.0f * ndDegreeToRad) * legLocation);
		legPivot.m_posit.m_y += limbLength * 0.5f;
		ndSharedPtr<ndJointBilateralConstraint> legJoint(new ndJointHinge(legPivot, legBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));
		ndJointHinge* const hinge = (ndJointHinge*)*legJoint;
		hinge->SetAsSpringDamper(0.001f, 1500, 40.0f);
		model->m_controlJoint = hinge;

		// make wheel
		ndFloat32 wheelRadio = 4.0f * limbRadio;
		ndSharedPtr<ndBody> wheelBody(world->GetBody(AddSphere(scene, ndGetIdentityMatrix(), wheelMass, wheelRadio, "smilli.tga")));
		ndMatrix wheelMatrix(legPivot);
		wheelMatrix.m_posit.m_y -= limbLength;
		wheelBody->SetMatrix(wheelMatrix);
		ndSharedPtr<ndJointBilateralConstraint> wheelJoint(new ndJointSpherical(wheelMatrix, wheelBody->GetAsBodyKinematic(), legBody->GetAsBodyKinematic()));
		//((ndJointSpherical*)*wheelJoint)->SetAsSpringDamper(ndFloat32(0.001f), ndFloat32(0.0f), ndFloat32(10.f));

		// teleport the model so that is on the floor
		ndMatrix probeMatrix(wheelMatrix);
		probeMatrix.m_posit.m_x += 1.0f;
		ndMatrix floor(FindFloor(*world, probeMatrix, wheelBody->GetAsBodyKinematic()->GetCollisionShape(), 20.0f));
		ndFloat32 dist = wheelMatrix.m_posit.m_y - floor.m_posit.m_y;

		ndMatrix rootMatrix(modelRoot->m_body->GetMatrix());

		rootMatrix.m_posit.m_y -= dist;
		wheelMatrix.m_posit.m_y -= dist;
		legLocation.m_posit.m_y -= dist;

		legBody->SetMatrix(legLocation);
		wheelBody->SetMatrix(wheelMatrix);
		modelRoot->m_body->SetMatrix(rootMatrix);

		legBody->GetNotifyCallback()->OnTransform(0, legLocation);
		wheelBody->GetNotifyCallback()->OnTransform(0, wheelMatrix);
		modelRoot->m_body->GetNotifyCallback()->OnTransform(0, rootMatrix);

		// add the joints manually, because on this model the wheel is not actuated.
		world->AddJoint(legJoint);
		world->AddJoint(wheelJoint);

		// add model limbs
		model->AddLimb(modelRoot, legBody, legJoint);

		model->m_ballBody = wheelBody->GetAsBodyDynamic();

		model->Init();
		return model;
	}
}

using namespace ndController_0;
void ndBalanceController(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

	ndSharedPtr<ndModel> model(BuildModel(scene, matrix));
	scene->GetWorld()->AddModel(model);

	ndModelArticulation* const articulation = (ndModelArticulation*)model->GetAsModelArticulation();
	ndBodyKinematic* const rootBody = articulation->GetRoot()->m_body->GetAsBodyKinematic();
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
	world->AddJoint(fixJoint);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
