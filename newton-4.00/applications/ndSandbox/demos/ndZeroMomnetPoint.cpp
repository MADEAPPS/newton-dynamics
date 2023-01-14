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
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndAnimationPose.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequenceBase.h"
#include "ndAnimationSequencePlayer.h"


namespace ndZmp
{
	class ndZeroMomentModel : public ndModel
	{
		public:
		D_CLASS_REFLECTION(ndZmp::ndZeroMomentModel);

		class ndEffector
		{
			public:
			ndEffector()
			{
			}

			ndEffector(ndIkSwivelPositionEffector* const effector)
				:m_yaw(0.0f)
				,m_roll(0.0f)
				,m_height(0.9f)
				,m_swivel(0.0f)
				,m_joint(effector)
			{
				SetPosition();
			}

			void SetPosition()
			{
				ndFloat32 minRadio;
				ndFloat32 maxRadio;

				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*m_joint;
				effector->GetWorkSpaceConstraints(minRadio, maxRadio);
				//ndVector posit(effector->GetLocalTargetPosition() & ndVector::m_triplexMask);

				const ndMatrix yaw(ndYawMatrix(m_yaw * ndDegreeToRad));
				const ndMatrix roll(ndRollMatrix(m_roll * ndDegreeToRad));
				ndVector posit(m_height * maxRadio, 0.0f, 0.0f, 1.0f);
				posit = yaw.RotateVector(posit);
				posit = roll.RotateVector(posit);
				//posit = posit.Normalize().Scale(maxRadio * m_height);
				effector->SetLocalTargetPosition(posit);
			}

			ndFloat32 m_yaw;
			ndFloat32 m_roll;
			ndFloat32 m_height;
			ndFloat32 m_swivel;
			ndSharedPtr<ndIkSwivelPositionEffector> m_joint;
		};

		ndZeroMomentModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
			:ndModel()
			,m_effector()
			,m_invDynamicsSolver()
			,m_bodies()
			,m_wheelBody(nullptr)
		{
			ndFloat32 mass = 10.0f;
			ndFloat32 xSize = 0.25f;
			ndFloat32 ySize = 0.50f;
			ndFloat32 zSize = 0.40f;
			
			ndMatrix matrix(matrixLocation);
			ndPhysicsWorld* const world = scene->GetWorld();
			const ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			matrix.m_posit.m_y = floor.m_y + 1.0f;

			// add hip body
			ndBodyKinematic* const hipBody = AddBox(scene, ndGetIdentityMatrix(), mass, xSize, ySize, zSize, "smilli.tga");
			hipBody->SetMatrix(matrix);
			m_bodies.PushBack(hipBody->GetAsBodyDynamic());

			ndMatrix limbLocation(matrix);
			limbLocation.m_posit.m_z += zSize * 0.0f;
			limbLocation.m_posit.m_y -= ySize * 0.5f;

			AddLimb(scene, limbLocation);
		}

		ndZeroMomentModel(const ndLoadSaveBase::ndLoadDescriptor& desc)
			:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
			,m_effector()
			,m_invDynamicsSolver()
			,m_bodies()
		{
			ndAssert(0);
		}

		~ndZeroMomentModel()
		{
		}

		ndBodyDynamic* GetRoot() const
		{
			return m_bodies[0];
		}

		void AddLimb(ndDemoEntityManager* const scene, const ndMatrix& matrix)
		{
			ndFloat32 limbMass = 0.5f;
			ndFloat32 limbLength = 0.3f;
			ndFloat32 limbRadio = 0.025f;

			ndBodyKinematic* const hipBody = GetRoot();
			ndPhysicsWorld* const world = scene->GetWorld();
			
			// Add upper leg limb (thigh)
			ndBodyKinematic* const legBody = AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga");
			ndMatrix legLocation(ndRollMatrix(-90.0f * ndDegreeToRad) * matrix);
			legLocation.m_posit.m_y -= limbLength * 0.5f;
			legBody->SetMatrix(legLocation);
			m_bodies.PushBack(legBody->GetAsBodyDynamic());

			ndMatrix legPivot(legLocation);
			legPivot.m_posit.m_y += limbLength * 0.5f;
			ndIkJointSpherical* const legJoint = new ndIkJointSpherical(legPivot, legBody, hipBody);
			ndSharedPtr<ndJointBilateralConstraint> ballPtr(legJoint);
			world->AddJoint(ballPtr);

			// Add lower leg limb (calf)
			ndBodyKinematic* const calfBody = AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga");
			ndMatrix calfLocation(legLocation);
			calfLocation.m_posit.m_y -= limbLength;
			calfBody->SetMatrix(calfLocation);
			m_bodies.PushBack(calfBody->GetAsBodyDynamic());

			ndMatrix calfPivot(ndYawMatrix(90.0f * ndDegreeToRad) * calfLocation);
			calfPivot.m_posit.m_y += limbLength * 0.5f;
			ndIkJointHinge* const calfJoint = new ndIkJointHinge(calfPivot, calfBody, legBody);
			calfJoint->SetLimitState(true);
			calfJoint->SetLimits(0.0f * ndDegreeToRad, 150.0f * ndDegreeToRad);
			ndSharedPtr<ndJointBilateralConstraint> calfPtr(calfJoint);
			world->AddJoint(calfPtr);

			// Add end effector
			ndFloat32 regularizer = 0.001f;
			ndVector effectorPivot(calfLocation.m_posit);
			effectorPivot.m_y -= limbLength * 0.5f;
			ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorPivot, legPivot, calfBody, hipBody);
			effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
			effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);

			ndFloat32 workSpace = 0.99f * 2.0f * limbLength;
			effector->SetWorkSpaceConstraints(0.0f, workSpace);

			// Add wheel leg limb (calf)
			ndFloat32 wheelRadios = 4.0f * limbRadio;
			ndBodyKinematic* const wheelBody = AddSphere(scene, ndGetIdentityMatrix(), 2.0f * limbMass, wheelRadios, "smilli.tga");
			ndMatrix wheelMatrix(effector->GetLocalMatrix0() * calfLocation);
			wheelBody->SetMatrix(wheelMatrix);
			m_bodies.PushBack(wheelBody->GetAsBodyDynamic());

			ndJointSpherical* const wheelJoint = new ndJointSpherical(wheelMatrix, wheelBody, calfBody);
			ndSharedPtr<ndJointBilateralConstraint> wheelJointPtr(wheelJoint);
			world->AddJoint(wheelJointPtr);

			m_effector = ndEffector(effector);
			m_wheelBody = wheelBody;
		}

		//void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
		void Save(const ndLoadSaveBase::ndSaveDescriptor&) const
		{
			ndAssert(0);
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndJointBilateralConstraint* const joint = (ndJointBilateralConstraint*)*m_effector.m_joint;
			joint->DebugJoint(context);
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostUpdate(world, timestep);
		}

		void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostTransformUpdate(world, timestep);
		}

		ndVector CalculateZeroMomentPoint(const ndVector& reference) const
		{
			ndVector forceAcc(ndVector::m_zero);
			ndVector torqueAcc(ndVector::m_zero);
			const ndVector gravity(ndFloat32(0.0f), -DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f));

			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				ndBodyKinematic* const body = m_bodies[i];
				ndVector com(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
				ndVector action(com - reference);
				ndVector force(gravity.Scale(body->GetMassMatrix().m_w));
				ndVector torque(m_invDynamicsSolver.GetBodyTorque(body));

				ndVector actionTorque(action.CrossProduct(force));
				forceAcc += force;
				torqueAcc += (actionTorque - torque);
			}
			ndVector zmp(torqueAcc.m_z / forceAcc.m_y, ndFloat32(0.0f), -torqueAcc.m_x / forceAcc.m_y, ndFloat32(1.0f));
			zmp = zmp.Scale(0.0f);
			zmp += reference;
			return zmp;
		}

		ndVector CalculateNetTorque() const
		{
			ndVector torqueAcc(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				ndBodyKinematic* const body = m_bodies[i];
				torqueAcc += m_invDynamicsSolver.GetBodyTorque(body);
			}
			return torqueAcc;
		}

		bool TestBalance(const ndVector& zeroMomentPoint,  const ndVector& reference) const
		{
			ndVector zmp(zeroMomentPoint - reference);
			ndVector torqueAcc (ndVector::m_zero);
			const ndVector gravity(ndFloat32(0.0f), -DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f));
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				ndBodyKinematic* const body = m_bodies[i];
				ndVector com(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
				ndVector action(zmp - (com - reference));
				ndVector torque(m_invDynamicsSolver.GetBodyTorque(body));
				ndVector force(gravity.Scale(body->GetMassMatrix().m_w));
				torqueAcc += (torque + action.CrossProduct(force));
			}
			bool ret = (ndAbs(torqueAcc.m_x) < 0.01f) && (ndAbs(torqueAcc.m_z) < 0.01f);
			//return ret;
			return true;
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::Update(world, timestep);
			
			ndSkeletonContainer* const skeleton = GetRoot()->GetSkeleton();
			ndAssert(skeleton);
		
			if (!m_invDynamicsSolver.IsSleeping(skeleton))
			{
				ndBodyKinematic::ndContactMap& contacts = m_wheelBody->GetContactMap();
				bool hasContact = false;
				ndBodyKinematic::ndContactMap::Iterator iter(contacts);
				for (iter.Begin(); iter; iter++)
				{
					ndContact* const contact = iter.GetNode()->GetInfo();
					hasContact = hasContact || contact->IsActive();
				}

				ndFixSizeArray<ndJointBilateralConstraint*, 2> effectors;
				ndIkSwivelPositionEffector* const joint = *m_effector.m_joint;
				effectors.PushBack(joint);

				m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), world, timestep);
				if (hasContact)
				{
					ndVector torque0(CalculateNetTorque());
					m_effector.m_roll = 25.5f;
					m_effector.SetPosition();
					m_invDynamicsSolver.Solve();
					ndVector torque1(CalculateNetTorque());


					m_effector.m_roll = -25.5f;
					m_effector.SetPosition();
					m_invDynamicsSolver.Solve();
					ndVector torque2(CalculateNetTorque());
					ndVector torque3(CalculateNetTorque());
					
					//m_invDynamicsSolver.Solve();
					//const ndVector refPoint(m_wheelBody->GetMatrix().m_posit);
					//ndVector zmp(CalculateZeroMomentPoint(refPoint));
					//ndAssert(TestBalance(zmp, refPoint));
					//
					//const ndMatrix refFrame(joint->GetReferenceFrame());
					//ndVector localZmp = refFrame.UntransformVector(zmp);
					//joint->SetLocalTargetPosition(localZmp);
					//
					//{
					//	m_invDynamicsSolver.Solve();
					//	ndVector zmp1(CalculateZeroMomentPoint(refPoint));
					//	ndVector force(m_invDynamicsSolver.GetBodyForce(m_bodies[m_bodies.GetCount() - 1]));
					//	ndVector torque(m_invDynamicsSolver.GetBodyTorque(m_bodies[m_bodies.GetCount() - 1]));
					//	ndTrace(("%d: F(%f %f %f) T(%f %f %f)\n", xxx, force.m_x, force.m_y, force.m_z, torque.m_x, torque.m_y, torque.m_z));
					//	ndAssert(TestBalance(zmp1, refPoint));
					//}
				}

				m_invDynamicsSolver.Solve();
				ndVector torque2(CalculateNetTorque());
			}
			m_invDynamicsSolver.SolverEnd();
		}

		ndEffector m_effector;
		ndIkSolver m_invDynamicsSolver;
		ndFixSizeArray<ndBodyDynamic*, 8> m_bodies;
		ndBodyKinematic* m_wheelBody;
	};
	D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndZeroMomentModel);

	class ndModelUI : public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, ndZeroMomentModel* const quadruped)
			:ndUIEntity(scene)
			,m_model(quadruped)
		{
		}

		~ndModelUI()
		{
		}

		virtual void RenderUI()
		{
		}

		virtual void RenderHelp()
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			m_scene->Print(color, "Control panel");

			ndZeroMomentModel::ndEffector& info = m_model->m_effector;
			bool change = false;
			ImGui::Text("height");
			change = change || ImGui::SliderFloat("##x", &info.m_height, 0.3f, 1.0f);
			ImGui::Text("roll");
			change = change | ImGui::SliderFloat("##z", &info.m_roll, -30.0f, 30.0f);
			ImGui::Text("yaw");
			change = change | ImGui::SliderFloat("##y", &info.m_yaw, -30.0f, 30.0f);

			//ImGui::Text("swivel");
			//change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);

			if (change)
			{
				m_model->GetRoot()->SetSleepState(false);
				info.SetPosition();
			}
		}

		ndZeroMomentModel* m_model;
	};
};

using namespace ndZmp;
void ndZeroMomentPoint(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	
	//ndVector origin1(1.0f, 0.0f, 0.0f, 1.0f);
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	matrix.m_posit.m_x = 1.0f;
	
	ndZeroMomentModel* const robot = new ndZeroMomentModel(scene, matrix);
	scene->SetSelectedModel(robot);
	ndSharedPtr<ndModel> modelPtr(robot);
	world->AddModel(modelPtr);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot->GetRoot()->GetMatrix(), robot->GetRoot(), world->GetSentinelBody()));
	//ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointPlane (robot->GetRoot()->GetMatrix().m_posit, ndVector (0.0f, 0.0f, 1.0f, 0.0f), robot->GetRoot(), world->GetSentinelBody()));
	//world->AddJoint(fixJoint);

	ndModelUI* const quadrupedUI = new ndModelUI(scene, robot);
	ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	scene->Set2DDisplayRenderFunction(quadrupedUIPtr);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 4.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
