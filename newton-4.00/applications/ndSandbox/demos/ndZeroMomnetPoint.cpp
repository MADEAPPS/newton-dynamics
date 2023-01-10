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

		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_basePosition(ndVector::m_wOne)
				,m_effector(nullptr)
				,m_lookAtJoint(nullptr)
				,m_swivel(0.0f)
				,m_x(0.0f)
				,m_y(0.0f)
				,m_z(0.0f)
			{
			}

			ndEffectorInfo(ndIkSwivelPositionEffector* const effector, ndJointHinge* const lookActJoint)
				:m_basePosition(effector->GetLocalTargetPosition())
				,m_effector(effector)
				,m_lookAtJoint(lookActJoint)
				,m_swivel(0.0f)
				,m_x(0.0f)
				,m_y(0.0f)
				,m_z(0.0f)
			{
			}

			ndVector m_basePosition;
			ndIkSwivelPositionEffector* m_effector;
			ndJointHinge* m_lookAtJoint;
			ndReal m_swivel;
			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
			ndParamMapper m_x_mapper;
			ndParamMapper m_y_mapper;
			ndParamMapper m_z_mapper;
			ndParamMapper m_swivel_mapper;
		};

		ndZeroMomentModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
			:ndModel()
			,m_invDynamicsSolver()
			,m_bodies()
			//,m_rootBody(nullptr)
			//,m_effectorsInfo()
			//,m_effectorsJoints()
		{
			ndFloat32 mass = 10.0f;
			ndFloat32 xSize = 0.25f;
			ndFloat32 ySize = 0.50f;
			ndFloat32 zSize = 0.40f;
			ndFloat32 limbMass = 0.5f;
			ndFloat32 limbLength = 0.4f;
			ndFloat32 limbRadio = 0.05f;
			
			ndMatrix matrix(matrixLocation);
			ndPhysicsWorld* const world = scene->GetWorld();
			const ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			matrix.m_posit.m_y = floor.m_y + 1.5f;

			ndBodyKinematic* const torso = AddBox(scene, ndGetIdentityMatrix(), mass, xSize, ySize, zSize, "smilli.tga");
			torso->SetMatrix(matrix);
			m_bodies.PushBack(torso->GetAsBodyDynamic());

			ndBodyKinematic* const leg = AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga");
			ndMatrix legLocation(ndRollMatrix(90.0f * ndDegreeToRad) * matrix);
			//legLocation.m_posit.m_z += zSize * 0.5f;
			legLocation.m_posit.m_z += zSize * 0.0f;
			legLocation.m_posit.m_y -= ySize * 0.5f;
			legLocation.m_posit.m_y -= limbLength * 0.5f;
			leg->SetMatrix(legLocation);
			m_bodies.PushBack(leg->GetAsBodyDynamic());

			ndMatrix legPivot(legLocation);
			legPivot.m_posit.m_y += limbLength * 0.5f;
			ndIkJointSpherical* const legJoint = new ndIkJointSpherical(legPivot, leg, torso);
			ndSharedPtr<ndJointBilateralConstraint> ballPtr(legJoint);
			world->AddJoint(ballPtr);

			ndBodyKinematic* const calf = AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga");
			ndMatrix calfLocation(legLocation);
			calfLocation.m_posit.m_y -= limbLength;
			calf->SetMatrix(calfLocation);
			m_bodies.PushBack(calf->GetAsBodyDynamic());

			ndMatrix calfPivot(ndYawMatrix (90.0f * ndDegreeToRad) * calfLocation);
			calfPivot.m_posit.m_y += limbLength * 0.5f;
			ndIkJointHinge* const calfJoint = new ndIkJointHinge(calfPivot, calf, leg);
			calfJoint->SetLimitState(true);
			calfJoint->SetLimits(0.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
			ndSharedPtr<ndJointBilateralConstraint> calfPtr(calfJoint);
			world->AddJoint(calfPtr);
		}

		ndZeroMomentModel(const ndLoadSaveBase::ndLoadDescriptor& desc)
			:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
			, m_invDynamicsSolver()
			, m_bodies()
			//,m_effectorsInfo()
			//,m_effectorsJoints()
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

		//void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
		void Save(const ndLoadSaveBase::ndSaveDescriptor&) const
		{
			ndAssert(0);
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			//ndVector upVector(m_rootBody->GetMatrix().m_up);
			////for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			//for (ndInt32 i = 0; i < 4; ++i)
			//{
			//	const ndEffectorInfo& info = m_effectorsInfo[i];
			//	ndJointBilateralConstraint* const effector = info.m_effector;
			//	effector->DebugJoint(context);
			//
			//	//ndMatrix lookAtMatrix0;
			//	//ndMatrix lookAtMatrix1;
			//	//ndJointBilateralConstraint* const lookAtJoint = info.m_lookAtJoint;
			//	//lookAtJoint->DebugJoint(context);
			//	//info.m_lookAtJoint->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
			//	//ndVector updir(lookAtMatrix1.m_posit + upVector.Scale(-1.0f));
			//	//context.DrawLine(lookAtMatrix1.m_posit, updir, ndVector(0.0f, 0.0f, 0.0f, 1.0f));
			//
			//	//ndMatrix swivelMatrix0;
			//	//ndMatrix swivelMatrix1;
			//	//info.m_effector->CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);
			//	//
			//	//ndVector posit1(swivelMatrix1.m_posit);
			//	//posit1.m_y += 1.0f;
			//	//context.DrawLine(swivelMatrix1.m_posit, posit1, ndVector(0.0f, 0.0f, 0.0f, 1.0f));
			//}
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostUpdate(world, timestep);
		}

		void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostTransformUpdate(world, timestep);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::Update(world, timestep);

			//ndVector upVector(m_rootBody->GetMatrix().m_up);
			//for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); ++i)
			//{
			//	ndEffectorInfo& info = m_effectorsInfo[i];
			//	ndVector posit(info.m_basePosition);
			//	posit.m_x += info.m_x_mapper.Interpolate(info.m_x);
			//	posit.m_y += info.m_y_mapper.Interpolate(info.m_y);
			//	posit.m_z += info.m_z_mapper.Interpolate(info.m_z);
			//	info.m_effector->SetLocalTargetPosition(posit);
			//	info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel));
			//
			//	// calculate lookAt angle
			//	ndMatrix lookAtMatrix0;
			//	ndMatrix lookAtMatrix1;
			//	info.m_lookAtJoint->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
			//	const ndFloat32 lookAngle = info.m_lookAtJoint->CalculateAngle(upVector.Scale(-1.0f), lookAtMatrix0[1], lookAtMatrix0[0]);
			//	info.m_lookAtJoint->SetOffsetAngle(lookAngle);
			//}
			//
			//ndSkeletonContainer* const skeleton = m_rootBody->GetSkeleton();
			//ndAssert(skeleton);
			//
			////m_invDynamicsSolver.SetMaxIterations(4);
			//if (m_effectorsJoints.GetCount() && !m_invDynamicsSolver.IsSleeping(skeleton))
			//{
			//	ndFixSizeArray<ndJointBilateralConstraint*, 8> effectors;
			//	for (ndInt32 i = 0; i < m_effectorsJoints.GetCount(); ++i)
			//	{
			//		effectors.PushBack(*m_effectorsJoints[i]);
			//	}
			//
			//	m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), world, timestep);
			//	m_invDynamicsSolver.Solve();
			//	m_invDynamicsSolver.SolverEnd();
			//}
		}

		ndIkSolver m_invDynamicsSolver;
		ndFixSizeArray<ndBodyDynamic*, 8> m_bodies;
		//ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		//ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;
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

			//ndZeroMomentModel::ndEffectorInfo& info = m_model->m_effectorsInfo[0];
			//bool change = false;
			//ImGui::Text("position x");
			//change = change | ImGui::SliderFloat("##x", &info.m_x, -1.0f, 1.0f);
			//ImGui::Text("position y");
			//change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
			//ImGui::Text("position z");
			//change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);
			//
			//ImGui::Text("swivel");
			//change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);
			//
			//if (change)
			//{
			//	m_model->m_rootBody->SetSleepState(false);
			//
			//	for (ndInt32 i = 1; i < m_model->m_effectorsInfo.GetCount(); ++i)
			//	{
			//		m_model->m_effectorsInfo[i].m_x = info.m_x;
			//		m_model->m_effectorsInfo[i].m_y = info.m_y;
			//		m_model->m_effectorsInfo[i].m_z = info.m_z;
			//		m_model->m_effectorsInfo[i].m_swivel = info.m_swivel;
			//	}
			//}
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
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
	ndZeroMomentModel* const robot0 = new ndZeroMomentModel(scene, matrix);
	scene->SetSelectedModel(robot0);
	ndSharedPtr<ndModel> modelPtr(robot0);
	world->AddModel(modelPtr);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot0->GetRoot()->GetMatrix(), robot0->GetRoot(), world->GetSentinelBody()));
	//world->AddJoint(fixJoint);

	ndModelUI* const quadrupedUI = new ndModelUI(scene, robot0);
	ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	scene->Set2DDisplayRenderFunction(quadrupedUIPtr);
	
	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
