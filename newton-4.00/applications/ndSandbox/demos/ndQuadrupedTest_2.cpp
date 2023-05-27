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

namespace ndQuadruped_2
{
	class ndQuadrupedModel : public ndModel
	{
		public:
		D_CLASS_REFLECTION(ndQuadruped_2::ndQuadrupedModel, ndModel)

		class ndQuadrupedModelSaveLoad : public ndFileFormatModel
		{
			public:
			ndQuadrupedModelSaveLoad()
				:ndFileFormatModel(ndQuadrupedModel::StaticClassName())
			{
			}

			void SaveModel(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndModel* const model)
			{
				nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndQuadrupedModel2", ndQuadrupedModel::StaticClassName());
				ndFileFormatModel::SaveModel(scene, classNode, model);

				ndQuadrupedModel* const quadruped = (ndQuadrupedModel*)model;
				xmlSaveParam(classNode, "rootBody", scene->FindBodyId(quadruped->m_rootBody));

				nd::TiXmlElement* const limbsArrayNode = new nd::TiXmlElement("limbsArray");
				classNode->LinkEndChild(limbsArrayNode);
				limbsArrayNode->SetAttribute("count", quadruped->m_effectorsInfo.GetCount());

				for (ndInt32 i = 0; i < quadruped->m_effectorsInfo.GetCount(); ++i)
				{
					nd::TiXmlElement* const limbNode = new nd::TiXmlElement("limb");
					limbsArrayNode->LinkEndChild(limbNode);
					const ndEffectorInfo& info = quadruped->m_effectorsInfo[i];
					ndJointBilateralConstraint* const effector = info.m_effector;

					//ndVector m_basePosition;
					//ndIkSwivelPositionEffector* m_effector;
					//ndJointHinge* m_lookAtJoint;
					//ndReal m_swivel;
					//ndReal m_x;
					//ndReal m_y;
					//ndReal m_z;
					//ndParamMapper m_x_mapper;
					//ndParamMapper m_y_mapper;
					//ndParamMapper m_z_mapper;
					//ndParamMapper m_swivel_mapper;

					ndAssert(info.m_lookAtJoint);
					xmlSaveParam(limbNode, "lookAtJoint", scene->FindJointId(info.m_lookAtJoint));
					xmlSaveParam(limbNode, "effectorBasePosition", info.m_basePosition);

					xmlSaveParam(limbNode, "x_mapper_x0", info.m_x_mapper.Interpolate(-1.0f));
					xmlSaveParam(limbNode, "x_mapper_x1", info.m_x_mapper.Interpolate( 1.0f));
																		  
					xmlSaveParam(limbNode, "y_mapper_x0", info.m_y_mapper.Interpolate(-1.0f));
					xmlSaveParam(limbNode, "y_mapper_x1", info.m_y_mapper.Interpolate( 1.0f));
																		  
					xmlSaveParam(limbNode, "z_mapper_x0", info.m_z_mapper.Interpolate(-1.0f));
					xmlSaveParam(limbNode, "z_mapper_x1", info.m_z_mapper.Interpolate( 1.0f));

					xmlSaveParam(limbNode, "swivel_mapper_x0", info.m_swivel_mapper.Interpolate(-1.0f));
					xmlSaveParam(limbNode, "swivel_mapper_x1", info.m_swivel_mapper.Interpolate( 1.0f));

					ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(effector->ClassName());
					ndAssert(handler);
					handler->SaveJoint(scene, limbNode, effector);
				}
			}
		};

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

		ndQuadrupedModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
			:ndModel()
			,m_invDynamicsSolver()
			,m_rootBody(nullptr)
			,m_effectorsInfo()
			,m_effectorsJoints()
		{
			static ndQuadrupedModelSaveLoad loadSaveModel;
			ndFloat32 mass = 10.0f;
			ndFloat32 radius = 0.25f;
			ndFloat32 limbMass = 0.5f;
			ndFloat32 limbLength = 0.3f;
			ndFloat32 limbRadios = 0.06f;
			
			ndPhysicsWorld* const world = scene->GetWorld();
			ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			ndBodyKinematic* const torso = AddSphere(scene, matrixLocation, mass, radius, "smilli.tga");
			m_rootBody = torso->GetAsBodyDynamic();
			
			ndMatrix location(matrixLocation);
			location.m_posit.m_y = floor.m_y + 1.0f;
			m_rootBody->SetMatrix(location);
			
			ndDemoEntity* const entity = (ndDemoEntity*)torso->GetNotifyCallback()->GetUserData();
			entity->SetMeshMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * ndPitchMatrix(90.0f * ndDegreeToRad));
			
			ndMatrix matrix(ndRollMatrix(45.0f * ndDegreeToRad));
			matrix.m_posit.m_x = radius * 0.9f;
			matrix.m_posit.m_y = -radius * 0.5f;
			
			ndFloat32 angles[] = { 300.0f, 240.0f, 120.0f, 60.0f };
			//ndFloat32 angles[] = { 270.0f, 90.0f, 120.0f, 60.0f };
			
			const ndVector upDir(location.m_up);
			for (ndInt32 i = 0; i < 4; ++i)
			{
				ndMatrix limbPivotLocation(matrix * ndYawMatrix(angles[i] * ndDegreeToRad));
				limbPivotLocation.m_posit += torso->GetMatrix().m_posit;
				limbPivotLocation.m_posit.m_w = 1.0f;
			
				// add leg thigh
				const ndVector thighPivot(limbPivotLocation.m_posit);
			
				ndFloat32 workSpace = 0.0f;
				ndBodyKinematic* thigh = nullptr;
				{
					ndMatrix bodyMatrix(limbPivotLocation);
					bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
					thigh = AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength);
					thigh->SetMatrix(bodyMatrix);
					ndIkJointSpherical* const ball = new ndIkJointSpherical(limbPivotLocation, thigh, torso);
					ndSharedPtr<ndJointBilateralConstraint> ballPtr(ball);
					world->AddJoint(ballPtr);
			
					limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
			
					workSpace += limbLength;
				}
			
				// add calf0
				ndBodyKinematic* calf0 = nullptr;
				{
					limbPivotLocation = ndRollMatrix(-90.0f * ndDegreeToRad) * limbPivotLocation;
			
					ndMatrix bodyMatrix(limbPivotLocation);
					bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
					calf0 = AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength);
					calf0->SetMatrix(bodyMatrix);
			
					ndMatrix caffPinAndPivotFrame(ndGetIdentityMatrix());
					ndFloat32 sign = angles[i] > 180.0f ? -1.0f : 1.0f;
					caffPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(sign);
					caffPinAndPivotFrame.m_up = limbPivotLocation.m_front;
					caffPinAndPivotFrame.m_right = caffPinAndPivotFrame.m_front.CrossProduct(caffPinAndPivotFrame.m_up);
					caffPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
					ndIkJointHinge* const hinge = new ndIkJointHinge(caffPinAndPivotFrame, calf0, thigh);
			
					// add joint limit to prevent knee from flipping
					hinge->SetLimitState(true);
					hinge->SetLimits(-70.0f * ndDegreeToRad, 70.0f * ndDegreeToRad);
					ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
					world->AddJoint(hingePtr);
			
					limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
					workSpace += limbLength;
				}
			
				ndBodyKinematic* calf1 = nullptr;
				ndJointHinge* lookActHinge = nullptr;
				{
					ndFloat32 lenght = limbLength * 0.5f;
					limbPivotLocation = ndRollMatrix(-45.0f * ndDegreeToRad) * limbPivotLocation;
					ndMatrix bodyMatrix(limbPivotLocation);
					bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(lenght * 0.5f);
			
					calf1 = AddCapsule(scene, bodyMatrix, limbMass * 0.5f, limbRadios, limbRadios, lenght);
					calf1->SetMatrix(bodyMatrix);
			
					ndMatrix caffPinAndPivotFrame(ndGetIdentityMatrix());
					caffPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(-1.0f);
					caffPinAndPivotFrame.m_up = limbPivotLocation.m_front;
					caffPinAndPivotFrame.m_right = caffPinAndPivotFrame.m_front.CrossProduct(caffPinAndPivotFrame.m_up);
					caffPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
			
					// add joint limit to prevent knee from flipping
					lookActHinge = new ndJointHinge(caffPinAndPivotFrame, calf0, calf1);
					lookActHinge->SetLimitState(true);
					lookActHinge->SetLimits(-60.0f * ndDegreeToRad, 60.0f * ndDegreeToRad);
					lookActHinge->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);

					ndSharedPtr<ndJointBilateralConstraint> hingePtr(lookActHinge);
					world->AddJoint(hingePtr);
			
					limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(lenght);
					workSpace += lenght;
				}
			
				// add leg effector
				{
					ndBodyKinematic* const targetBody = calf1;
			
					ndFloat32 angle(i < 2 ? -90.0f : 90.0f);
					ndMatrix effectorToeFrame(ndGetIdentityMatrix());
					ndMatrix effectorRefFrame(ndYawMatrix(angle * ndDegreeToRad));
					effectorRefFrame.m_posit = thighPivot;
					effectorToeFrame.m_posit = limbPivotLocation.m_posit;
			
					ndFloat32 regularizer = 0.001f;
					ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorToeFrame.m_posit, effectorRefFrame, targetBody, torso);
					effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
					effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
					effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.9f);
					
					ndEffectorInfo info(effector, lookActHinge);
					info.m_x_mapper = ndParamMapper(-0.2f, 0.2f);
					info.m_y_mapper = ndParamMapper(-0.4f, 0.1f);
					info.m_z_mapper = ndParamMapper(-0.15f, 0.15f);
					info.m_swivel_mapper = ndParamMapper(-20.0f * ndDegreeToRad, 20.0f * ndDegreeToRad);
					m_effectorsInfo.PushBack(info);
					m_effectorsJoints.PushBack(effector);
				}
			}
		}

		~ndQuadrupedModel()
		{
		}

		virtual void OnAddToWorld() { ndAssert(0); }
		virtual void OnRemoveFromToWorld() { ndAssert(0); }

		void GetContacts(ndFixSizeArray<ndVector, 4>& contacts) const 
		{
			for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); ++i)
			{
				const ndEffectorInfo& info = m_effectorsInfo[i];
				ndJointBilateralConstraint* const effector = info.m_effector;
				ndBodyKinematic* const body = effector->GetBody0();
				ndBodyKinematic::ndContactMap& contactMap = body->GetContactMap();
				ndBodyKinematic::ndContactMap& contactMap1 = body->GetContactMap();
			}
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndVector upVector(m_rootBody->GetMatrix().m_up);
			//for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			for (ndInt32 i = 0; i < 4; ++i)
			{
				const ndEffectorInfo& info = m_effectorsInfo[i];
				ndJointBilateralConstraint* const effector = info.m_effector;
				effector->DebugJoint(context);

				//ndMatrix lookAtMatrix0;
				//ndMatrix lookAtMatrix1;
				//ndJointBilateralConstraint* const lookAtJoint = info.m_lookAtJoint;
				//lookAtJoint->DebugJoint(context);
				//info.m_lookAtJoint->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
				//ndVector updir(lookAtMatrix1.m_posit + upVector.Scale(-1.0f));
				//context.DrawLine(lookAtMatrix1.m_posit, updir, ndVector(0.0f, 0.0f, 0.0f, 1.0f));

				//ndMatrix swivelMatrix0;
				//ndMatrix swivelMatrix1;
				//info.m_effector->CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);
				//
				//ndVector posit1(swivelMatrix1.m_posit);
				//posit1.m_y += 1.0f;
				//context.DrawLine(swivelMatrix1.m_posit, posit1, ndVector(0.0f, 0.0f, 0.0f, 1.0f));
			}

			ndFixSizeArray<ndVector, 4> contacts;
			GetContacts(contacts);

			//ndFixSizeArray<ndVector, 16> contactPoints;
			//ndGaitController::ndSupportContacts support(m_state.GetSupportContacts());
			//for (ndInt32 i = 0; i < m_state.m_posit.GetCount(); ++i)
			//{
			//	const ndEffectorPosit& effectPosit = m_state.m_posit[i];
			//	ndJointBilateralConstraint* const joint = (ndJointBilateralConstraint*)effectPosit.m_effector;
			//	joint->DebugJoint(context);
			//	if (support[i])
			//	{
			//		ndBodyKinematic* const body = joint->GetBody0();
			//		contactPoints.PushBack(body->GetMatrix().TransformVector(joint->GetLocalMatrix0().m_posit));
			//	}
			//}
			//
			//ndMatrix comMatrix(m_localFrame * m_bodyArray[0]->GetMatrix());
			//comMatrix.m_posit = CalculateCenterOfMass().m_linear;
			//context.DrawFrame(comMatrix);
			//
			//if (contactPoints.GetCount() >= 3)
			//{
			//	ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
			//	rotation.TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
			//	ndInt32 supportCount = ndConvexHull2d(&contactPoints[0], contactPoints.GetCount());
			//	rotation.Inverse().TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
			//	ndVector p0(contactPoints[supportCount - 1]);
			//	ndBigVector bigPolygon[16];
			//	for (ndInt32 i = 0; i < supportCount; ++i)
			//	{
			//		bigPolygon[i] = contactPoints[i];
			//		context.DrawLine(contactPoints[i], p0, ndVector::m_zero);
			//		p0 = contactPoints[i];
			//	}
			//
			//	ndBigVector p0Out;
			//	ndBigVector p1Out;
			//	ndBigVector ray_p0(comMatrix.m_posit);
			//	ndBigVector ray_p1(comMatrix.m_posit);
			//	ray_p1.m_y -= 1.0f;
			//
			//	ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
			//
			//	context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
			//	context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			//}
			//else if (contactPoints.GetCount() == 2)
			//{
			//	ndAssert(0);
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

			ndVector upVector(m_rootBody->GetMatrix().m_up);
			for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); ++i)
			{
				ndEffectorInfo& info = m_effectorsInfo[i];
				ndVector posit(info.m_basePosition);
				posit.m_x += info.m_x_mapper.Interpolate(info.m_x);
				posit.m_y += info.m_y_mapper.Interpolate(info.m_y);
				posit.m_z += info.m_z_mapper.Interpolate(info.m_z);
				info.m_effector->SetLocalTargetPosition(posit);
				info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel));

				// calculate lookAt angle
				ndMatrix lookAtMatrix0;
				ndMatrix lookAtMatrix1;
				info.m_lookAtJoint->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
				const ndFloat32 lookAngle = info.m_lookAtJoint->CalculateAngle(upVector.Scale(-1.0f), lookAtMatrix0[1], lookAtMatrix0[0]);
				info.m_lookAtJoint->SetOffsetAngle(lookAngle);
			}

			ndSkeletonContainer* const skeleton = m_rootBody->GetSkeleton();
			ndAssert(skeleton);

			//m_invDynamicsSolver.SetMaxIterations(4);
			if (m_effectorsJoints.GetCount() && !m_invDynamicsSolver.IsSleeping(skeleton))
			{
				ndFixSizeArray<ndJointBilateralConstraint*, 8> effectors;
				for (ndInt32 i = 0; i < m_effectorsJoints.GetCount(); ++i)
				{
					effectors.PushBack(*m_effectorsJoints[i]);
				}

				m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), world, timestep);
				m_invDynamicsSolver.Solve();
				m_invDynamicsSolver.SolverEnd();
			}
		}

		ndIkSolver m_invDynamicsSolver;
		ndBodyDynamic* m_rootBody;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;
	};

	class ndModelUI : public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, ndQuadrupedModel* const quadruped)
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

			ndQuadrupedModel::ndEffectorInfo& info = m_model->m_effectorsInfo[0];

			bool change = false;
			ImGui::Text("position x");
			change = change | ImGui::SliderFloat("##x", &info.m_x, -1.0f, 1.0f);
			ImGui::Text("position y");
			change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
			ImGui::Text("position z");
			change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);

			ImGui::Text("swivel");
			change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);

			if (change)
			{
				m_model->m_rootBody->SetSleepState(false);

				for (ndInt32 i = 1; i < m_model->m_effectorsInfo.GetCount(); ++i)
				{
					m_model->m_effectorsInfo[i].m_x = info.m_x;
					m_model->m_effectorsInfo[i].m_y = info.m_y;
					m_model->m_effectorsInfo[i].m_z = info.m_z;
					m_model->m_effectorsInfo[i].m_swivel = info.m_swivel;
				}
			}
		}

		ndQuadrupedModel* m_model;
	};
}

using namespace ndQuadruped_2;
void ndQuadrupedTest_2(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildFlatPlane(scene, true);
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
	ndQuadrupedModel* const robot0 = new ndQuadrupedModel(scene, matrix);
	scene->SetSelectedModel(robot0);
	ndSharedPtr<ndModel> modelPtr(robot0);
	world->AddModel(modelPtr);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot0->m_rootBody->GetMatrix(), robot0->m_rootBody, world->GetSentinelBody()));
	//world->AddJoint(fixJoint);

	ndModelUI* const quadrupedUI = new ndModelUI(scene, robot0);
	ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	scene->Set2DDisplayRenderFunction(quadrupedUIPtr);
	
	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);

	//ndFileFormatSave xxxx;
	//xxxx.SaveWorld(scene->GetWorld(), "xxxx.nd");
}
