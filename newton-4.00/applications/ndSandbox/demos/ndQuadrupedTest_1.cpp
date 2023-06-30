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

namespace ndQuadruped_1
{
#if 0 
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
#else

	class ndModelQuadruped: public ndModelArticulation
	{
		public:
		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_footHinge(nullptr)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(const ndSharedPtr<ndJointBilateralConstraint>& effector, ndJointHinge* const footHinge)
				:m_footHinge(footHinge)
				,m_effector(effector)
			{
			}

			ndJointHinge* m_footHinge;
			ndSharedPtr<ndJointBilateralConstraint> m_effector;
		};

		class ndPoseGenerator : public ndAnimationSequence
		{
			public:
			ndPoseGenerator()
				:ndAnimationSequence()
			{
				m_duration = ndFloat32 (4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = ndFloat32 (0.0f);
					m_offset[i] = ndFloat32(0.0f);
				}
			}

			virtual ndVector GetTranslation(ndFloat32) const
			{
				return ndVector::m_zero;
			}

			virtual void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
			{
				// generate a procedural in place march gait
				const ndFloat32 gaitFraction = 0.25f;
				ndFloat32 amp = 0.27f;
				ndFloat32 omega = ndPi / gaitFraction;

				ndFloat32 high = -0.3f;
				ndVector base (ndVector::m_wOne);
				base.m_x = 0.4f;
				base.m_y = high;
				for (ndInt32 i = 0; i < 4; i++)
				{
					output[i].m_posit = base;
					output[i].m_posit.m_z = m_offset[i];
					ndFloat32 t = ndMod (param - m_phase[i] + ndFloat32(1.0f), ndFloat32 (1.0f));
					if (t <= gaitFraction)
					{
						output[i].m_posit.m_y += amp * ndSin(omega * t);
					}
					output[i].m_rotation.m_w = ((t > ndFloat32(0.0f)) && (t < gaitFraction)) ? ndFloat32(0.0f) : ndFloat32(1.0f);
				}
			}

			ndFloat32 m_phase[4];
			ndFloat32 m_offset[4];
		};

		class ndUIControlNode: public ndAnimationBlendTreeNode
		{
			public:
			ndUIControlNode(ndAnimationBlendTreeNode* const input)
				:ndAnimationBlendTreeNode(input)
				,m_x(ndReal(0.0f))
				,m_y(ndReal(0.0f))
				,m_z(ndReal(0.0f))
				,m_pitch(ndReal(0.0f))
				,m_yaw(ndReal(0.0f))
				,m_roll(ndReal(0.0f))
				,m_animSpeed(ndReal(0.0f))
			{
			}

			void Evaluate(ndAnimationPose& output, ndVector& veloc)
			{
				ndAnimationBlendTreeNode::Evaluate(output, veloc);

				ndMatrix matrix(ndPitchMatrix(m_pitch * ndDegreeToRad) * ndYawMatrix(m_yaw * ndDegreeToRad) * ndRollMatrix(m_roll * ndDegreeToRad));
				matrix.m_posit.m_x = -m_x;
				matrix.m_posit.m_y = -m_y;
				matrix.m_posit.m_z = -m_z;
				for (ndInt32 i = 0; i < output.GetCount(); ++i)
				{
					ndAnimKeyframe& keyFrame = output[i];
					ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
					ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
					const ndMatrix& localMatrix = effector->GetLocalMatrix1();
					ndVector p0(localMatrix.TransformVector(keyFrame.m_posit));
					ndVector p1(matrix.TransformVector(p0));
					ndVector p2(localMatrix.UntransformVector(p1));
					keyFrame.m_posit = p2;
				}
			}

			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
			ndReal m_pitch;
			ndReal m_yaw;
			ndReal m_roll;
			ndReal m_animSpeed;
		};
		
		ndModelQuadruped()
			:ndModelArticulation()
		{
		}

		ndVector CalculateCom() const
		{
			ndVector com(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = m_bodies[i];
				const ndMatrix matrix(body->GetMatrix());
				com += matrix.TransformVector(body->GetCentreOfMass() * body->GetMassMatrix().m_w);
			}
			com = com.Scale(m_invMass);
			com.m_w = ndFloat32(1.0f);
			return com;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndVector upVector(rootBody->GetMatrix().m_up);
			for (ndInt32 i = 0; i < 4; ++i)
			{
				const ndEffectorInfo& info = m_effectorsInfo[i];
				const ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info.m_effector;
				effector->DebugJoint(context);

				//ndMatrix lookAtMatrix0;
				//ndMatrix lookAtMatrix1;
				//ndJointBilateralConstraint* const footJoint = info.m_footHinge;
				//footJoint->DebugJoint(context);
				//footJoint->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
				//ndVector updir(lookAtMatrix1.m_posit + upVector.Scale(-1.0f));
				//context.DrawLine(lookAtMatrix1.m_posit, updir, ndVector(0.0f, 0.0f, 0.0f, 1.0f));
			}

			ndFixSizeArray<ndVector, 4> contactPoints;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				if (keyFrame.m_rotation.m_w > ndFloat32(0.0f))
				{
					ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
					ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
					ndBodyKinematic* const body = effector->GetBody0();
					contactPoints.PushBack(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit));
				}
			}

			ndMatrix comMatrix(m_rootNode->m_body->GetAsBodyKinematic()->GetMatrix());
			comMatrix.m_posit = CalculateCom();
			context.DrawFrame(comMatrix);

			if (contactPoints.GetCount() >= 3)
			{
				ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
				rotation.TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
				ndInt32 supportCount = ndConvexHull2d(&contactPoints[0], contactPoints.GetCount());
				rotation.OrthoInverse().TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
				ndVector p0(contactPoints[supportCount - 1]);
				ndBigVector bigPolygon[16];
				for (ndInt32 i = 0; i < supportCount; ++i)
				{
					bigPolygon[i] = contactPoints[i];
					context.DrawLine(contactPoints[i], p0, ndVector::m_zero);
					p0 = contactPoints[i];
				}

				ndBigVector p0Out;
				ndBigVector p1Out;
				ndBigVector ray_p0(comMatrix.m_posit);
				ndBigVector ray_p1(comMatrix.m_posit);
				ray_p1.m_y -= 1.0f;
				
				ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
				context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
				context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			}
			else if (contactPoints.GetCount() == 2)
			{
				context.DrawLine(contactPoints[0], contactPoints[1], ndVector::m_zero);
			}
		}

		void InitState()
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

			ndVector com(ndVector::m_zero);
			ndFixSizeArray<ndVector, 32> bodiesCom;
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = m_bodies[i];
				const ndMatrix matrix(body->GetMatrix());
				ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
				bodiesCom.PushBack(bodyCom);
				//com += matrix.TransformVector(body->GetCentreOfMass() * body->GetMassMatrix().m_w);
				com += bodyCom.Scale(body->GetMassMatrix().m_w);
			}
			com = com.Scale(m_invMass);

			ndVector gyroTorque(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = m_bodies[i];

				const ndMatrix matrix(body->GetMatrix());
				const ndVector omega(body->GetOmega());
				const ndVector comDist(matrix.TransformVector(body->GetCentreOfMass()) - com);
				m_comDist[i] = comDist;
				ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				gyroTorque += omega.CrossProduct(bodyInertia.RotateVector(omega));
			}

			m_gyroTorque = gyroTorque;
		}

		ndVector CalculateTorque()
		{
			ndVector torque(m_gyroTorque);
			m_invDynamicsSolver.Solve();
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = m_bodies[i];

				ndVector r(m_comDist[i]);
				ndVector f(m_invDynamicsSolver.GetBodyForce(body));
				ndVector t(m_invDynamicsSolver.GetBodyTorque(body));
				torque += (t + r.CrossProduct(f));
			}
			return torque;
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndVector veloc;
			m_animBlendTree->Update(timestep * m_control->m_animSpeed);
			m_animBlendTree->Evaluate(m_animPose, veloc);

			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);

			ndJointBilateralConstraint* joint[4];
			ndVector upVector(rootBody->GetMatrix().m_up.Scale (-1.0f));
			for (ndInt32 i = 0; i < 4; ++i)
			{
				ndEffectorInfo* const info = (ndEffectorInfo*)m_animPose[i].m_userData;
				ndAssert(info == &m_effectorsInfo[i]);
				joint[i] = *info->m_effector;

				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
				ndVector posit (m_animPose[i].m_posit);
				effector->SetLocalTargetPosition(posit);
				effector->SetSwivelAngle(0.0f);

				// calculate lookAt angle
				ndMatrix lookAtMatrix0;
				ndMatrix lookAtMatrix1;
				info->m_footHinge->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);

				ndMatrix upMatrix(ndGetIdentityMatrix());
				upMatrix.m_front = lookAtMatrix1.m_front;
				upMatrix.m_right = (upMatrix.m_front.CrossProduct(upVector) & ndVector::m_triplexMask).Normalize();
				upMatrix.m_up = upMatrix.m_right.CrossProduct(upMatrix.m_front);
				upMatrix = upMatrix * lookAtMatrix0.OrthoInverse();
				const ndFloat32 angle = ndAtan2(upMatrix.m_up.m_z, upMatrix.m_up.m_y);
				info->m_footHinge->SetTargetAngle(angle);
			}

			InitState();

			m_invDynamicsSolver.SolverBegin(skeleton, joint, 4, world, timestep);
			//m_invDynamicsSolver.Solve();
			ndVector torque(CalculateTorque());
			ndTrace(("%f %f %f\n", torque.m_x, torque.m_y, torque.m_z));

			m_invDynamicsSolver.SolverEnd();
		}

		ndAnimationPose m_animPose;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;

		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;

		ndVector m_gyroTorque;
		ndFixSizeArray<ndVector, 32> m_comDist;
		ndFixSizeArray<ndBodyKinematic*, 32> m_bodies;
		ndFloat32 m_invMass;
	};

	class ndModelUI: public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, const ndSharedPtr<ndModel>& quadruped)
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
		
			ndModelQuadruped* const model = (ndModelQuadruped*)*m_model;
			ndModelQuadruped::ndUIControlNode* const control = model->m_control;
			
			bool change = false;
			ImGui::Text("position x");
			change = change | ImGui::SliderFloat("##x", &control->m_x, -0.1f, 0.1f);
			ImGui::Text("position y");
			change = change | ImGui::SliderFloat("##y", &control->m_y, -0.2f, 0.1f);
			ImGui::Text("position z");
			change = change | ImGui::SliderFloat("##z", &control->m_z, -0.15f, 0.15f);

			ImGui::Text("pitch");
			change = change | ImGui::SliderFloat("##pitch", &control->m_pitch, -15.0f, 15.0f);
			ImGui::Text("yaw");
			change = change | ImGui::SliderFloat("##yaw", &control->m_yaw, -20.0f, 20.0f);
			ImGui::Text("roll");
			change = change | ImGui::SliderFloat("##roll", &control->m_roll, -15.0f, 15.0f);

			ImGui::Text("animSpeed");
			change = change | ImGui::SliderFloat("##animSpeed", &control->m_animSpeed, 0.0f, 1.0f);

			if (change)
			{
				ndBodyKinematic* const body = m_model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
				body->SetSleepState(false);
			}
		}

		ndSharedPtr<ndModel> m_model;
	};

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
	{
		ndFloat32 mass = 10.0f;
		ndFloat32 radius = 0.25f;
		ndFloat32 limbMass = 0.5f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadios = 0.06f;

		ndModelQuadruped* const model = new ndModelQuadruped();

		ndPhysicsWorld* const world = scene->GetWorld();
		ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		ndSharedPtr<ndBody> torso (world->GetBody(AddSphere(scene, matrixLocation, mass, radius, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(torso);
		
		ndMatrix location(matrixLocation);
		location.m_posit.m_y = floor.m_y + 0.5f;
		torso->SetMatrix(location);
		
		ndDemoEntity* const entity = (ndDemoEntity*)torso->GetNotifyCallback()->GetUserData();
		entity->SetMeshMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * ndPitchMatrix(90.0f * ndDegreeToRad));
		
		ndMatrix matrix(ndRollMatrix(45.0f * ndDegreeToRad));
		matrix.m_posit.m_x = radius * 0.9f;
		matrix.m_posit.m_y = -radius * 0.5f;
		
		ndFloat32 angles[] = { 300.0f, 240.0f, 120.0f, 60.0f };
		ndFloat32 offset[] = { -0.3f, 0.3f, -0.3f, 0.3f };
		ndFloat32 phase[] = { 0.0f, 0.5f, 0.25f, 0.75f };

		ndSharedPtr<ndAnimationSequence> sequence(new ndModelQuadruped::ndPoseGenerator());
		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndModelQuadruped::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);

		ndModelQuadruped::ndPoseGenerator* const poseGenerator = (ndModelQuadruped::ndPoseGenerator*)*sequence;
		const ndVector upDir(location.m_up);
		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndMatrix limbPivotLocation(matrix * ndYawMatrix(angles[i] * ndDegreeToRad));
			limbPivotLocation.m_posit += torso->GetMatrix().m_posit;
			limbPivotLocation.m_posit.m_w = 1.0f;
		
			// add leg thigh
			const ndVector thighPivot(limbPivotLocation.m_posit);
		
			ndFloat32 workSpace = 0.0f;
			ndModelArticulation::ndNode* thighNode = nullptr;
			{
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
				ndSharedPtr<ndBody> thigh (world->GetBody(AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength)));
				thigh->SetMatrix(bodyMatrix);
				ndSharedPtr<ndJointBilateralConstraint> ballJoint (new ndIkJointSpherical(limbPivotLocation, thigh->GetAsBodyKinematic(), torso->GetAsBodyKinematic()));
				thighNode = model->AddLimb(modelRoot, thigh, ballJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
		
				workSpace += limbLength;
			}
		
			// add calf0
			ndModelArticulation::ndNode* calf0Node = nullptr;
			{
				limbPivotLocation = ndRollMatrix(-90.0f * ndDegreeToRad) * limbPivotLocation;
		
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
				ndSharedPtr<ndBody> calf0 (world->GetBody(AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength)));
				calf0->SetMatrix(bodyMatrix);
		
				ndMatrix caffPinAndPivotFrame(ndGetIdentityMatrix());
				ndFloat32 sign = angles[i] > 180.0f ? -1.0f : 1.0f;
				caffPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(sign);
				caffPinAndPivotFrame.m_up = limbPivotLocation.m_front;
				caffPinAndPivotFrame.m_right = caffPinAndPivotFrame.m_front.CrossProduct(caffPinAndPivotFrame.m_up);
				caffPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
				ndSharedPtr<ndJointBilateralConstraint> hingeJoint (new ndIkJointHinge(caffPinAndPivotFrame, calf0->GetAsBodyKinematic(), thighNode->m_body->GetAsBodyKinematic()));
		
				// add joint limit to prevent knee from flipping
				 ndIkJointHinge* const hinge = (ndIkJointHinge*)*hingeJoint;
				hinge->SetLimitState(true);
				hinge->SetLimits(-70.0f * ndDegreeToRad, 70.0f * ndDegreeToRad);
				calf0Node = model->AddLimb(thighNode, calf0, hingeJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
				workSpace += limbLength;
			}
		
			// add calf1
			ndJointHinge* footHinge = nullptr;
			ndModelArticulation::ndNode* footNode = nullptr;
			{
				ndFloat32 lenght = limbLength * 0.5f;
				limbPivotLocation = ndRollMatrix(-45.0f * ndDegreeToRad) * limbPivotLocation;
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(lenght * 0.5f);
		
				ndSharedPtr<ndBody> foot (world->GetBody(AddCapsule(scene, bodyMatrix, limbMass * 0.5f, limbRadios, limbRadios, lenght)));
				foot->SetMatrix(bodyMatrix);

				// set a Material with zero restitution for the feet
				ndShapeInstance& instanceShape = foot->GetAsBodyDynamic()->GetCollisionShape();
				instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_frictionTest;
		
				ndMatrix footPinAndPivotFrame(ndGetIdentityMatrix());
				footPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(-1.0f);
				footPinAndPivotFrame.m_up = limbPivotLocation.m_front;
				footPinAndPivotFrame.m_right = footPinAndPivotFrame.m_front.CrossProduct(footPinAndPivotFrame.m_up);
				footPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
		
				// add joint limit to prevent knee from flipping
				footHinge = new ndJointHinge(footPinAndPivotFrame, foot->GetAsBodyKinematic(), calf0Node->m_body->GetAsBodyKinematic());
				footHinge->SetLimitState(true);
				footHinge->SetLimits(-20.0f * ndDegreeToRad, 20.0f * ndDegreeToRad);
				footHinge->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
		
				//ndSharedPtr<ndJointBilateralConstraint> hinge(footHinge);
				footNode = model->AddLimb(calf0Node, foot, ndSharedPtr<ndJointBilateralConstraint>(footHinge));
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(lenght);
				workSpace += lenght;
			}
		
			// add leg effector
			{
				ndBodyKinematic* const targetBody = footNode->m_body->GetAsBodyKinematic();
		
				ndFloat32 angle(i < 2 ? -90.0f : 90.0f);
				ndMatrix effectorToeFrame(ndGetIdentityMatrix());
				ndMatrix effectorRefFrame(ndYawMatrix(angle * ndDegreeToRad));
				effectorRefFrame.m_posit = thighPivot;
				effectorToeFrame.m_posit = limbPivotLocation.m_posit;
		
				ndFloat32 regularizer = 0.001f;
				ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorToeFrame.m_posit, effectorRefFrame, targetBody, torso->GetAsBodyKinematic());
				effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
				effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
				effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.9f);

				ndModelQuadruped::ndEffectorInfo info(ndSharedPtr<ndJointBilateralConstraint> (effector), footHinge);
				model->m_effectorsInfo.PushBack(info);

				ndAnimKeyframe keyFrame;
				keyFrame.m_userData = &model->m_effectorsInfo[model->m_effectorsInfo.GetCount() - 1];
				model->m_animPose.PushBack(keyFrame);
				poseGenerator->AddTrack();
				poseGenerator->m_phase[i] = phase[i];
				poseGenerator->m_offset[i] = offset[i];
			}
		}

		// init physics pre-computed values
		ndFloat32 totalMass = ndFloat32(0.0f);
		for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			model->m_bodies.PushBack(body);
			model->m_comDist.PushBack(ndVector::m_zero);
			totalMass += body->GetMassMatrix().m_w;
		}
		model->m_invMass = ndFloat32(1.0f) / totalMass;

		return model;
	}
#endif
}

using namespace ndQuadruped_1;

void ndQuadrupedTest_1(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildFlatPlane(scene, true);

	// register a zero restitution and high friction material for the feet
	ndApplicationMaterial material;
	material.m_restitution = 0.0f;
	material.m_staticFriction0 = 0.8f;
	material.m_staticFriction1 = 0.8f;
	material.m_dynamicFriction0 = 0.8f;
	material.m_dynamicFriction1 = 0.8f;
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_frictionTest, ndDemoContactCallback::m_default);
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
	ndSharedPtr<ndModel> model(BuildModel(scene, matrix));
	world->AddModel(model);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
	//world->AddJoint(fixJoint);

	ndSharedPtr<ndUIEntity> quadrupedUI (new ndModelUI(scene, model));
	scene->Set2DDisplayRenderFunction(quadrupedUI);
	
	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);

	//ndFileFormatSave xxxx;
	//xxxx.SaveWorld(scene->GetWorld(), "xxxx.nd");
}
