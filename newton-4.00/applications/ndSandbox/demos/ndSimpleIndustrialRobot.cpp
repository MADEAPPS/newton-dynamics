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
#include "ndDemoMesh.h"
#include "ndUIEntity.h"
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndSimpleRobot
{
	class ndDefinition
	{
		public:
		enum ndJointType
		{
			m_root,
			m_hinge,
			m_slider,
			m_effector
		};

		char m_boneName[32];
		ndJointType m_type;
		ndFloat32 m_mass;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "base", ndDefinition::m_root, 100.0f, 0.0f, 0.0f},
		{ "base_rotator", ndDefinition::m_hinge, 50.0f, -1.0e10f, 1.0e10f},
		{ "arm_0", ndDefinition::m_hinge , 20.0f, -120.0f * ndDegreeToRad, 60.0f * ndDegreeToRad},
		{ "arm_1", ndDefinition::m_hinge , 20.0f, -90.0f * ndDegreeToRad, 60.0f * ndDegreeToRad},
		{ "arm_2", ndDefinition::m_hinge , 20.0f, -1.0e10f, 1.0e10f},
		{ "arm_3", ndDefinition::m_hinge , 10.0f, -1.0e10f, 1.0e10f},
		{ "arm_4", ndDefinition::m_hinge , 10.0f, -1.0e10f, 1.0e10f},
		{ "gripperLeft", ndDefinition::m_slider , 5.0f, -0.2f, 0.03f},
		{ "gripperRight", ndDefinition::m_slider , 5.0f, -0.2f, 0.03f},
		{ "effector", ndDefinition::m_effector , 0.0f, 0.0f, 0.0f},
	};

	#define ND_MIN_X_SPAND	ndReal (-2.2f)
	#define ND_MAX_X_SPAND	ndReal ( 1.5f)
	#define ND_MIN_Z_SPAND	ndReal (-1.5f)
	#define ND_MAX_Z_SPAND	ndReal ( 1.5f)

	class ndRobotBodyNotify : public ndDemoEntityNotify
	{
		public:
		ndRobotBodyNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyKinematic* const parentBody = nullptr)
			:ndDemoEntityNotify(manager, entity, parentBody)
		{
		}

		virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
		{
			const ndBodyKinematic* const body0 = ((ndBody*)GetBody())->GetAsBodyKinematic();
			const ndBodyKinematic* const body1 = ((ndBody*)otherBody)->GetAsBodyKinematic();
			const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
			const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();

			if ((instanceShape0.m_shapeMaterial.m_userId == ndDemoContactCallback::m_modelPart) &&
				(instanceShape1.m_shapeMaterial.m_userId == ndDemoContactCallback::m_modelPart))
			{
				return false;
			}
			return true;
		}
	};

	class RobotModelNotify : public ndModelNotify
	{
		public:
		RobotModelNotify(ndModelArticulation* const robot, bool showDebug, ndFixSizeArray<ndBodyKinematic*, 16>& backGround)
			:ndModelNotify()
			,m_effectorLocalBase(ndGetIdentityMatrix())
			,m_effectorLocalTarget(ndGetIdentityMatrix())
			,m_effectorReference(ndGetIdentityMatrix())
			,m_world(nullptr)
			,m_x(0.0f)
			,m_z(0.0f)
			,m_azimuth(0.0f)
			,m_yaw(0.0f)
			,m_pitch(0.0f)
			,m_roll(0.0f)
			,m_gripperPosit(0.0f)
			,m_timestep(ndFloat32(0.0f))
			,m_showDebug(showDebug)
		{

			for (ndInt32 i = 0; i < backGround.GetCount(); ++i)
			{
				m_backGround.PushBack(backGround[i]);
			}
			Init(robot);
		}

		RobotModelNotify(const RobotModelNotify& src)
			:ndModelNotify(src)
		{
			//Init(robot);
			ndAssert(0);
		}

		//RobotModelNotify(const RobotModelNotify& src)
		//	:ndModelNotify(src)
		//	,m_controller(src.m_controller)
		//{
		//	//Init(robot);
		//	ndAssert(0);
		//}

		~RobotModelNotify()
		{
		}

		ndModelNotify* Clone() const
		{
			return new RobotModelNotify(*this);
		}

		void Init(ndModelArticulation* const robot)
		{
			m_rootBody = robot->GetRoot()->m_body->GetAsBodyDynamic();
			m_leftGripper = (ndJointSlider*)robot->FindByName("leftGripper")->m_joint->GetAsBilateral();
			m_rightGripper = (ndJointSlider*)robot->FindByName("rightGripper")->m_joint->GetAsBilateral();
			m_effector = (ndIk6DofEffector*)robot->FindLoopByName("effector")->m_joint->GetAsBilateral();

			m_baseLimitJoint = (ndJointHinge*)robot->FindByName("arm_0")->m_joint->GetAsBilateral();
			m_elbowLimitJoint = (ndJointHinge*)robot->FindByName("arm_1")->m_joint->GetAsBilateral();

			ndBodyDynamic* const rootBody = robot->GetRoot()->m_body->GetAsBodyDynamic();
			ndDemoEntity* const rootEntity = (ndDemoEntity*)rootBody->GetNotifyCallback()->GetUserData();
			ndDemoEntity* const effectorEntity = rootEntity->Find("effector");

			const ndMatrix referenceFrame(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
			const ndMatrix effectorFrame(effectorEntity->CalculateGlobalMatrix());

			m_effectorLocalBase = referenceFrame * m_effector->GetBody1()->GetMatrix().OrthoInverse();
			m_effectorLocalTarget = effectorFrame * m_effector->GetBody0()->GetMatrix().OrthoInverse();;
			m_effectorReference = effectorFrame * referenceFrame.OrthoInverse();
		}

		void ResetModel()
		{
			ndTrace(("Reset Model\n"));
		}

		ndMatrix CalculateTargetMatrix() const
		{
			ndFloat32 x = m_x + m_effectorReference.m_posit.m_x;
			ndFloat32 y = m_effectorReference.m_posit.m_y;
			ndFloat32 z = m_z + m_effectorReference.m_posit.m_z;

			const ndMatrix aximuthMatrix(ndPitchMatrix(m_azimuth));
			const ndVector localPosit(ndVector::m_wOne + aximuthMatrix.RotateVector(ndVector(x, y, z, ndFloat32(1.0f))));

			ndMatrix targetMatrix(ndPitchMatrix(m_pitch) * ndYawMatrix(m_yaw) * ndRollMatrix(m_roll));

			targetMatrix.m_posit = localPosit;
			return targetMatrix;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			const ndVector color(1.0f, 0.0f, 0.0f, 1.0f);

			const ndMatrix baseMatrix(m_effector->CalculateGlobalBaseMatrix1());
			context.DrawFrame(baseMatrix);
			context.DrawPoint(baseMatrix.m_posit, color, ndFloat32(5.0f));

			const ndMatrix effectorTarget(CalculateTargetMatrix() * m_effector->CalculateGlobalBaseMatrix1());
			context.DrawFrame(effectorTarget);
			context.DrawPoint(effectorTarget.m_posit, color, ndFloat32(5.0f));

			const ndMatrix effectorMatrix(m_effector->CalculateGlobalMatrix0());
			context.DrawFrame(effectorMatrix);
			context.DrawPoint(effectorMatrix.m_posit, color, ndFloat32(5.0f));
		}

		const ndVector CalculateTargetPosit() const
		{
			const ndMatrix currentEffectorMatrix(m_effector->GetEffectorMatrix());
			
			// intepolate in local space;
			ndFloat32 azimuth = -ndAtan2(currentEffectorMatrix.m_posit.m_y, currentEffectorMatrix.m_posit.m_z);
			
			const ndVector localPosit(ndPitchMatrix(-azimuth).RotateVector(currentEffectorMatrix.m_posit) - m_effectorReference.m_posit);
			
			// move on the plane with constant speed
			ndFloat32 planeSpeed = 0.05f;
			ndFloat32 dx = m_x - localPosit.m_x;
			ndFloat32 dz = m_z - localPosit.m_z;
			ndFloat32 mag = ndSqrt(dx * dx + dz * dz);
			ndFloat32 invPositMag = 1.0f;
			if (mag > planeSpeed)
			{
				invPositMag = planeSpeed / mag;
			}
			ndFloat32 x = localPosit.m_x + dx * invPositMag;
			ndFloat32 z = localPosit.m_z + dz * invPositMag;
			
			ndFloat32 azimuthSpeed = 5.0f * ndDegreeToRad;
			ndFloat32 deltaAzimuth = ndAnglesSub(m_azimuth, azimuth);
			if (ndAbs(deltaAzimuth) > azimuthSpeed)
			{
				deltaAzimuth = azimuthSpeed * ndSign(deltaAzimuth);
			}
			ndFloat32 angle = azimuth + deltaAzimuth;
			
			// now calculate the in between matrix;
			ndFloat32 x1 = x + m_effectorReference.m_posit.m_x;
			ndFloat32 y1 = m_effectorReference.m_posit.m_y;
			ndFloat32 z1 = z + m_effectorReference.m_posit.m_z;
			
			const ndMatrix azimuthMatrix1(ndPitchMatrix(angle));
			const ndVector posit(ndVector::m_wOne + azimuthMatrix1.RotateVector(ndVector(x1, y1, z1, ndFloat32(1.0f))));
			return posit;
		}

		const ndQuaternion CalculateTargetRotation() const
		{
			const ndMatrix currentEffectorMatrix(m_effector->GetEffectorMatrix());
			const ndQuaternion currentRotation(currentEffectorMatrix);

			const ndMatrix targetMatrix(ndPitchMatrix(m_pitch) * ndYawMatrix(m_yaw) * ndRollMatrix(m_roll));
#if 0
			ndQuaternion targetRotation(targetMatrix);
#else
			ndQuaternion targetRotation(currentRotation);
			ndFloat32 angleCos = currentEffectorMatrix.m_front.DotProduct(targetMatrix.m_front).GetScalar();
			if (angleCos < 0.999f)
			{
				ndFloat32 angle = ndAcos(angleCos);
				ndVector pin(currentEffectorMatrix.m_front.CrossProduct(targetMatrix.m_front).Normalize());
				targetRotation = currentRotation * ndQuaternion(pin, angle);
				//ndQuaternion targetRotation2(targetMatrix);
				//ndQuaternion targetRotation3(targetMatrix);
				//ndMatrix xxxx(ndCalculateMatrix(targetRotation));
				//ndMatrix xxxx1(targetMatrix * xxxx);
				//ndMatrix xxxx2(targetMatrix * xxxx);
			}
#endif
			if (currentRotation.DotProduct(targetRotation).GetScalar() < 0.0f)
			{
				targetRotation = targetRotation.Scale(-1.0f);
			}

			ndVector omega(currentRotation.CalcAverageOmega(targetRotation, 1.0f));
			ndFloat32 omegaMag = ndSqrt(omega.DotProduct(omega).GetScalar());

			ndFloat32 omegaSpeed = 5.0f * ndDegreeToRad;

			ndQuaternion rotation(targetRotation);
			if (omegaMag > omegaSpeed)
			{
				omega = omega.Normalize().Scale(omegaSpeed);
				rotation = currentRotation.IntegrateOmega(omega, 1.0f);
			}
			return rotation;
		}

		const ndMatrix CalculateNextTargetMatrix() const
		{
			const ndVector posit(CalculateTargetPosit());
			const ndQuaternion rotation(CalculateTargetRotation());
			return ndCalculateMatrix(rotation, posit);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			m_world = world;
			m_timestep = timestep;

			if (m_baseLimitJoint->GetJointHitLimits() || m_elbowLimitJoint->GetJointHitLimits())
			{
				const ndMatrix currentEffectorMatrix(m_effector->GetEffectorMatrix());
				m_azimuth = -ndAtan2(currentEffectorMatrix.m_posit.m_y, currentEffectorMatrix.m_posit.m_z);
				const ndVector localPosit(ndPitchMatrix(-m_azimuth).RotateVector(currentEffectorMatrix.m_posit) - m_effectorReference.m_posit);
				m_x = localPosit.m_x;
				m_z = localPosit.m_z;
			}

			m_leftGripper->SetOffsetPosit(-m_gripperPosit * 0.5f);
			m_rightGripper->SetOffsetPosit(-m_gripperPosit * 0.5f);

			const ndMatrix targetMatrix(CalculateNextTargetMatrix());
			m_effector->SetOffsetMatrix(targetMatrix);

			//ndModelArticulation* const robot = GetModel()->GetAsModelArticulation();
			//ndJointHinge* xxxx4 = (ndJointHinge*)robot->FindByName("arm_4")->m_joint->GetAsBilateral();
			//ndJointHinge* xxxx3 = (ndJointHinge*)robot->FindByName("arm_3")->m_joint->GetAsBilateral();
			//ndJointHinge* xxxx2 = (ndJointHinge*)robot->FindByName("arm_2")->m_joint->GetAsBilateral();
			//ndJointHinge* xxxx1 = (ndJointHinge*)robot->FindByName("arm_1")->m_joint->GetAsBilateral();
		}

		void PostUpdate(ndWorld* const, ndFloat32)
		{
		}

		void PostTransformUpdate(ndWorld* const, ndFloat32)
		{
		}

		ndMatrix m_effectorLocalBase;
		ndMatrix m_effectorLocalTarget;
		ndMatrix m_effectorReference;

		ndBodyDynamic* m_rootBody;
		ndJointSlider* m_leftGripper;
		ndJointSlider* m_rightGripper;
		ndJointHinge* m_baseLimitJoint;
		ndJointHinge* m_elbowLimitJoint;
		ndIk6DofEffector* m_effector;

		ndWorld* m_world;
		ndFixSizeArray<ndBodyKinematic*, 16> m_backGround;

		ndReal m_x;
		ndReal m_z;
		ndReal m_azimuth;
		ndReal m_yaw;
		ndReal m_pitch;
		ndReal m_roll;
		ndReal m_gripperPosit;
		ndFloat32 m_timestep;
		bool m_showDebug;

		friend class ndRobotUI;
	};

	class ndRobotUI : public ndUIEntity
	{
		public:
		ndRobotUI(ndDemoEntityManager* const scene, RobotModelNotify* const robot)
			:ndUIEntity(scene)
			,m_robot(robot)
		{
		}
	
		~ndRobotUI()
		{
		}
	
		virtual void RenderUI()
		{
		}
	
		virtual void RenderHelp()
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			m_scene->Print(color, "Control panel");

			ndInt8 change = 0;
			change = change | ndInt8(ImGui::SliderFloat("y", &m_robot->m_x, ND_MIN_X_SPAND, ND_MAX_X_SPAND));
			change = change | ndInt8(ImGui::SliderFloat("z", &m_robot->m_z, ND_MIN_Z_SPAND, ND_MAX_Z_SPAND));
			change = change | ndInt8(ImGui::SliderFloat("azimuth", &m_robot->m_azimuth, -ndPi, ndPi));
			change = change | ndInt8(ImGui::SliderFloat("pitch", &m_robot->m_pitch, -ndPi, ndPi));
			change = change | ndInt8(ImGui::SliderFloat("yaw", &m_robot->m_yaw, -ndPi * 0.35f, ndPi * 0.8f));
			change = change | ndInt8(ImGui::SliderFloat("roll", &m_robot->m_roll, -ndPi, ndPi));
			change = change | ndInt8(ImGui::SliderFloat("gripper", &m_robot->m_gripperPosit, -0.2f, 0.4f));

			//m_robot->m_azimuth = 60.0f * ndDegreeToRad;

			bool newTarget = ndInt8(ImGui::Button("random target"));
			if (newTarget)
			{
				change = 1;

				m_robot->m_pitch = ndReal((2.0f * ndRand() - 1.0f) * ndPi);
				m_robot->m_roll = ndReal((2.0f * ndRand() - 1.0f) * ndPi);
				m_robot->m_yaw = ndReal(-ndPi * 0.35f + ndRand() * (ndPi * 0.8f - (-ndPi * 0.35f)));

				m_robot->m_azimuth = ndReal((2.0f * ndRand() - 1.0f) * ndPi);
				m_robot->m_x = ndReal(ND_MIN_X_SPAND + ndRand() * (ND_MAX_X_SPAND - ND_MIN_X_SPAND));
				m_robot->m_z = ndReal(ND_MIN_Z_SPAND + ndRand() * (ND_MAX_Z_SPAND - ND_MIN_Z_SPAND));
			}

			bool newBoxTarget = ndInt8(ImGui::Button("random box target"));
			if (newBoxTarget)
			{
				change = 1;
				
				ndInt32 index = ndInt32 (ndRandInt() % 4);
				ndBodyKinematic* const body = m_robot->m_backGround[index];
				const ndMatrix baseMatrix(m_robot->m_effector->CalculateGlobalBaseMatrix1());

				ndMatrix bodyMatrix(ndGramSchmidtMatrix(ndVector(0.0f, 1.0f, 0.0f, 0.0f)));
				bodyMatrix.m_posit = body->GetMatrix().m_posit;
				ndMatrix matrix(bodyMatrix * baseMatrix.OrthoInverse());
				matrix.m_posit.m_x += 0.5f;
				
				// intepolate in local space;
				ndFloat32 azimuth = -ndAtan2(matrix.m_posit.m_y, matrix.m_posit.m_z);
				const ndVector localPosit(ndPitchMatrix(-azimuth).RotateVector(matrix.m_posit) - m_robot->m_effectorReference.m_posit);
				
				m_robot->m_azimuth = azimuth;
				m_robot->m_x = localPosit.m_x;
				m_robot->m_z = localPosit.m_z;

				ndVector euler1;
				ndVector euler(matrix.CalcPitchYawRoll(euler1));
				
				m_robot->m_yaw = euler.m_y;
				m_robot->m_roll = euler.m_z;
				m_robot->m_pitch = euler.m_x;
			}

			if (change)
			{
				m_robot->GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic()->SetSleepState(false);
			}

			//m_robot->m_yaw = 45.0f * ndDegreeToRad;
		}
	
		RobotModelNotify* m_robot;
	};

	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndFloat32 mass, ndBodyDynamic* const parentBone)
	{
		ndSharedPtr<ndShapeInstance> shapePtr(entityPart->CreateCollisionFromChildren());
		ndShapeInstance* const shape = *shapePtr;
		ndAssert(shape);

		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyKinematic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(mass, *shape);
		body->SetNotifyCallback(new ndRobotBodyNotify(scene, entityPart, parentBone));

		ndShapeInstance& instanceShape = body->GetCollisionShape();
		instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;

		return body->GetAsBodyDynamic();
	}

	void NormalizeInertia(ndModelArticulation* const model)
	{
		for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

			ndVector inertia(body->GetMassMatrix());
			ndFloat32 maxInertia = ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z);
			ndFloat32 minInertia = ndMin(ndMin(inertia.m_x, inertia.m_y), inertia.m_z);
			if (minInertia < maxInertia * 0.125f)
			{
				minInertia = maxInertia * 0.125f;
				for (ndInt32 j = 0; j < 3; ++j)
				{
					if (inertia[j] < minInertia)
					{
						inertia[j] = minInertia;
					}
				}
			}
			body->SetMassMatrix(inertia);
		}
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, ndDemoEntity* const modelMesh, const ndMatrix& location)
	{
		// make a clone of the mesh and add it to the scene
		ndModelArticulation* const model = new ndModelArticulation();

		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const entity = modelMesh->CreateClone();
		scene->AddEntity(entity);

		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);

		// find the floor location 
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;
				
		rootEntity->ResetMatrix(matrix);

		// add the root body
		ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr));

		rootBody->SetMatrix(rootEntity->CalculateGlobalMatrix());

		// add the root body to the model
		ndModelArticulation::ndNode* const modelNode = model->AddRootBody(rootBody);
		
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndModelArticulation::ndNode*, 32> parentBones;
		for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
		{
			childEntities.PushBack(child);
			parentBones.PushBack(modelNode);
		}
		
		const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (parentBones.GetCount())
		{
			ndDemoEntity* const childEntity = childEntities.Pop();
			ndModelArticulation::ndNode* parentBone = parentBones.Pop();

			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; ++i)
			{
				const ndDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					ndTrace(("name: %s\n", name));
					if (definition.m_type == ndDefinition::m_hinge)
					{
						ndSharedPtr<ndBody> childBody (CreateBodyPart(scene, childEntity, definition.m_mass, parentBone->m_body->GetAsBodyDynamic()));
						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndSharedPtr<ndJointBilateralConstraint> hinge (new ndJointHinge(pivotMatrix, childBody->GetAsBodyKinematic(), parentBone->m_body->GetAsBodyKinematic()));

						ndJointHinge* const hingeJoint = (ndJointHinge*)*hinge;
						hingeJoint->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						if ((definition.m_minLimit > -1000.0f) && (definition.m_maxLimit < 1000.0f))
						{
							hingeJoint->SetLimitState(true);
						}

						parentBone = model->AddLimb(parentBone, childBody, hinge);
						parentBone->m_name = name;
					}
					else if (definition.m_type == ndDefinition::m_slider)
					{
						ndSharedPtr<ndBody> childBody(CreateBodyPart(scene, childEntity, definition.m_mass, parentBone->m_body->GetAsBodyDynamic()));
						
						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndSharedPtr<ndJointBilateralConstraint> slider (new ndJointSlider(pivotMatrix, childBody->GetAsBodyKinematic(), parentBone->m_body->GetAsBodyKinematic()));

						ndJointSlider* const sliderJoint = (ndJointSlider*)*slider;
						sliderJoint->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						sliderJoint->SetLimitState(true);
						sliderJoint->SetAsSpringDamper(0.1f, 1000.0f, 50.0f);
						parentBone = model->AddLimb(parentBone, childBody, slider);

						if (!strstr(definition.m_boneName, "Left"))
						{
							parentBone->m_name = "leftGripper";
						}
						else
						{
							parentBone->m_name = "rightGripper";
						}
					}
					else if (definition.m_type == ndDefinition::m_effector)
					{
						ndBodyDynamic* const childBody = parentBone->m_body->GetAsBodyDynamic();
						
						const ndMatrix pivotFrame(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
						const ndMatrix effectorFrame(childEntity->CalculateGlobalMatrix());
						ndSharedPtr<ndJointBilateralConstraint> effector (new ndIk6DofEffector(effectorFrame, pivotFrame, childBody, modelNode->m_body->GetAsBodyKinematic()));
						
						ndIk6DofEffector* const effectorJoint = (ndIk6DofEffector*)*effector;
						ndFloat32 relaxation = 1.0e-4f;
						effectorJoint->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
						effectorJoint->SetLinearSpringDamper(relaxation, 10000.0f, 500.0f);
						effectorJoint->SetAngularSpringDamper(relaxation, 10000.0f, 500.0f);
						effectorJoint->SetMaxForce(10000.0f);
						effectorJoint->SetMaxTorque(10000.0f);
						
						// the effector is part of the rig
						model->AddCloseLoop(effector, "effector");
					}
					break;
				}
			}
		
			for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
			{
				childEntities.PushBack(child);
				parentBones.PushBack(parentBone);
			}
		}
		NormalizeInertia(model);
		return model;
	}

	void AddBackgroundScene(ndDemoEntityManager* const scene, const ndMatrix& matrix, ndFixSizeArray<ndBodyKinematic*, 16>& bodyList)
	{
		ndMatrix location(matrix);
		location.m_posit.m_x -= 1.0f;
		location.m_posit.m_z += 2.5f;
		bodyList.PushBack(AddBox(scene, location, 2.0f, 0.3f, 0.4f, 0.7f));
		bodyList.PushBack(AddBox(scene, location, 1.0f, 0.3f, 0.4f, 0.7f));
		
		location = ndYawMatrix(90.0f * ndDegreeToRad) * location;
		location.m_posit.m_x += 1.0f;
		location.m_posit.m_z += 0.5f;
		bodyList.PushBack(AddBox(scene, location, 8.0f, 0.3f, 0.4f, 0.7f));
		bodyList.PushBack(AddBox(scene, location, 4.0f, 0.3f, 0.4f, 0.7f));
	}
}

using namespace ndSimpleRobot;
void ndSimpleIndustrialRobot (ndDemoEntityManager* const scene)
{
	// build a floor
	ndBodyKinematic* const floor = BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("robot.fbx", scene));
	ndMatrix matrix(ndYawMatrix(-90.0f * ndDegreeToRad));
	//ndMatrix matrix(ndYawMatrix(0.0f * ndDegreeToRad));

	ndFixSizeArray<ndBodyKinematic*, 16> backGround;
	AddBackgroundScene(scene, matrix, backGround);

	auto SpawnModel = [scene, &modelMesh, floor, &backGround](const ndMatrix& matrix)
	{
		ndWorld* const world = scene->GetWorld();
		ndModelArticulation* const model = CreateModel(scene, *modelMesh, matrix);
		model->SetNotifyCallback(new RobotModelNotify(model, true, backGround));
		model->AddToWorld(world);
		((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();

		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetRoot()->m_body->GetMatrix(), model->GetRoot()->m_body->GetAsBodyKinematic(), floor));
		world->AddJoint(fixJoint);
		return model;
	};

	ndModelArticulation* const visualModel = SpawnModel(matrix);
	ndSharedPtr<ndUIEntity> robotUI(new ndRobotUI(scene, (RobotModelNotify*)*visualModel->GetNotifyCallback()));
	scene->Set2DDisplayRenderFunction(robotUI);

	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 0.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
