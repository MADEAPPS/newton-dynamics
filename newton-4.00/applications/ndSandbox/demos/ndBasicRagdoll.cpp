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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"

namespace ndRagdoll
{
	class ndDefinition
	{
		public:
		enum ndjointType
		{
			m_root,
			m_hinge,
			m_spherical,
			m_doubleHinge,
			m_effector
		};

		struct ndDampData
		{
			ndDampData()
				:m_spring(0.0f)
				,m_damper(0.25f)
				,m_regularizer(0.1f)
			{
			}

			ndDampData(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
				:m_spring(spring)
				,m_damper(damper)
				,m_regularizer(regularizer)
			{
			}

			ndFloat32 m_spring;
			ndFloat32 m_damper;
			ndFloat32 m_regularizer;
		};

		struct ndJointLimits
		{
			ndFloat32 m_minTwistAngle;
			ndFloat32 m_maxTwistAngle;
			ndFloat32 m_coneAngle;
		};

		struct ndOffsetFrameMatrix
		{
			ndFloat32 m_pitch;
			ndFloat32 m_yaw;
			ndFloat32 m_roll;
		};

		char m_boneName[32];
		ndjointType m_limbType;
		ndFloat32 m_massWeight;
		ndJointLimits m_jointLimits;
		ndOffsetFrameMatrix m_frameBasics;
		ndDampData m_coneSpringData;
		ndDampData m_twistSpringData;
	};

	static ndDefinition ragdollDefinition[] =
	{
		{ "root", ndDefinition::m_root, 1.0f, {}, {} },

		{ "lowerback", ndDefinition::m_spherical, 1.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
		{ "upperback", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "lowerneck", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "upperneck", ndDefinition::m_spherical, 1.0f,{ -60.0f, 60.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		
		{ "lclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "lhumerus", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "lradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "rclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "rhumerus", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "rhipjoint", ndDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "rfemur", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rtibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "lhipjoint", ndDefinition::m_spherical, 1.0f,{ -45.0f, 45.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "lfemur", ndDefinition::m_hinge, 1.0f, { -0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "ltibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

		{ "", ndDefinition::m_root,{},{} },
	};

	class ndRagDollController : public ndModelNotify
	{ 
		public:
		ndRagDollController()
			:ndModelNotify()
		{
		}

		ndSharedPtr<ndBody> CreateBodyPart(
			ndDemoEntityManager* const scene,
			const ndSharedPtr<ndRenderSceneNode>& rootMesh,
			const ndMeshLoader& loader, 
			const ndDefinition& definition,
			ndBodyDynamic* const parentBody)
		{
			ndMesh* const mesh(loader.m_mesh->FindByName(definition.m_boneName));
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollisionFromChildren());
		
			ndAssert(rootMesh->FindByName(definition.m_boneName));
			ndSharedPtr<ndRenderSceneNode> bonePart(rootMesh->FindByName(definition.m_boneName)->GetSharedPtr());
			// create the rigid body that will make this body
			ndMatrix matrix(bonePart->CalculateGlobalTransform());
		
			ndSharedPtr<ndBody> body (new ndBodyDynamic());
			body->SetMatrix(matrix);
			body->GetAsBodyDynamic()->SetCollisionShape(**shape);
			body->GetAsBodyDynamic()->SetMassMatrix(1.0f, **shape);
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, bonePart, parentBody));
			return body;
		}

		ndJointBilateralConstraint* ConnectBodyParts(ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody, const ndDefinition& definition)
		{
			ndMatrix matrix(childBody->GetMatrix());
			ndDefinition::ndOffsetFrameMatrix frameAngle(definition.m_frameBasics);
			ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

			ndDefinition::ndJointLimits jointLimits(definition.m_jointLimits);

			switch (definition.m_limbType)
			{
				case ndDefinition::m_spherical:
				{
					ndJointSpherical* const joint = new ndJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBody);
					joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
					joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
					joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					return joint;
				}

				case ndDefinition::m_hinge:
				{
					ndJointHinge* const joint = new ndJointHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
					joint->SetLimitState(true);
					joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
					joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					return joint;
				}

				case ndDefinition::m_doubleHinge:
				{
					ndJointDoubleHinge* const joint = new ndJointDoubleHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
					joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
					joint->SetLimits1(-45.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
					joint->SetAsSpringDamper0(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					joint->SetAsSpringDamper1(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
					return joint;
				}

				default:
					ndAssert(0);
			}
			return nullptr;
		}

		void CalculateMassDistribution(ndModelArticulation* const ragdoll, ndFloat32 totalMass)
		{
			ndFixSizeArray<ndBodyDynamic*, 256> bodyArray;
			ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
			if (ragdoll->GetRoot())
			{
				stack.PushBack(ragdoll->GetRoot());
				while (stack.GetCount())
				{
					ndInt32 index = stack.GetCount() - 1;
					ndModelArticulation::ndNode* const node = stack[index];
					stack.SetCount(index);

					bodyArray.PushBack(node->m_body->GetAsBodyDynamic());
					for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
					{
						stack.PushBack(child);
					}
				}
			}

			ndFloat32 volume = 0.0f;
			for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
			{
				volume += bodyArray[i]->GetCollisionShape().GetVolume();
			}
			ndFloat32 density = totalMass / volume;

			for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
			{
				ndBodyDynamic* const body = bodyArray[i];
				ndFloat32 mass = density * body->GetCollisionShape().GetVolume();
				ndVector inertia(body->GetMassMatrix().Scale(mass));
				body->SetMassMatrix(inertia);
			}
		}

		void RagdollBuildScript(ndDemoEntityManager* const scene, const ndMeshLoader& loader, const ndMatrix& location)
		{
			ndSharedPtr<ndRenderSceneNode> entityDuplicate(loader.m_renderMesh->Clone());
			entityDuplicate->SetTransform(location);
			entityDuplicate->SetTransform(location);
			scene->AddEntity(entityDuplicate);

			ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, entityDuplicate, loader, ragdollDefinition[0], nullptr));

			ndModelArticulation* const ragdoll = (ndModelArticulation*)GetModel();
			ndModelArticulation::ndNode* const modelRootNode = ragdoll->AddRootBody(rootBody);
			ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)modelRootNode->m_body->GetAsBodyKinematic()->GetNotifyCallback();

			struct StackData
			{
				ndModelArticulation::ndNode* parentBone;
				ndSharedPtr<ndRenderSceneNode> childEntity;
			};
			ndFixSizeArray<StackData, 256> stack;

			for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = notify->GetUserData()->GetChildren().GetFirst(); node; node = node->GetNext())
			{
				StackData data;
				data.parentBone = modelRootNode;
				data.childEntity = node->GetInfo();
				stack.PushBack(data);
			}

			while (stack.GetCount())
			{
				StackData data(stack.Pop());

				const char* const name = data.childEntity->m_name.GetStr();
				//ndTrace(("name: %s\n", name));
				for (ndInt32 i = 0; ragdollDefinition[i].m_boneName[0]; ++i)
				{
					const ndDefinition& definition = ragdollDefinition[i];

					if (!strcmp(definition.m_boneName, name))
					{
						ndBodyDynamic* const parentBody = data.parentBone->m_body->GetAsBodyDynamic();
						ndSharedPtr<ndBody> childBody(CreateBodyPart(scene, entityDuplicate, loader, definition, parentBody));

						//connect this body part to its parentBody with a rag doll joint
						ndSharedPtr<ndJointBilateralConstraint> joint(ConnectBodyParts(childBody->GetAsBodyDynamic(), parentBody, definition));

						// add this child body to the rad doll model.
						data.parentBone = ragdoll->AddLimb(data.parentBone, childBody, joint);
						break;
					}
				}
			
				for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = data.childEntity->GetChildren().GetFirst(); node; node = node->GetNext())
				{
					StackData childData;
					childData.parentBone = data.parentBone;
					childData.childEntity = node->GetInfo();
					stack.PushBack(childData);
				}
			}

			CalculateMassDistribution(ragdoll, ndFloat32(100.0f));
		}
	};

	ndSharedPtr<ndModelNotify> CreateRagdoll(ndDemoEntityManager* const scene, const ndMeshLoader& loader, const ndMatrix& location)
	{
		// make a hierchical atriculate model
		ndSharedPtr<ndModel> model(new ndModelArticulation());
		
		// create a ragdoll controller 
		ndSharedPtr<ndModelNotify> controller(new ndRagDollController());
		model->SetNotifyCallback(controller);
		
		ndRagDollController* const ragdollController = (ndRagDollController*)*controller;
		ragdollController->RagdollBuildScript(scene, loader, location);
		
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		model->AddBodiesAndJointsToWorld();
		return controller;
	}
}

using namespace ndRagdoll;
void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	//ndSharedPtr<ndBody> bodyFloor(BuildPlayground(scene));
	ndSharedPtr<ndBody> bodyFloor(BuildCompoundScene(scene, ndGetIdentityMatrix()));
	//ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "marblecheckboard.png", 0.1f, true));
	
	
	ndMeshLoader loader;
	loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("ragdoll.fbx"));
	
	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndDemoEntityManager* const scene, ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
			m_posit = FindFloor(*scene->GetWorld(), m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f);
			m_posit.m_y += ndFloat32(10.0f);
		}
	};

	ndMatrix playerMatrix(PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
	ndSharedPtr<ndModelNotify> modelNotity(CreateRagdoll(scene, loader, playerMatrix));

	{
#if 1
		// add few more rag dolls
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 0.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, 0.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, 0.0f));

		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, 10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, 10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, 10.0f));

		CreateRagdoll(scene, loader, PlaceMatrix(scene, 0.0f, 0.0f, -10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 3.0f, 0.0f, -10.0f));
		CreateRagdoll(scene, loader, PlaceMatrix(scene, 5.0f, 0.0f, -10.0f));
#endif
	}

	ndFloat32 angle = ndFloat32(90.0f * ndDegreeToRad);
	playerMatrix = ndYawMatrix(angle) * playerMatrix;
	playerMatrix.m_posit += playerMatrix.m_front.Scale (-15.0f);
	playerMatrix.m_posit = FindFloor(*scene->GetWorld(), playerMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f);
	scene->SetCameraMatrix(playerMatrix, playerMatrix.m_posit);
}
