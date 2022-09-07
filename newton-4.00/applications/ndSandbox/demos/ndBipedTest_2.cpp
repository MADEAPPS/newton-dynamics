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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace biped2
{
	class ndDefinition
	{
		public:
		enum ndjointType
		{
			m_root,
			m_fix,
			m_hinge,
			m_spherical,
			m_doubleHinge,
			m_effector
		};

		struct ndJointLimit
		{
			ndFloat32 m_minTwistAngle;
			ndFloat32 m_maxTwistAngle;
			ndFloat32 m_coneAngle;
		};

		struct ndFrameMatrix
		{
			ndFloat32 m_pitch;
			ndFloat32 m_yaw;
			ndFloat32 m_roll;
		};

		char m_boneName[32];
		ndjointType m_type;
		ndInt32 m_selfCollide;
		ndJointLimit m_jointLimits;
		ndFrameMatrix m_frameBasics;
	};

	static ndDefinition ragdollDefinition[] =
	{
		{ "root", ndDefinition::m_root,{},{} },
#if 0
		//{ "lowerback", ndDefinition::m_spherical, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
		//{ "upperback", ndDefinition::m_spherical, { -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		//{ "lowerneck", ndDefinition::m_spherical, { -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		//{ "upperneck", ndDefinition::m_spherical, { -60.0f, 60.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		//{ "lclavicle", ndDefinition::m_spherical, { -60.0f, 60.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		//{ "lhumerus", ndDefinition::m_hinge, { 0.0f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		//{ "lradius", ndDefinition::m_doubleHinge, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		//{ "rclavicle", ndDefinition::m_spherical, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		//{ "rhumerus", ndDefinition::m_hinge, { 0.0f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		//{ "rradius", ndDefinition::m_doubleHinge, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

#else
		{ "lowerback", ndDefinition::m_fix, 1, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
		{ "upperback", ndDefinition::m_fix, 1, { -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "lowerneck", ndDefinition::m_fix, 1, { -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "upperneck", ndDefinition::m_fix, 1, { -60.0f, 60.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "lclavicle", ndDefinition::m_fix, 1, { -60.0f, 60.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "lhumerus", ndDefinition::m_fix, 1, { 0.0f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "lradius", ndDefinition::m_fix, 1, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		{ "rclavicle", ndDefinition::m_fix, 1, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "rhumerus", ndDefinition::m_fix, 1, { 0.0f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rradius", ndDefinition::m_fix, 1, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
#endif

		{ "rhipjoint", ndDefinition::m_spherical, 0,{ -60.0f, 60.0f, 80.0f },{ 0.0f, -60.0f, 0.0f } },
		{ "rfemur", ndDefinition::m_hinge, 1, { 0.5f, 120.0f, 0.0f },{ 0.0f, 90.0f, 0.0f } },
		{ "rfoof_effector", ndDefinition::m_effector, 1, { 0.0f, 0.0f, 60.0f },{ 0.0f, 0.0f, 90.0f } },
		{ "rtibia", ndDefinition::m_doubleHinge, 1, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

		{ "lhipjoint", ndDefinition::m_spherical, 0, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "lfemur", ndDefinition::m_hinge, 1, { 0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "lfoof_effector", ndDefinition::m_effector, 1, { 0.0f, 0.0f, 60.0f },{ 0.0f, 0.0f, 90.0f } },
		{ "ltibia", ndDefinition::m_doubleHinge, 1, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

		{ "", ndDefinition::m_root, 0, {},{} },
	};

	class ndBipedMaterial : public ndApplicationMaterial
	{
		public:
		ndBipedMaterial()
			:ndApplicationMaterial()
		{
		}

		ndBipedMaterial(const ndBipedMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const
		{
			return new ndBipedMaterial(*this);
		}

		bool OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const
		{
			// filter self collision when the contact is with in the same model
			const ndShapeMaterial& material0 = instanceShape0.GetMaterial();
			const ndShapeMaterial& material1 = instanceShape1.GetMaterial();

			ndUnsigned64 pointer0 = material0.m_userParam[ndContactCallback::m_modelPointer].m_intData;
			ndUnsigned64 pointer1 = material1.m_userParam[ndContactCallback::m_modelPointer].m_intData;
			if (pointer0 == pointer1)
			{
				// here we know the part are from the same model.
				// we can apply some more filtering by for now we just disable all self model collisions. 
				ndUnsigned64 selfCollide0 = material0.m_userParam[ndContactCallback::m_materialFlags].m_intData;
				ndUnsigned64 selfCollide1 = material1.m_userParam[ndContactCallback::m_materialFlags].m_intData;
				if (!(selfCollide0 || selfCollide1))
				{
					return false;
				}
			}
			return true;
		}
	};

	class ndDQNcontroller : public ndDeepBrain
	{
		public: 
		ndDQNcontroller(ndInt32 numberOfImputs, ndInt32 numberOfOutputs)
			:ndDeepBrain()
		{
			ndInt32 neuronsPerLayers = 32;

			ndDeepBrainLayer* const inputLayer = new ndDeepBrainLayer(numberOfImputs, neuronsPerLayers, m_tanh);
			ndDeepBrainLayer* const hiddenLayer0 = new ndDeepBrainLayer(inputLayer->GetOuputSize(), neuronsPerLayers, m_tanh);
			ndDeepBrainLayer* const hiddenLayer1 = new ndDeepBrainLayer(hiddenLayer0->GetOuputSize(), neuronsPerLayers, m_tanh);
			//ndDeepBrainLayer* const hiddenLayer2 = new ndDeepBrainLayer(hiddenLayer1->GetOuputSize(), neuronsPerLayers, m_tanh);
			ndDeepBrainLayer* const ouputLayer = new ndDeepBrainLayer(hiddenLayer1->GetOuputSize(), numberOfOutputs, m_sigmoid);

			BeginAddLayer();
			AddLayer(inputLayer);
			AddLayer(hiddenLayer0);
			AddLayer(hiddenLayer1);
			//brain.AddLayer(hiddenLayer2);
			AddLayer(ouputLayer);
			EndAddLayer();
		}
	};

	class ndHumanoidModel : public ndModel
	{
		public:
		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_basePosition(ndVector::m_wOne)
				,m_effector(nullptr)
				,m_swivel(0.0f)
				,m_x(0.0f)
				,m_y(0.0f)
				,m_z(0.0f)
			{
			}

			ndEffectorInfo(ndIkSwivelPositionEffector* const effector)
				:m_basePosition(effector->GetPosition())
				,m_effector(effector)
				,m_swivel(0.0f)
				,m_x(0.0f)
				,m_y(0.0f)
				,m_z(0.0f)
			{
			}

			ndVector m_basePosition;
			ndIkSwivelPositionEffector* m_effector;
			ndReal m_swivel;
			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
			ndParamMapper m_x_mapper;
			ndParamMapper m_y_mapper;
			ndParamMapper m_z_mapper;
			ndParamMapper m_swivel_mapper;
		};

		ndHumanoidModel(ndDemoEntityManager* const scene, ndDemoEntity* const model, const ndMatrix& location, ndDefinition* const definition)
			:ndModel()
			,m_controller(2, 3)
			,m_invDynamicsSolver()
			,m_effectors()
			,m_bodyArray()
			,m_effectorsJoints()
		{
			ndWorld* const world = scene->GetWorld();

			// make a clone of the mesh and add it to the scene
			ndDemoEntity* const entity = (ndDemoEntity*)model->CreateClone();
			scene->AddEntity(entity);

			// find the floor location 
			ndMatrix entMatrix(entity->CalculateGlobalMatrix() * location);
			ndVector floor(FindFloor(*world, entMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			entMatrix.m_posit.m_y = floor.m_y + 1.1f;
			entity->ResetMatrix(entMatrix);

			// add the root body
			ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(ragdollDefinition[0].m_boneName);
			ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, nullptr, ragdollDefinition[0]);

			ndInt32 stack = 0;
			ndFixSizeArray<ndFloat32, 64> massWeight;
			ndFixSizeArray<ndBodyDynamic*, 32> parentBones;
			ndFixSizeArray<ndDemoEntity*, 32> childEntities;

			parentBones.SetCount(32);
			childEntities.SetCount(32);

			parentBones.SetCount(32);
			childEntities.SetCount(32);

			for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling())
			{
				childEntities[stack] = child;
				parentBones[stack] = rootBody;
				stack++;
			}

			// walk model hierarchic adding all children designed as rigid body bones. 
			while (stack)
			{
				stack--;
				ndBodyDynamic* parentBody = parentBones[stack];
				ndDemoEntity* const childEntity = childEntities[stack];
				const char* const name = childEntity->GetName().GetStr();
				//ndTrace(("name: %s\n", name));
				for (ndInt32 i = 0; definition[i].m_boneName[0]; ++i)
				{
					if (!strcmp(definition[i].m_boneName, name))
					{
						if (definition[i].m_type != ndDefinition::m_effector)
						{
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBody, definition[i]);

							// connect this body part to its parentBody with a robot joint
							ndJointBilateralConstraint* const joint = ConnectBodyParts(childBody, parentBody, definition[i]);
							world->AddJoint(joint);
							parentBody = childBody;
						}
						else
						{
							ndDemoEntityNotify* const childNotify = (ndDemoEntityNotify*)parentBody->GetNotifyCallback();
							ndAssert(childNotify);
							ndDemoEntityNotify* const midNotify = (ndDemoEntityNotify*)childNotify->m_parentBody->GetNotifyCallback();
							ndAssert(midNotify);
							ndDemoEntityNotify* const pivotNotify = (ndDemoEntityNotify*)midNotify->m_parentBody->GetNotifyCallback();
							ndAssert(pivotNotify);

							ndBodyDynamic* const childBody = childNotify->GetBody()->GetAsBodyDynamic();
							ndBodyDynamic* const pivotBody = pivotNotify->GetBody()->GetAsBodyDynamic();

							ndDemoEntity* const pivotFrameNode = midNotify->m_entity->FindBySubString("reference");
							ndDemoEntity* const childFrameNode = childNotify->m_entity->FindBySubString("effector");
							ndAssert(pivotFrameNode);
							ndAssert(childFrameNode);
							ndAssert(childFrameNode == childEntity);

							ndMatrix pivotFrame(pivotFrameNode->CalculateGlobalMatrix());
							ndMatrix effectorFrame(childFrameNode->CalculateGlobalMatrix());

							ndMatrix swivelFrame(ndGetIdentityMatrix());
							swivelFrame.m_front = (effectorFrame.m_posit - pivotFrame.m_posit).Normalize();
							swivelFrame.m_up = pivotFrame.m_up;
							swivelFrame.m_right = (swivelFrame.m_front.CrossProduct(swivelFrame.m_up)).Normalize();
							swivelFrame.m_up = swivelFrame.m_right.CrossProduct(swivelFrame.m_front);

							ndFloat32 regularizer = 0.001f;
							ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame, pivotFrame, swivelFrame, childBody, pivotBody);
							effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
							effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);

							const ndVector kneePoint(childFrameNode->GetParent()->CalculateGlobalMatrix().m_posit);
							const ndVector dist0(effectorFrame.m_posit - kneePoint);
							const ndVector dist1(kneePoint - pivotFrame.m_posit);
							const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
							effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.999f);

							ndEffectorInfo info(effector);
							info.m_x_mapper = ndParamMapper(0.0f, workSpace * 0.999f);
							info.m_y_mapper = ndParamMapper(-80.0f * ndDegreeToRad, 80.0f * ndDegreeToRad);
							info.m_z_mapper = ndParamMapper(-80.0f * ndDegreeToRad, 80.0f * ndDegreeToRad);
							info.m_swivel_mapper = ndParamMapper(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);

							// set the default pose param.
							ndVector localPosit(effector->GetPosition());
							info.m_x = info.m_x_mapper.CalculateParam(ndSqrt(localPosit.DotProduct(localPosit & ndVector::m_triplexMask).GetScalar()));

							//ndVector localPositDir(localPosit.Normalize());
							//ndFloat32 yawAngle = ndAtan2(-localPositDir.m_z, localPositDir.m_x);;
							//info.m_y = info.m_y_mapper.CalculateParam(yawAngle);
							//ndFloat32 rollAngle = ndSin(localPositDir.m_y);
							//info.m_z = info.m_z_mapper.CalculateParam(rollAngle);

							m_effectors.PushBack(info);
							m_effectorsJoints.PushBack(info.m_effector);
						}
						break;
					}
				}

				for (ndDemoEntity* child = childEntity->GetChild(); child; child = child->GetSibling())
				{
					childEntities[stack] = child;
					parentBones[stack] = parentBody;
					stack++;
				}
			}

			NormalizeMassDistribution(100.0f);
		}

		~ndHumanoidModel()
		{
			for (ndInt32 i = 0; i < m_effectorsJoints.GetCount(); ++i)
			{
				delete m_effectorsJoints[i];
			}
		}

		void NormalizeMassDistribution(ndFloat32 mass) const
		{
			ndFloat32 maxVolume = -1.0e10f;
			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			{
				ndFloat32 volume = m_bodyArray[i]->GetCollisionShape().GetVolume();
				maxVolume = ndMax(maxVolume, volume);
			}

			ndFloat32 totalVolume = 0.0f;
			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			{
				ndFloat32 volume = m_bodyArray[i]->GetCollisionShape().GetVolume();
				if (volume < 0.01f * maxVolume)
				{
					volume = 0.01f * maxVolume;
				}
				totalVolume += volume;
			}

			ndFloat32 density = mass / totalVolume;

			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			{
				ndBodyDynamic* const body = m_bodyArray[i];
				ndFloat32 volume = body->GetCollisionShape().GetVolume();
				if (volume < 0.01f * maxVolume)
				{
					volume = 0.01f * maxVolume;
				}
				ndFloat32 normalMass = density * volume;
				body->SetMassMatrix(normalMass, body->GetCollisionShape());
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
			}
		}

		ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone, const ndDefinition& definition)
		{
			ndShapeInstance* const shape = entityPart->CreateCollisionFromChildren();
			ndAssert(shape);

			// create the rigid body that will make this body
			ndMatrix matrix(entityPart->CalculateGlobalMatrix());

			ndBodyDynamic* const body = new ndBodyDynamic();
			body->SetMatrix(matrix);
			body->SetCollisionShape(*shape);
			body->SetMassMatrix(1.0f, *shape);
			body->SetNotifyCallback(new ndBindingRagdollEntityNotify(scene, entityPart, parentBone, 100.0f));

			// save the shape material type
			ndShapeInstance& instanceShape = body->GetCollisionShape();
			instanceShape.m_shapeMaterial.m_userId = ndApplicationMaterial::m_modelPart;
			instanceShape.m_shapeMaterial.m_userParam[ndContactCallback::m_modelPointer].m_intData = ndUnsigned64(this);
			instanceShape.m_shapeMaterial.m_userParam[ndContactCallback::m_materialFlags].m_intData = definition.m_selfCollide;

			m_bodyArray.PushBack(body);
			scene->GetWorld()->AddBody(body);
			delete shape;
			return body;
		}

		ndJointBilateralConstraint* ConnectBodyParts(ndBodyDynamic* const childBody, ndBodyDynamic* const parentBone, const ndDefinition& definition)
		{
			ndMatrix matrix(childBody->GetMatrix());
			ndDefinition::ndFrameMatrix frameAngle(definition.m_frameBasics);
			ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

			switch (definition.m_type)
			{
				case ndDefinition::m_fix:
				{
					ndJointFix6dof* const joint = new ndJointFix6dof(pinAndPivotInGlobalSpace, childBody, parentBone);
					return joint;
				}

				case ndDefinition::m_spherical:
				{
					ndIkJointSpherical* const joint = new ndIkJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBone);
					//ndDefinition::ndJointLimit jointLimits(definition.m_jointLimits);
					//joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
					//joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
					return joint;
				}

				case ndDefinition::m_hinge:
				{
					ndIkJointHinge* const joint = new ndIkJointHinge(pinAndPivotInGlobalSpace, childBody, parentBone);

					ndDefinition::ndJointLimit jointLimits(definition.m_jointLimits);
					joint->SetLimitState(true);
					joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
					return joint;
				}

				case ndDefinition::m_doubleHinge:
				{
					ndJointDoubleHinge* const joint = new ndJointDoubleHinge(pinAndPivotInGlobalSpace, childBody, parentBone);

					ndDefinition::ndJointLimit jointLimits(definition.m_jointLimits);
					joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
					joint->SetLimits1(-45.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);

					joint->SetAsSpringDamper0(0.01f, 0.0f, 10.0f);
					joint->SetAsSpringDamper1(0.01f, 0.0f, 10.0f);
					return joint;
				}

				default:
					ndAssert(0);
			}
			return nullptr;
		}

		ndVector CalculateCenterOfMass() const
		{
			ndFloat32 toltalMass = 0.0f;
			ndVector com(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			{
				ndBodyDynamic* const body = m_bodyArray[i];
				ndFloat32 mass = body->GetMassMatrix().m_w;
				ndVector comMass(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
				com += comMass.Scale(mass);
				toltalMass += mass;
			}
			com = com.Scale(1.0f / toltalMass);
			com.m_w = 1.0f;
			return com;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndMatrix matrix(m_bodyArray[0]->GetMatrix());
			matrix.m_posit = CalculateCenterOfMass();
			context.DrawFrame(matrix);

			if (m_effectors.GetCount() >= 2)
			{
				const ndEffectorInfo& info0 = m_effectors[0];
				const ndEffectorInfo& info1 = m_effectors[1];
				//	ndJointBilateralConstraint* const joint = info.m_effector;
				//	joint->DebugJoint(context);

				//ndBodyKinematic* const body0 = info0.m_effector->GetBody0();
				//ndBodyKinematic* const body1 = info1.m_effector->GetBody0();
				ndVector p0(info0.m_effector->GetGlobalPosition());
				ndVector p1(info1.m_effector->GetGlobalPosition());
				context.DrawLine(p0, p1, ndVector::m_zero);

				ndVector q0(matrix.m_posit);
				ndVector q1(matrix.m_posit);
				q1.m_y -= 1.2f;

				//context.DrawPoint(matrix.m_posit, ndVector(1.0f, 1.0f, 0.0f, 1.0f), 8.0f);
				//context.DrawLine(q0, q1, ndVector(1.0f, 0.0f, 1.0f, 1.0f));

				ndBigVector p0Out;
				ndBigVector p1Out;

				dRayToRayDistance(q0, q1, p0, p1, p0Out, p1Out);
				context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 5);
				context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 5);


				//for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
				//{
				//	const ndEffectorInfo& info = m_effectors[i];
				//	ndJointBilateralConstraint* const joint = info.m_effector;
				//	joint->DebugJoint(context);
				//}
			}
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::Update(world, timestep);
			for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			{
				ndEffectorInfo& info = m_effectors[i];
				const ndMatrix yaw(ndYawMatrix(info.m_y_mapper.Interpolate(info.m_y)));
				const ndMatrix roll(ndRollMatrix(info.m_z_mapper.Interpolate(info.m_z)));

				ndVector posit(info.m_x_mapper.Interpolate(info.m_x), 0.0f, 0.0f, 1.0f);
				posit = roll.RotateVector(posit);
				posit = yaw.RotateVector(posit);

				info.m_effector->SetPosition(posit);
				info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel));
			}

			ndSkeletonContainer* const skeleton = m_bodyArray[0]->GetSkeleton();
			ndAssert(skeleton);

			//m_invDynamicsSolver.SetMaxIterations(4);
			if (m_effectorsJoints.GetCount() && !m_invDynamicsSolver.IsSleeping(skeleton))
			{
				m_invDynamicsSolver.SolverBegin(skeleton, &m_effectorsJoints[0], m_effectorsJoints.GetCount(), world, timestep);
				m_invDynamicsSolver.Solve();
				m_invDynamicsSolver.SolverEnd();
			}
		}

		void ApplyControls(ndDemoEntityManager* const scene)
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Control panel");

			ndEffectorInfo& info = m_effectors[0];

			bool change = false;
			ImGui::Text("distance");
			change = change | ImGui::SliderFloat("##x", &info.m_x, -0.5f, 1.0f);
			ImGui::Text("roll");
			change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);
			ImGui::Text("yaw");
			change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);

			ImGui::Text("swivel");
			change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);

			ndEffectorInfo& info1 = m_effectors[1];
			info1.m_x = info.m_x;
			info1.m_y = info.m_y;
			info1.m_z = info.m_z;
			info1.m_swivel = info.m_swivel;

			static ndOUNoise xxxxxxx0(0.0f, 0.5f, 0.0f, 0.1f);
			static ndOUNoise xxxxxxx1(0.0f, 0.5f, 0.0f, 0.3f);
			//info.m_z = xxxxxxx0.Evaluate(1.0f / 500.0f);
			//info1.m_z = xxxxxxx1.Evaluate(1.0f / 500.0f);

			if (change)
			{
				m_bodyArray[0]->SetSleepState(false);
			}
		}

		static void ControlPanel(ndDemoEntityManager* const scene, void* const context)
		{
			ndHumanoidModel* const me = (ndHumanoidModel*)context;
			me->ApplyControls(scene);
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostUpdate(world, timestep);
		}

		void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostTransformUpdate(world, timestep);
		}

		ndDQNcontroller m_controller;
		ndIkSolver m_invDynamicsSolver;
		ndFixSizeArray<ndEffectorInfo, 8> m_effectors;
		ndFixSizeArray<ndBodyDynamic*, 32> m_bodyArray;
		ndFixSizeArray<ndJointBilateralConstraint*, 8> m_effectorsJoints;
	};

	class ndHumanoidTraningModel : public ndHumanoidModel
	{
		enum ndTraningStage
		{
			m_init,
			m_startEpock,
		};

		public: 
		ndHumanoidTraningModel(ndDemoEntityManager* const scene, ndDemoEntity* const model, const ndMatrix& location, ndDefinition* const definition)
			:ndHumanoidModel(scene, model, location, definition)
			,m_onlineController(m_controller)
			,m_stage(m_init)
		{
		}

		static void TrainingLoop(ndDemoEntityManager* const scene, void* const context)
		{
			ndHumanoidTraningModel* const me = (ndHumanoidTraningModel*)context;
			me->TrainingLoop(scene);
		}

		void TrainingLoop(ndDemoEntityManager* const scene)
		{
			ndWorld* const world = scene->GetWorld();

			switch (m_stage)
			{
				case m_init:
				{
					world->Sync();
					InitTraning();
					break;
				}

				case m_startEpock:
				{
					world->Sync();
					StartEpock();
					break;
				}

				default:;
			}
		}

		void InitTraning()
		{
			m_stage = m_startEpock;
		}

		void StartEpock()
		{

		}

		ndDQNcontroller m_onlineController;
		ndTraningStage m_stage;
	};
};

using namespace biped2;
void ndBipedTest_2Trainer(ndDemoEntityManager* const scene)
{
	// build a floor
	ndSetRandSeed(12345);

	BuildFloorBox(scene, ndGetIdentityMatrix());

	ndBipedMaterial material;
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.9f;
	material.m_staticFriction1 = 0.9f;
	material.m_dynamicFriction0 = 0.9f;
	material.m_dynamicFriction1 = 0.9f;

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_modelPart);

	ndMatrix origin(ndGetIdentityMatrix());
	origin.m_posit.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	origin.m_posit.m_x -= 20.0f;
	ndDemoEntity* const modelMesh = scene->LoadFbxMesh("walker.fbx");

	ndWorld* const world = scene->GetWorld();
	ndHumanoidTraningModel* const model = new ndHumanoidTraningModel(scene, modelMesh, origin, ragdollDefinition);
	world->AddModel(model);
	scene->Set2DDisplayRenderFunction(ndHumanoidTraningModel::TrainingLoop, nullptr, model);

	//world->AddJoint(new ndJointFix6dof(model->m_bodyArray[0]->GetMatrix(), model->m_bodyArray[0], world->GetSentinelBody()));

	delete modelMesh;

	ndQuaternion rot;
	origin.m_posit.m_x -= 5.0f;
	origin.m_posit.m_y = 2.0f;
	scene->SetCameraMatrix(rot, origin.m_posit);
}

void ndBipedTest_2(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

	ndBipedMaterial material;
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.9f;
	material.m_staticFriction1 = 0.9f;
	material.m_dynamicFriction0 = 0.9f;
	material.m_dynamicFriction1 = 0.9f;

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_modelPart);

	ndMatrix origin(ndGetIdentityMatrix());
	origin.m_posit.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	origin.m_posit.m_x -= 20.0f;
	ndDemoEntity* const modelMesh = scene->LoadFbxMesh("walker.fbx");

	ndWorld* const world = scene->GetWorld();
	ndHumanoidModel* const model = new ndHumanoidModel(scene, modelMesh, origin, ragdollDefinition);
	world->AddModel(model);
	scene->Set2DDisplayRenderFunction(ndHumanoidModel::ControlPanel, nullptr, model);

	//world->AddJoint(new ndJointFix6dof(model->m_bodyArray[0]->GetMatrix(), model->m_bodyArray[0], world->GetSentinelBody()));

	delete modelMesh;

	ndQuaternion rot;
	origin.m_posit.m_x -= 5.0f;
	origin.m_posit.m_y = 2.0f;
	scene->SetCameraMatrix(rot, origin.m_posit);
}