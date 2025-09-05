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

			ndUnsigned64 pointer0 = material0.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
			ndUnsigned64 pointer1 = material1.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
			if (pointer0 == pointer1)
			{
				// here we know the part are from the same model.
				// we can apply some more filtering by for now we just disable all self model collisions. 
				ndUnsigned64 selfCollide0 = material0.m_userParam[ndDemoContactCallback::m_materialFlags].m_intData;
				ndUnsigned64 selfCollide1 = material1.m_userParam[ndDemoContactCallback::m_materialFlags].m_intData;
				if (!(selfCollide0 || selfCollide1))
				{
					return false;
				}
			}
			return true;
		}
	};

	class ndModelPhysicState
	{
		public:
		ndMatrix m_comFrame;
		ndVector m_comTarget;
		ndVector m_zmpPosit;
		ndVector m_centerOfMassVeloc;
		ndFloat32 m_comSagitalAngle;
		ndFloat32 m_comSagitalOmega;
	};

	class ndHumanoidBrain: public ndBrain
	{
		public: 
		enum ndModelStateParam
		{
			m_comSagittalSpeed,
			m_comSagittalPosit,
			m_observationsSize,
		};

		enum ndModelActionParam
		{
			m_goBack,
			m_stay,
			m_goFoward,
			m_actionSize,
		};

		//ndHumanoidBrain(ndInt32 numberOfImputs, ndInt32 numberOfOutputs)
		ndHumanoidBrain(ndInt32, ndInt32)
			:ndBrain()
		{
			ndAssert(0);
			//const ndInt32 neuronsPerHiddenLayers = 16;
			//ndBrainLayerLinearActivated* const inputLayer = new ndBrainLayerLinearActivated(numberOfImputs, neuronsPerHiddenLayers, m_relu);
			//ndBrainLayerLinearActivated* const hiddenLayer0 = new ndBrainLayerLinearActivated(inputLayer->GetOutputSize(), neuronsPerHiddenLayers, m_relu);
			//ndBrainLayerLinearActivated* const hiddenLayer1 = new ndBrainLayerLinearActivated(hiddenLayer0->GetOutputSize(), neuronsPerHiddenLayers, m_relu);
			////ndBrainLayer* const hiddenLayer2 = new ndBrainLayer(hiddenLayer1->GetOutputSize(), neuronsPerHiddenLayers, m_relu);
			//ndBrainLayerLinearActivated* const outputLayer = new ndBrainLayerLinearActivated(hiddenLayer1->GetOutputSize(), numberOfOutputs, m_relu);
			//
			//AddLayer(inputLayer);
			//AddLayer(hiddenLayer0);
			//AddLayer(hiddenLayer1);
			////brain.AddLayer(hiddenLayer2);
			//AddLayer(outputLayer);

			ndAssert(0);
			//InitGaussianWeights(ndReal(0.125f));
		}
	};

#if 0
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
				:m_basePosition(effector->GetLocalTargetPosition())
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
			,m_locaFrame(ndGetIdentityMatrix())
			,m_brain(2, 3)
			,m_controller(&m_brain)
			,m_invDynamicsSolver()
			,m_effectors()
			,m_bodyArray()
			,m_effectorsJoints()
		{
			ndWorld* const world = scene->GetWorld();
			
			// make a clone of the mesh and add it to the scene
			ndDemoEntity* const entity = model->CreateClone();
			scene->AddEntity(entity);
			
			// find the floor location 
			ndMatrix entMatrix(entity->CalculateGlobalMatrix() * location);
			ndVector floor(FindFloor(*world, entMatrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			entMatrix.m_posit.m_y = floor.m_y + 1.1f;
			entity->ResetMatrix(entMatrix);
			
			// add the root body
			ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(ragdollDefinition[0].m_boneName);
			ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, nullptr, ragdollDefinition[0]);
			
			ndDemoEntity* const localFrame = rootEntity->Find("modelLocalFrame");
			ndAssert(localFrame);
			m_locaFrame = localFrame->GetRenderMatrix();
			
			ndInt32 stack = 0;
			ndFixSizeArray<ndFloat32, 64> massWeight;
			ndFixSizeArray<ndBodyDynamic*, 32> parentBones;
			ndFixSizeArray<ndDemoEntity*, 32> childEntities;
			
			parentBones.SetCount(32);
			childEntities.SetCount(32);
			
			parentBones.SetCount(32);
			childEntities.SetCount(32);
			
			for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
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
							ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
							world->AddJoint(jointPtr);
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
			
							ndFloat32 regularizer = 0.001f;
							ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame.m_posit, pivotFrame, childBody, pivotBody);
							const ndVector kneePoint(childFrameNode->GetParent()->CalculateGlobalMatrix().m_posit);
							const ndVector dist0(effectorFrame.m_posit - kneePoint);
							const ndVector dist1(kneePoint - pivotFrame.m_posit);
							const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
							effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.995f);
							
							effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
							effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
							
							ndEffectorInfo info(effector);
							info.m_x_mapper = ndParamMapper(0.0f, workSpace * 0.999f);
							info.m_y_mapper = ndParamMapper(-80.0f * ndDegreeToRad, 80.0f * ndDegreeToRad);
							info.m_z_mapper = ndParamMapper(-120.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
							info.m_swivel_mapper = ndParamMapper(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
							
							// set the default pose param.
							ndVector localPosit(effector->GetLocalTargetPosition());
							info.m_x = ndReal (info.m_x_mapper.CalculateParam(ndSqrt(localPosit.DotProduct(localPosit & ndVector::m_triplexMask).GetScalar())));
							
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
			
				for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
				{
					childEntities[stack] = child;
					parentBones[stack] = parentBody;
					stack++;
				}
			}
			
			NormalizeMassDistribution(100.0f);
			
			m_actionToActiationMap[ndHumanoidBrain::m_goBack] = -8.0f;
			m_actionToActiationMap[ndHumanoidBrain::m_stay] = 0.0f;
			m_actionToActiationMap[ndHumanoidBrain::m_goFoward] = 8.0f;
		}

		~ndHumanoidModel()
		{
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
			ndSharedPtr<ndShapeInstance> shapePtr(entityPart->CreateCollisionFromChildren());
			ndShapeInstance* const shape = *shapePtr;
			ndAssert(shape);

			// create the rigid body that will make this body
			ndMatrix matrix(entityPart->CalculateGlobalMatrix());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetMatrix(matrix);
			body->SetCollisionShape(*shape);
			body->SetMassMatrix(1.0f, *shape);
			body->SetNotifyCallback(new ndBindingRagdollEntityNotify(scene, entityPart, parentBone, 100.0f));

			// save the shape material type
			ndShapeInstance& instanceShape = body->GetCollisionShape();
			instanceShape.m_shapeMaterial.m_userId = ndApplicationMaterial::m_modelPart;
			instanceShape.m_shapeMaterial.m_userParam[ndContactCallback::m_modelPointer].m_intData = ndUnsigned64(this);
			instanceShape.m_shapeMaterial.m_userParam[ndContactCallback::m_materialFlags].m_intData = ndUnsigned64(definition.m_selfCollide);

			m_bodyArray.PushBack(body->GetAsBodyDynamic());
			ndSharedPtr<ndBody> bodyPtr(body);
			scene->GetWorld()->AddBody(bodyPtr);

			return body->GetAsBodyDynamic();
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

		ndModelPhysicState CalculateModelState() const
		{
			ndModelPhysicState modelState;

			ndFloat32 toltalMass = 0.0f;
			ndVector com(ndVector::m_zero);
			ndVector comVeloc(ndVector::m_zero);
			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			{
				ndBodyDynamic* const body = m_bodyArray[i];
				ndFloat32 mass = body->GetMassMatrix().m_w;
				ndVector comMass(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
				com += comMass.Scale(mass);
				comVeloc += body->GetVelocity().Scale(mass);
				toltalMass += mass;
			}

			ndFloat32 invScaleMass = 1.0f / toltalMass;
			com = com.Scale(invScaleMass);
			comVeloc = comVeloc.Scale(invScaleMass);
			com.m_w = 1.0f;

			ndVector zmp(m_bodyArray[0]->GetPosition());
			if (m_effectors.GetCount() >= 2)
			{
				const ndEffectorInfo& info0 = m_effectors[0];
				const ndEffectorInfo& info1 = m_effectors[1];
				ndVector p0(info0.m_effector->GetGlobalPosition());
				ndVector p1(info1.m_effector->GetGlobalPosition());
				
				ndVector q0(com);
				ndVector q1(com);
				q1.m_y -= 1.2f;
				
				ndBigVector qOq1ut;
				ndBigVector p0p1Out;
				ndRayToRayDistance(q0, q1, p0, p1, qOq1ut, p0p1Out);
				zmp = p0p1Out;
			}

			zmp.m_w = 1.0f;
			const ndMatrix matrix(m_locaFrame * m_bodyArray[0]->GetMatrix());

			modelState.m_comFrame.m_right = matrix.m_right;
			modelState.m_comFrame.m_front = ndVector(0.0f, 1.0f, 0.0f, 0.0f).CrossProduct(matrix.m_right);
			modelState.m_comFrame.m_front = modelState.m_comFrame.m_front.Normalize();
			modelState.m_comFrame.m_up = modelState.m_comFrame.m_right.CrossProduct(modelState.m_comFrame.m_front);
			modelState.m_comFrame.m_posit = com;
		
			modelState.m_zmpPosit = zmp;
			modelState.m_centerOfMassVeloc = comVeloc;
			
			ndVector segment(zmp - com);
			ndFloat32 length = ndSqrt(segment.DotProduct(segment & ndVector::m_triplexMask).GetScalar());
			ndVector targetPoint(zmp);
			targetPoint.m_y += length;
			modelState.m_comTarget = targetPoint;

			return modelState;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndModelPhysicState modeState(CalculateModelState());

			context.DrawFrame(modeState.m_comFrame);
			context.DrawLine(modeState.m_zmpPosit, modeState.m_comFrame.m_posit, ndVector(1.0f, 0.0f, 1.0f, 1.0f));
			context.DrawLine(modeState.m_zmpPosit, modeState.m_comTarget, ndVector(0.0f, 1.0f, 1.0f, 1.0f));
			
			context.DrawPoint(modeState.m_comTarget, ndVector(0.0f, 1.0f, 1.0f, 1.0f), 5);
			context.DrawPoint(modeState.m_zmpPosit, ndVector(1.0f, 1.0f, 0.0f, 1.0f), 5);
			context.DrawPoint(modeState.m_comFrame.m_posit, ndVector(1.0f, 0.0f, 1.0f, 1.0f), 5);

			for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			{
				const ndEffectorInfo& info = m_effectors[i];
				ndJointBilateralConstraint* const joint = info.m_effector;
				joint->DebugJoint(context);
			}

		}

		void Update(ndFloat32 timestep)
		{
			ndModel::Update(timestep);
			for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			{
				ndEffectorInfo& info = m_effectors[i];
				const ndMatrix yaw(ndYawMatrix(info.m_y_mapper.Interpolate(info.m_y)));
				const ndMatrix roll(ndRollMatrix(info.m_z_mapper.Interpolate(info.m_z)));

				ndVector posit(info.m_x_mapper.Interpolate(info.m_x), 0.0f, 0.0f, 1.0f);
				posit = roll.RotateVector(posit);
				posit = yaw.RotateVector(posit);

				info.m_effector->SetLocalTargetPosition(posit);
				info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel));
			}

			ndSkeletonContainer* const skeleton = m_bodyArray[0]->GetSkeleton();
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

		//void ApplyControls(ndDemoEntityManager* const scene)
		//{
		//	ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		//	scene->Print(color, "Control panel");
		//
		//	ndEffectorInfo& info = m_effectors[0];
		//
		//	bool change = false;
		//	ImGui::Text("distance");
		//	change = change | ImGui::SliderFloat("##x", &info.m_x, -0.5f, 1.0f);
		//	ImGui::Text("roll");
		//	change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);
		//	ImGui::Text("yaw");
		//	change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
		//
		//	ImGui::Text("swivel");
		//	change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);
		//
		//	ndEffectorInfo& info1 = m_effectors[1];
		//	info1.m_x = info.m_x;
		//	info1.m_y = info.m_y;
		//	info1.m_z = info.m_z;
		//	info1.m_swivel = info.m_swivel;
		//
		//	static ndOUNoise xxxxxxx0(0.0f, 0.5f, 0.0f, 0.1f);
		//	static ndOUNoise xxxxxxx1(0.0f, 0.5f, 0.0f, 0.3f);
		//	//info.m_z = xxxxxxx0.Evaluate(1.0f / 500.0f);
		//	//info1.m_z = xxxxxxx1.Evaluate(1.0f / 500.0f);
		//
		//	if (change)
		//	{
		//		m_bodyArray[0]->SetSleepState(false);
		//	}
		//}
		//
		//static void ControlPanel(ndDemoEntityManager* const scene, void* const context)
		//{
		//	ndHumanoidModel* const me = (ndHumanoidModel*)context;
		//	me->ApplyControls(scene);
		//}

		void PostUpdate(ndFloat32 timestep)
		{
			ndModel::PostUpdate(timestep);
		}

		void PostTransformUpdate(ndFloat32 timestep)
		{
			ndModel::PostTransformUpdate(timestep);
		}

		ndMatrix m_locaFrame;
		ndHumanoidBrain m_brain;
		ndBrainInstance m_controller;
		ndIkSolver m_invDynamicsSolver;
		ndFixSizeArray<ndEffectorInfo, 8> m_effectors;
		ndFixSizeArray<ndBodyDynamic*, 32> m_bodyArray;
		ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;
		ndFloat32 m_actionToActiationMap[ndHumanoidBrain::m_actionSize];
	};

	class ndHumanoidTraningModel : public ndHumanoidModel
	{
		enum ndTrainingStage
		{
			m_initTraining,
			m_tickTrainingEpoch,
			m_endTraining,
		};

		class ndBasePose
		{
			public:
			ndBasePose()
			{
			}

			ndBasePose(ndBodyDynamic* const body)
				:m_veloc(body->GetVelocity())
				,m_omega(body->GetOmega())
				,m_posit(body->GetPosition())
				,m_rotation(body->GetRotation())
				,m_body(body)
			{
			}

			void SetPose()
			{
				ndMatrix matrix(m_rotation, m_posit);
				m_body->SetMatrix(matrix);
				m_body->SetOmega(m_omega);
				m_body->SetVelocity(m_veloc);
			}

			ndVector m_veloc;
			ndVector m_omega;
			ndVector m_posit;
			ndQuaternion m_rotation;
			ndBodyDynamic* m_body;
		};

		class ndDeepBrainAgentTrainier : public ndBrainAgentDQN
		{
			public:
			ndDeepBrainAgentTrainier(ndBrain* const agent)
				:ndBrainAgentDQN(agent, 10000, 256)
			{
				m_transition.m_state.SetCount(agent->GetInputSize());
				m_transition.m_action.SetCount(agent->GetOutputSize());
				m_transition.m_nextState.SetCount(agent->GetInputSize());
			}

			void GetTransition(ndBrainReiforcementTransition& transition) const
			{
				transition.CopyFrom(m_transition);
			}

			ndBrainReiforcementTransition m_transition;
		};

		public: 
		ndHumanoidTraningModel(ndDemoEntityManager* const scene, ndDemoEntity* const model, const ndMatrix& location, ndDefinition* const definition)
			:ndHumanoidModel(scene, model, location, definition)
			,m_dqnAgent(&m_brain)
			,m_basePose()
			,m_traingCounter(0)
			,m_epochCounter(0)
			,m_trainingState(m_initTraining)
		{
			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); i++)
			{
				m_basePose.PushBack(ndBasePose(m_bodyArray[i]));
			}
		}

		void Update(ndFloat32 timestep)
		{
			ndModel::Update(timestep);
			TrainingLoopBegin(world, timestep);

			GetStateAndAction(timestep);
			ndInt32 valueIndex = m_dqnAgent.m_transition.m_action.GetMaxIndex();
			ndFloat32 effectorAction = m_actionToActiationMap[valueIndex];
			m_rollAngle += effectorAction * timestep;

			for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			{
				ndEffectorInfo& info = m_effectors[i];
				//const ndMatrix yaw(ndYawMatrix(info.m_y_mapper.Interpolate(info.m_y)));
				const ndMatrix yaw(ndYawMatrix(0.0f));
				//const ndMatrix roll(ndRollMatrix(info.m_z_mapper.Interpolate(info.m_z)));
				const ndMatrix roll(ndRollMatrix(m_rollAngle));
			
				ndVector posit(info.m_x_mapper.Interpolate(info.m_x), 0.0f, 0.0f, 1.0f);
				posit = yaw.RotateVector(posit);
				posit = roll.RotateVector(posit);

				info.m_effector->SetLocalTargetPosition(posit);
				info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel));
			}
			
			ndSkeletonContainer* const skeleton = m_bodyArray[0]->GetSkeleton();
			ndAssert(skeleton);

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

		void PostUpdate(ndFloat32 timestep)
		{
			ndModel::PostUpdate(timestep);
			TrainingLoopEnd(world, timestep);
		}
		
		void TrainingLoopBegin(ndFloat32)
		{
			switch (m_trainingState)
			{
				case m_initTraining:
				{
					InitTraning();
					break;
				}

				case m_tickTrainingEpoch:
				{
					break;
				}

				case m_endTraining:
				default:;
					ndAssert(0);
			}
		}

		void TrainingLoopEnd(ndFloat32 timestep)
		{
			if (m_trainingState == m_tickTrainingEpoch)
			{
				TickEpoch(world, timestep);
			}
		}

		void InitTraning()
		{
			//ndTrace(("frame %d\n", world->GetSubFrameNumber()));
 			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}

			m_rollAngle = 0;
			m_traingCounter++;
			m_epochCounter = 0;
			m_dqnAgent.m_exploration = ndMax(ndFloat32(0.01f), ndFloat32(m_dqnAgent.m_exploration - 0.01f));

			//m_trainingState = (m_traingCounter < 200) ? m_tickTrainingEpoch : m_endTraining;
			m_trainingState = (m_traingCounter < 1000) ? m_tickTrainingEpoch : m_endTraining;
		}

		void GetRandomAction()
		{
			ndBrainVector& action = m_dqnAgent.m_transition.m_action;
			ndAssert(action.GetCount() == ndHumanoidBrain::ndModelActionParam::m_actionSize);
			for (ndInt32 i = 0; i < ndHumanoidBrain::ndModelActionParam::m_actionSize; i++)
			{
				action[i] = ndReal (ndRand());
			}

			//ndInt32 valueIndex = ndInt32(ndFloat32(ndRand() * ndHumanoidBrain::ndModelActionParam::m_actionSize));
			//action[action GetMaxIndex() valueIndex] = 1.0f;
		}

		//void PredictAction(ndFloat32 timestep)
		void PredictAction(ndFloat32)
		{
			#if 0
			ndArray<ndReal>& action = m_dqnAgent.m_transition.m_action;
			//action.SetCount(ndHumanoidBrain::ndModelActionParam::m_actionSize);
			for (ndInt32 i = 0; i < ndHumanoidBrain::ndModelActionParam::m_actionSize; i++)
			{
				action[i] = 0.0f;
			}
			
			ndFloat32 stateSpeed = m_dqnAgent.m_transition.m_state[ndHumanoidBrain::m_comSagittalSpeed];
			ndFloat32 statePosit = m_dqnAgent.m_transition.m_state[ndHumanoidBrain::m_comSagittalPosit];
			ndFloat32 predictePosit = statePosit + stateSpeed * timestep;
			ndInt32 valueIndex = (predictePosit > 0.0) ? ndHumanoidBrain::ndModelActionParam::m_goBack : ndHumanoidBrain::ndModelActionParam::m_goFoward;
			if (ndAbs(predictePosit) < 0.0025f)
			{
				valueIndex = ndHumanoidBrain::ndModelActionParam::m_stay;
			}
			action[valueIndex] = 1.0f;
			#else
			m_dqnAgent.PredictAccion(m_dqnAgent.m_transition);
			#endif
		}

		void GetState(ndBrainVector& state)
		{
			ndModelPhysicState modelState(CalculateModelState());

			const ndVector sagittalDir(modelState.m_comFrame.m_front);
			const ndVector sagittalDist(modelState.m_comTarget - modelState.m_comFrame.m_posit);

			ndFloat32 sagittalComPosit = sagittalDir.DotProduct(sagittalDist).GetScalar();
			ndFloat32 sagittalComSpeed = sagittalDir.DotProduct(modelState.m_centerOfMassVeloc).GetScalar();

			//state.SetCount(ndHumanoidBrain::ndModelStateParam::m_observationsSize);
			ndAssert(state.GetCount() == ndHumanoidBrain::ndModelStateParam::m_observationsSize);
			state[ndHumanoidBrain::ndModelStateParam::m_comSagittalPosit] = ndReal(sagittalComPosit);
			state[ndHumanoidBrain::ndModelStateParam::m_comSagittalSpeed] = ndReal(sagittalComSpeed);
		}

		void GetStateAndAction(ndFloat32 timestep)
		{
			GetState(m_dqnAgent.m_transition.m_observation);
			ndFloat32 exploit = ndRand();
			if (m_dqnAgent.m_exploration > exploit)
			{
				GetRandomAction();
			}
			else
			{
				PredictAction(timestep);
			}

			// Get the reward of this state
			m_dqnAgent.m_transition.m_reward = 1.0f;
			m_dqnAgent.m_transition.m_terminalState = false;

			ndFloat32 dist = m_dqnAgent.m_transition.m_observation[ndHumanoidBrain::ndModelStateParam::m_comSagittalPosit];
			if (ndAbs(dist) > 0.4f)
			{
				m_dqnAgent.m_transition.m_terminalState = true;
			}
			if (ndAbs(m_rollAngle) > ndFloat32(80.0f * ndDegreeToRad))
			{
				m_dqnAgent.m_transition.m_terminalState = true;
			}
		}

		void TickEpoch(ndWorld* const, ndFloat32)
		{
			GetState(m_dqnAgent.m_transition.m_nextState);

			m_dqnAgent.LearnStep();

			m_epochCounter ++;
			bool isTerminal = m_dqnAgent.m_transition.m_terminalState;
			//if (isTerminal)
			if (isTerminal)
			{
				ndTrace (("failed epoch: frames alived %d\n", m_epochCounter));
				m_trainingState = m_initTraining;
			}

			if (m_epochCounter >= 600)
			{
				ndTrace(("succeded epoch frames alived %d\n", m_epochCounter));
				m_trainingState = m_initTraining;
				ndAssert(0);
			}
		}
		
		ndDeepBrainAgentTrainier m_dqnAgent;
		ndFixSizeArray<ndBasePose, 32> m_basePose;
		ndFloat32 m_rollAngle;
		ndInt32 m_traingCounter;
		ndInt32 m_epochCounter;
		ndTrainingStage m_trainingState;
	};

	class ndModelUI : public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, ndHumanoidModel* const biped)
			:ndUIEntity(scene)
			, m_biped(biped)
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

			ndHumanoidModel::ndEffectorInfo& info = m_biped->m_effectors[0];

			bool change = false;
			ImGui::Text("distance");
			change = change || ImGui::SliderFloat("##x", &info.m_x, -0.5f, 1.0f);
			ImGui::Text("roll");
			change = change || ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);
			ImGui::Text("yaw");
			change = change || ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);

			ImGui::Text("swivel");
			change = change || ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);

			ndHumanoidModel::ndEffectorInfo& info1 = m_biped->m_effectors[1];
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
				m_biped->m_bodyArray[0]->SetSleepState(false);
			}
		}

		ndHumanoidModel* m_biped;
	};
#endif
}

using namespace biped2;

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
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);
	
	ndMatrix origin(ndGetIdentityMatrix());
	origin.m_posit.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);
	
	origin.m_posit.m_x -= 20.0f;
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh = loader.LoadEntity("walker.fbx", scene);
	
	ndAssert(0);
	//ndWorld* const world = scene->GetWorld();
	//ndHumanoidModel* const model = new ndHumanoidModel(scene, *modelMesh, origin, ragdollDefinition);
	//ndSharedPtr<ndModel> modelPtr(model);
	//ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->m_bodyArray[0]->GetMatrix(), model->m_bodyArray[0], world->GetSentinelBody()));
	//world->AddModel(modelPtr);
	////world->AddJoint(fixJoint);
	//
	//ndModelUI* const bipedUI = new ndModelUI(scene, model);
	//ndSharedPtr<ndUIEntity> bipedUIPtr(bipedUI);
	//scene->Set2DDisplayRenderFunction(bipedUIPtr);

	ndQuaternion rot;
	origin.m_posit.m_x -= 5.0f;
	origin.m_posit.m_y = 2.0f;
	scene->SetCameraMatrix(rot, origin.m_posit);
}

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
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);
	
	ndMatrix origin(ndGetIdentityMatrix());
	origin.m_posit.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);
	
	origin.m_posit.m_x -= 20.0f;
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("walker.fbx", scene));
	
	ndAssert(0);
	//ndWorld* const world = scene->GetWorld();
	//ndHumanoidTraningModel* const model = new ndHumanoidTraningModel(scene, *modelMesh, origin, ragdollDefinition);
	//ndSharedPtr<ndModel> modelPtr(model);
	//ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->m_bodyArray[0]->GetMatrix(), model->m_bodyArray[0], world->GetSentinelBody()));
	//world->AddModel(modelPtr);
	////world->AddJoint(fixJoint);
	//
	////scene->Set2DDisplayRenderFunction(ndHumanoidTraningModel::TrainingLoop, nullptr, model);
	////scene->Set2DDisplayRenderFunction(ndHumanoidModel::ControlPanel, nullptr, model);
	//ndModelUI* const bipedUI = new ndModelUI(scene, model);
	//ndSharedPtr<ndUIEntity> bipedUIPtr(bipedUI);
	//scene->Set2DDisplayRenderFunction(bipedUIPtr);
	
	ndQuaternion rot(ndYawMatrix(ndPi * 0.5f));
	//origin.m_posit.m_x -= 5.0f;
	origin.m_posit.m_z = 4.0f;
	origin.m_posit.m_y = 1.5f;
	scene->SetCameraMatrix(rot, origin.m_posit);
}

