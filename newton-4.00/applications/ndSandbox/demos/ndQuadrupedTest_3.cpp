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
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndAnimationPose.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequenceBase.h"
#include "ndAnimationSequencePlayer.h"

namespace ndQuadruped_3
{
	class ndDefinition
	{
		public:
		enum ndJointType
		{
			m_root,
			m_hinge,
			m_effector
		};

		char m_boneName[32];
		ndJointType m_type;
		ndFloat32 m_minAngle;
		ndFloat32 m_maxAngle;
		ndFloat32 m_walkPhase;
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "spot_body", ndDefinition::m_root, 0.0f, 0.0f, 0.0f},

		{ "spot_shoulder_FR", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_FR", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_FR", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_FR_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },

		{ "spot_shoulder_FL", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_FL", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_FL", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_FL_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_BR", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_BR", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_BR", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_BR_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_BL", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_BL", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_BL", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_BL_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
	};

	class ndQuadrupedMaterial: public ndApplicationMaterial
	{
		public:
		ndQuadrupedMaterial()
			:ndApplicationMaterial()
		{
		}

		ndQuadrupedMaterial(const ndQuadrupedMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const
		{
			return new ndQuadrupedMaterial(*this);
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
				return false;
			}
			return true;
		}
	};

	class ndWalkSequence : public ndAnimationSequenceBase
	{
		public:
		class ndSegment
		{
			public:
			ndSegment()
				:m_a(ndVector::m_zero)
				,m_b(ndVector::m_zero)
				,m_t0(0.0f)
				,m_perimeter(0.0f)
			{
			}

			void Init(const ndVector& p0, const ndVector& p1, ndFloat32 t0, ndFloat32 t1)
			{
				ndFloat32 den = t1 - t0;
				m_a.m_x = (p1.m_x - p0.m_x) / den;
				m_b.m_x = (p0.m_x * t1 - p1.m_x * t0) / den;

				m_b.m_y = 0.0f;
				m_a.m_y = p1.m_y;

				m_t0 = t0;
				m_perimeter = ndPi / den;
			}

			ndVector Interpolate(ndFloat32 t) const
			{
				ndVector point(m_b);
				point.m_x += m_a.m_x * t;
				point.m_y += m_a.m_y * ndSin(m_perimeter * (t - m_t0));
				return point;
			}

			ndVector m_a;
			ndVector m_b;
			ndFloat32 m_t0;
			ndFloat32 m_perimeter;
		};

		ndWalkSequence(ndFloat32 midParam)
			:ndAnimationSequenceBase()
			,m_segment0()
			,m_segment1()
			,m_xBias(0.11f)
			,m_xStride(1.0f)
			,m_midParam(midParam)
			,m_offsets()
			,m_isGrounded()
		{
			ndFloat32 walkStride = 0.3f;
			const ndVector p0(-walkStride, 0.0f, 0.0f, 0.0f);
			const ndVector p1(walkStride, 0.0f, 0.0f, 0.0f);
			const ndVector p2(-walkStride, 0.1f, 0.0f, 0.0f);
			m_segment0.Init(p0, p1, 0.0f, m_midParam);
			m_segment1.Init(p1, p2, m_midParam, 1.0f);

			m_isGrounded.SetCount(4);
			m_offsets.PushBack(0.0f);
			m_offsets.PushBack(0.0f);
			m_offsets.PushBack(0.0f);
			m_offsets.PushBack(0.0f);

			if (m_midParam > 0.5f)
			{
				// set walk sequence gait offset

				// front gait, left leg is 1/2 offset form the right leg
				m_offsets[1] = 0.5f; // front right leg
				m_offsets[2] = 0.0f; // front left leg

				// rear gait is a 3/4 delay form the front gait
				m_offsets[3] = 0.25f; // rear right leg
				m_offsets[0] = 0.75f; // rear left leg
			}
			else
			{
				// set trot sequence offset

				// front gait, left leg is 1/2 offset form the right leg
				m_offsets[1] = 0.5f; // front right leg
				m_offsets[2] = 0.0f; // front left leg

				// rear gait is a 1/2 delay form the front gait
				m_offsets[3] = 0.0f; // rear right leg
				m_offsets[0] = 0.5f; // rear left leg
			}
		}

		~ndWalkSequence()
		{
		}

		void InitParam(ndFloat32& b, ndFloat32& a, ndFloat32 x0, ndFloat32 t0, ndFloat32 x1, ndFloat32 t1) const
		{
			ndFloat32 den = t1 - t0;
			a = (x1 - x0) / den;
			b = (x0 * t1 - x1 * t0) / den;
		}

		void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
		{
			ndAssert(output.GetCount() == m_offsets.GetCount());
			for (ndInt32 i = 0; i < m_offsets.GetCount(); ++i)
			{
				ndAnimKeyframe& keyFrame = output[i];

				const ndFloat32 t = ndMod(param + m_offsets[i], ndFloat32(1.0f));
				m_isGrounded[i] = t <= m_midParam;
				ndVector posit(m_isGrounded[i] ? m_segment0.Interpolate(t) : m_segment1.Interpolate(t));
				posit.m_x = posit.m_x * m_xStride - m_xBias;
				keyFrame.m_posit = posit;
				keyFrame.m_rotation = ndQuaternion();
			}
		}

		ndSegment m_segment0;
		ndSegment m_segment1;
		ndFloat32 m_xBias;
		ndFloat32 m_xStride;
		ndFloat32 m_midParam;
		ndFixSizeArray<ndFloat32, 4> m_offsets;
		mutable ndFixSizeArray<bool, 4> m_isGrounded;
	};

	class ndQuadrupedModel : public ndModel
	{
		public:
		#define D_SAMPLES_COUNT 128

		D_CLASS_REFLECTION(ndQuadruped_3::ndQuadrupedModel);

		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_basePosition(ndVector::m_wOne)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(ndIkSwivelPositionEffector* const effector)
				:m_basePosition(effector->GetLocalTargetPosition())
				,m_effector(effector)
			{
			}

			ndVector m_basePosition;
			ndIkSwivelPositionEffector* m_effector;
		};

		ndQuadrupedModel(ndDemoEntityManager* const scene, ndDemoEntity* const robotMesh, const ndMatrix& location)
			:ndModel()
			,m_invDynamicsSolver()
			,m_walk(nullptr)
			,m_animBlendTree(nullptr)
			,m_output()
			,m_walkCycle(0.8f)
			,m_trotCycle(0.4f)
			,m_bodyArray()
			,m_effectorsInfo()
			,m_effectorsJoints()
		{
			// make a clone of the mesh and add it to the scene
			ndDemoEntity* const entity = robotMesh->CreateClone();
			scene->AddEntity(entity);
			ndWorld* const world = scene->GetWorld();
			
			ndDemoEntity* const rootEntity = entity->Find(jointsDefinition[0].m_boneName);
			
			// find the floor location 
			ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);
			ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			matrix.m_posit.m_y = floor.m_y;
			
			matrix.m_posit.m_y += 1.0f;
			rootEntity->ResetMatrix(matrix);
			
			// add the root body
			ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, 1.0f, nullptr);
			m_bodyArray.PushBack(rootBody);
			
			ndFixSizeArray<ndBodyDynamic*, 32> parentBone;
			ndFixSizeArray<ndDemoEntity*, 32> childEntities;
			
			ndInt32 stack = 0;
			for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
			{
				childEntities[stack] = child;
				parentBone[stack] = rootBody;
				stack++;
			}
			
			const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
			while (stack)
			{
				stack--;
				ndBodyDynamic* parentBody = parentBone[stack];
				ndDemoEntity* const childEntity = childEntities[stack];
			
				const char* const name = childEntity->GetName().GetStr();
				for (ndInt32 i = 0; i < definitionCount; ++i)
				{
					const ndDefinition& definition = jointsDefinition[i];
					if (!strcmp(definition.m_boneName, name))
					{
						//dTrace(("name: %s\n", name));
						if (definition.m_type == ndDefinition::m_hinge)
						{
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, 1.0f, parentBody);
							m_bodyArray.PushBack(childBody);
			
							const ndMatrix pivotMatrix(childBody->GetMatrix());
							ndIkJointHinge* const hinge = new ndIkJointHinge(pivotMatrix, childBody, parentBody);
							hinge->SetLimitState(true);
							hinge->SetLimits(definition.m_minAngle * ndDegreeToRad, definition.m_maxAngle * ndDegreeToRad);

							ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
							world->AddJoint(hingePtr);
							parentBody = childBody;
						}
						else
						{
							ndDemoEntityNotify* notify = (ndDemoEntityNotify*)parentBody->GetNotifyCallback();
							notify = (ndDemoEntityNotify*)notify->m_parentBody->GetNotifyCallback();
							notify = (ndDemoEntityNotify*)notify->m_parentBody->GetNotifyCallback();
			
							ndMatrix effectorFrame(rootBody->GetMatrix());
							ndMatrix pivotFrame(ndRollMatrix(-90.0f * ndDegreeToRad) * rootBody->GetMatrix());
							pivotFrame.m_posit = notify->GetBody()->GetMatrix().m_posit;
							effectorFrame.m_posit = childEntity->CalculateGlobalMatrix().m_posit;
			
							ndFloat32 regularizer = 0.001f;
							ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame.m_posit, pivotFrame, parentBody, rootBody);
							
							effector->SetSwivelMode(false);
							effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
							effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
							
							const ndVector elbowPoint(childEntity->GetParent()->CalculateGlobalMatrix().m_posit);
							const ndVector dist0(effectorFrame.m_posit - elbowPoint);
							const ndVector dist1(elbowPoint - pivotFrame.m_posit);
							const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
							effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.95f);
							
							ndEffectorInfo info(effector);
							m_effectorsInfo.PushBack(info);
							m_effectorsJoints.PushBack(effector);
						}
						break;
					}
				}
			
				for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
				{
					childEntities[stack] = child;
					parentBone[stack] = parentBody;
					stack++;
				}
			}
			
			NormalizeMassDistribution(100.0f);
			
			m_timer = 0.0f;
			m_param_x0 = -1.0f;
			m_param_xxxx = ndParamMapper(0.0, 0.75f);
			
			m_output.SetCount(4);
			m_walk = new ndAnimationSequencePlayer(&m_walkCycle);
			m_animBlendTree = m_walk;
		}

		ndQuadrupedModel(const ndLoadSaveBase::ndLoadDescriptor& desc)
			:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
			,m_invDynamicsSolver()
			,m_walk(nullptr)
			,m_animBlendTree(nullptr)
			,m_output()
			,m_walkCycle(0.75f)
			,m_trotCycle(0.4f)
			,m_bodyArray()
			,m_effectorsInfo()
			,m_effectorsJoints()
		{
			const nd::TiXmlNode* const modelRootNode = desc.m_rootNode;

			ndAssert(0);
			const nd::TiXmlNode* const bodies = modelRootNode->FirstChild("bodies");
			for (const nd::TiXmlNode* node = bodies->FirstChild(); node; node = node->NextSibling())
			{
				ndInt32 hashId;
				const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
				element->Attribute("int32", &hashId);
				//ndBodyLoaderCache::ndNode* const bodyNode = desc.m_bodyMap->Find(hashId);
				//ndBody* const body = (ndBody*)bodyNode->GetInfo();
			}

			const nd::TiXmlNode* const joints = modelRootNode->FirstChild("joints");
			for (const nd::TiXmlNode* node = joints->FirstChild(); node; node = node->NextSibling())
			{
				ndInt32 hashId;
				const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
				element->Attribute("int32", &hashId);
				//ndJointLoaderCache::ndNode* const jointNode = desc.m_jointMap->Find(hashId);
				//ndJointBilateralConstraint* const joint = (ndJointBilateralConstraint*)jointNode->GetInfo();
			}

			// load root body
			//ndBodyLoaderCache::ndNode* const rootBodyNode = desc.m_bodyMap->Find(xmlGetInt(modelRootNode, "rootBodyHash"));
			//ndBodyDynamic* const rootBody = ((ndBody*)rootBodyNode->GetInfo())->GetAsBodyDynamic();

			// load effector joint
			const nd::TiXmlNode* const endEffectorNode = modelRootNode->FirstChild("endEffector");
			if (xmlGetInt(endEffectorNode, "hasEffector"))
			{
				ndAssert(0);
			}
		}

		~ndQuadrupedModel()
		{
			if (m_animBlendTree)
			{
				delete m_animBlendTree;
			}
		}

		void NormalizeMassDistribution(ndFloat32 mass) const
		{
			ndFloat32 volumeRatio = 0.02f;
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
				if (volume < volumeRatio * maxVolume)
				{
					volume = volumeRatio * maxVolume;
				}
				totalVolume += volume;
			}

			ndFloat32 density = mass / totalVolume;

			for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			{
				ndBodyDynamic* const body = m_bodyArray[i];
				ndFloat32 volume = body->GetCollisionShape().GetVolume();
				if (volume < volumeRatio * maxVolume)
				{
					volume = volumeRatio * maxVolume;
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
				body->SetMassMatrix(inertia);
			}
		}

		void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
		{
			nd::TiXmlElement* const modelRootNode = new nd::TiXmlElement(ClassName());
			desc.m_rootNode->LinkEndChild(modelRootNode);
			modelRootNode->SetAttribute("hashId", desc.m_nodeNodeHash);
			ndModel::Save(ndLoadSaveBase::ndSaveDescriptor(desc, modelRootNode));

			ndAssert(0);
			// save all bodies.
			//nd::TiXmlElement* const bodiesNode = new nd::TiXmlElement("bodies");
			//modelRootNode->LinkEndChild(bodiesNode);
			//for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			//{
			//	nd::TiXmlElement* const paramNode = new nd::TiXmlElement("body");
			//	bodiesNode->LinkEndChild(paramNode);
			//
			//	ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const bodyPartNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_bodyArray[i]);
			//	paramNode->SetAttribute("int32", bodyPartNode->GetInfo());
			//}
			//
			//// save all joints
			//nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("joints");
			//modelRootNode->LinkEndChild(jointsNode);
			//for (ndInt32 i = 0; i < m_jointArray.GetCount(); ++i)
			//{
			//	nd::TiXmlElement* const paramNode = new nd::TiXmlElement("joint");
			//	jointsNode->LinkEndChild(paramNode);
			//
			//	ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* const jointPartNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_jointArray[i]);
			//	paramNode->SetAttribute("int32", jointPartNode->GetInfo());
			//}
			//
			//// indicate which body is the root
			//xmlSaveParam(modelRootNode, "rootBodyHash", desc.m_bodyMap->Find(m_rootBody)->GetInfo());
			//
			//// save end effector info
			//nd::TiXmlElement* const endEffectorNode = new nd::TiXmlElement("endEffector");
			//modelRootNode->LinkEndChild(endEffectorNode);
			//
			//ndAssert(0);
			////xmlSaveParam(endEffectorNode, "hasEffector", m_effector ? 1 : 0);
			////if (m_effector)
			////{
			////	ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody0 = desc.m_bodyMap->Find(m_effector->GetBody0());
			////	ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody1 = desc.m_bodyMap->Find(m_effector->GetBody1());
			////	xmlSaveParam(endEffectorNode, "body0Hash", effectBody0->GetInfo());
			////	xmlSaveParam(endEffectorNode, "body1Hash", effectBody1->GetInfo());
			////}
		}

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
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, entityPart, parentBone));

			ndShapeInstance& instanceShape = body->GetCollisionShape();
			instanceShape.m_shapeMaterial.m_userId = ndApplicationMaterial::m_modelPart;
			instanceShape.m_shapeMaterial.m_userParam[ndContactCallback::m_modelPointer].m_intData = ndUnsigned64(this);

			// add body to the world
			ndSharedPtr<ndBody> bodyPtr(body);
			scene->GetWorld()->AddBody(bodyPtr);

			return body->GetAsBodyDynamic();
		}

		ndBodyDynamic* GetRoot() const
		{
			return m_bodyArray[0];
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
			//ndMatrix comMatrix(m_bodyArray[0]->GetMatrix());
			//comMatrix.m_posit = CalculateCenterOfMass();
			//context.DrawFrame(comMatrix);
			//
			//ndFixSizeArray<ndVector, 16> contactPoints;
			//for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			//{
			//	//const ndEffectorInfo& info = m_effectors[i];
			//	//ndJointBilateralConstraint* const joint = info.m_effector;
			//	//ndBodyKinematic* const body = joint->GetBody0();
			//	//const ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
			//	//ndBodyKinematic::ndContactMap::Iterator it(contacts);
			//	//for (it.Begin(); it; it++)
			//	//{
			//	//	const ndContact* const contact = *it;
			//	//	if (contact->IsActive())
			//	//	{
			//	//		//const ndContactPointList& contactMap = contact->GetContactPoints();
			//	//		//contactPoints.PushBack(contactMap.GetFirst()->GetInfo().m_point);
			//	//		contactPoints.PushBack(body->GetMatrix().TransformVector(info.m_effector->GetLocalMatrix0().m_posit));
			//	//	}
			//	//}
			//	if (m_walkCycle.m_isGrounded[i])
			//	{
			//		const ndEffectorInfo& info = m_effectors[i];
			//		ndJointBilateralConstraint* const joint = info.m_effector;
			//		ndBodyKinematic* const body = joint->GetBody0();
			//		contactPoints.PushBack(body->GetMatrix().TransformVector(info.m_effector->GetLocalMatrix0().m_posit));
			//	}
			//
			//	//	joint->DebugJoint(context);
			//}
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

			for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); ++i)
			{
				const ndEffectorInfo& info = m_effectorsInfo[i];
				ndJointBilateralConstraint* const joint = info.m_effector;
				joint->DebugJoint(context);
			}

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

			m_bodyArray[0]->SetSleepState(false);
			ndFloat32 animSpeed = m_param_xxxx.Interpolate(m_param_x0);
			m_timer = ndMod(m_timer + timestep * animSpeed, ndFloat32(1.0f));

			m_walk->SetParam(1.0f - m_timer);
			m_animBlendTree->Evaluate(m_output);
			for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); i++)
			{
				ndEffectorInfo& info = m_effectorsInfo[i];
				const ndAnimKeyframe& keyFrame = m_output[i];
				ndVector posit(info.m_basePosition);
				posit.m_x += keyFrame.m_posit.m_x;
				posit.m_y += keyFrame.m_posit.m_y;
				posit.m_z += keyFrame.m_posit.m_z;
				info.m_effector->SetLocalTargetPosition(posit);
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

		ndIkSolver m_invDynamicsSolver;
		ndAnimationSequencePlayer* m_walk;
		ndAnimationBlendTreeNode* m_animBlendTree;
		ndAnimationPose m_output;
		ndWalkSequence m_walkCycle;
		ndWalkSequence m_trotCycle;
		ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;

		ndFloat32 m_timer;
		ndReal m_param_x0;
		ndParamMapper m_param_xxxx;
	};
	D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndQuadruped_3::ndQuadrupedModel);

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

			bool change = false;
			ImGui::Text("position x");
			change = change | ImGui::SliderFloat("##x", &m_model->m_param_x0, -1.0f, 1.0f);
			//ImGui::Text("position y");
			//change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
			//ImGui::Text("position z");
			//change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);

			if (change)
			{
				m_model->m_bodyArray[0]->SetSleepState(false);
			}
		}

		ndQuadrupedModel* m_model;
	};
};

using namespace ndQuadruped_3;
void ndQuadrupedTest_3(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndSharedPtr<ndDemoEntity> modelMesh (ndDemoEntity::LoadFbx("spotBoston.fbx", scene));
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
	// register a material for filtering self collisions 
	ndQuadrupedMaterial material;
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.9f;
	material.m_staticFriction1 = 0.9f;
	material.m_dynamicFriction0 = 0.9f;
	material.m_dynamicFriction1 = 0.9f;
	
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_default);
	callback->RegisterMaterial(material, ndApplicationMaterial::m_modelPart, ndApplicationMaterial::m_modelPart);
	
	ndQuadrupedModel* const robot0 = new ndQuadrupedModel(scene, *modelMesh, matrix);
	scene->SetSelectedModel(robot0);
	ndSharedPtr<ndModel> modelPtr(robot0);
	world->AddModel(modelPtr);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//ndQuadrupedModel* const robot1 = new ndQuadrupedModel(scene, robotEntity, matrix);
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
	
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot0->GetRoot()->GetMatrix(), robot0->GetRoot(), world->GetSentinelBody()));
	world->AddJoint(fixJoint);

	ndModelUI* const quadrupedUI = new ndModelUI(scene, robot0);
	ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	scene->Set2DDisplayRenderFunction(quadrupedUIPtr);

	matrix.m_posit.m_x -= 5.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
