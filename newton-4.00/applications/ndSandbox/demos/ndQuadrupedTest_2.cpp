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
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequenceBase.h"
#include "ndAnimationSequencePlayer.h"

namespace ndQuadruped_2
{
	class ndDefinition
	{
		public:
		enum ndJointType
		{
			m_root,
			m_hinge,
			m_spherical,
			m_effector
		};

		char m_boneName[32];
		ndJointType m_type;
		ndFloat32 m_mass;
		ndFloat32 m_walkPhase;
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "root_Bone010", ndDefinition::m_root, 40.0f},

		{ "rb_thigh_Bone014", ndDefinition::m_spherical, 3.0f },
		{ "rb_knee_Bone013", ndDefinition::m_hinge, 2.0f },
		{ "rb_effector_Bone009", ndDefinition::m_effector , 0.0f, 0.0f },

		{ "lb_thigh_Bone011", ndDefinition::m_spherical, 3.0f },
		{ "lb_knee_Bone012", ndDefinition::m_hinge, 2.0f },
		{ "lb_effector_Bone010", ndDefinition::m_effector , 0.0f, 0.5f },
		
		{ "fr_thigh_Bone003", ndDefinition::m_spherical, 3.0f },
		{ "fr_knee_Bone004", ndDefinition::m_hinge, 2.0f },
		{ "fr_effector_Bone005", ndDefinition::m_effector , 0.0f, 0.75f },
		
		{ "fl_thigh_Bone008", ndDefinition::m_spherical, 3.0f },
		{ "fl_knee_Bone006", ndDefinition::m_hinge, 2.0f },
		{ "fl_effector_Bone007", ndDefinition::m_effector , 0.0f, 0.25f },
	};

	class ndQuadrupedMaterial : public ndApplicationMaterial
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
	
	class ndEffectorPosit
	{
		public:
		ndEffectorPosit()
		{
		}

		ndEffectorPosit(ndIkSwivelPositionEffector* const effector)
			:m_posit(effector->GetLocalTargetPosition())
			,m_swivel(0.0f)
			,m_effector(effector)
		{
		}

		ndVector m_posit;
		ndFloat32 m_swivel;
		ndIkSwivelPositionEffector* m_effector;
	};

	class ndGaitController: public ndClassAlloc
	{
		public:
		class ndSupportContacts
		{
			public:
			ndSupportContacts()
			{
				m_contact[0] = true;
				m_contact[1] = true;
				m_contact[2] = true;
				m_contact[3] = true;
			}
			bool m_contact[4];
		};

		//class ndContactSequence
		//{
		//	public:
		//	ndContactSequence()
		//	{
		//		Init();
		//	}
		//
		//	ndContactSequence(ndInt32 index)
		//	{
		//		Init();
		//		m_mask[index] = 0;
		//	}
		//
		//	void Init()
		//	{
		//		m_mask[0] = 1;
		//		m_mask[1] = 1;
		//		m_mask[2] = 1;
		//		m_mask[3] = 1;
		//	}
		//
		//	ndInt32 operator[] (ndInt32 i) const
		//	{
		//		return m_mask[i];
		//	}
		//
		//	ndInt32 m_mask[4];
		//};

		ndGaitController(const ndFixSizeArray<ndEffectorPosit, 4>& effectorsPosit)
			:ndClassAlloc()
			,m_effectorsPosit(effectorsPosit)
		{
		}

		virtual ~ndGaitController()
		{
		}

		virtual ndSupportContacts GetSupportContacts() const
		{
			return ndSupportContacts();
		}

		virtual void ExecuteStep(ndFloat32)
		{
			for (ndInt32 i = 0; i < m_effectorsPosit.GetCount(); ++i)
			{
				const ndEffectorPosit& posit = m_effectorsPosit[i];
				posit.m_effector->SetLocalTargetPosition(posit.m_posit);
				posit.m_effector->SetSwivelAngle(posit.m_swivel);
			}
		}

		ndFixSizeArray<ndEffectorPosit, 4> m_effectorsPosit;
	};

	class ndTrotController : public ndGaitController
	{
		public:
		ndTrotController(const ndFixSizeArray<ndEffectorPosit, 4>& effectorsPosit)
			:ndGaitController(effectorsPosit)
		{
		}

		~ndTrotController()
		{
		}

		virtual void ExecuteStep(ndFloat32 timestep)
		{
			ndGaitController::ExecuteStep(timestep);
		}
	};

	class ndWalkController : public ndGaitController
	{
		public:

		ndWalkController(const ndFixSizeArray<ndEffectorPosit, 4>& effectorsPosit)
			:ndGaitController(effectorsPosit)
		{
			//m_sequence.PushBack(ndContactSequence());
			//m_sequence.PushBack(ndContactSequence(2));
			//m_sequence.PushBack(ndContactSequence());
			//m_sequence.PushBack(ndContactSequence(0));
			//m_sequence.PushBack(ndContactSequence());
			//m_sequence.PushBack(ndContactSequence(3));
			//m_sequence.PushBack(ndContactSequence());
			//m_sequence.PushBack(ndContactSequence(1));
		}

		~ndWalkController()
		{
		}

		virtual void ExecuteStep(ndFloat32 timestep)
		{
			ndGaitController::ExecuteStep(timestep);
		}

		//ndFixSizeArray<ndContactSequence, 8> m_sequence;
	};

	class ndStandController : public ndGaitController
	{
		public:
		ndStandController(const ndFixSizeArray<ndEffectorPosit, 4>& effectorsPosit)
			:ndGaitController(effectorsPosit)
		{
		}

		~ndStandController()
		{
		}

		virtual ndSupportContacts GetSupportContacts() const
		{
			return ndSupportContacts();
		}

		virtual void ExecuteStep(ndFloat32 timestep)
		{
			ndGaitController::ExecuteStep(timestep);
		}
	};

	class ndState
	{
		public:
		enum ControllerState
		{
			m_stand,
			m_walk, 
			m_trot,
		};

		ndState()
			:m_controller(nullptr)
			,m_walkController(nullptr)
			,m_trotController(nullptr)
			,m_standController(nullptr)
			,m_state(m_stand)
			,m_state0(m_stand)
			,m_posit()
			,m_tick(0)
		{
		}

		void Init(const ndFixSizeArray<ndEffectorPosit, 4>& effectorsPosit)
		{
			m_walkController = ndSharedPtr<ndGaitController>(new ndWalkController(effectorsPosit));
			m_trotController = ndSharedPtr<ndGaitController>(new ndTrotController(effectorsPosit));
			m_standController = ndSharedPtr<ndGaitController>(new ndStandController(effectorsPosit));
			m_controller = m_standController;
		}

		ndGaitController::ndSupportContacts GetSupportContacts() const
		{
			return m_controller->GetSupportContacts();
		}

		void Update()
		{
			if (m_state != m_state0)
			{
				m_state0 = m_state;
				switch (m_state)
				{
					case m_stand:
					{
						m_tick = 0;
						m_controller = m_standController;
						break;
					}

					case m_walk:
					{
						m_tick = 0;
						m_controller = m_walkController;
						break;
					}

					case m_trot:
					{
						ndAssert(0);
						m_tick = 0;
						m_controller = m_trotController;
						break;
					}
				}
			}

			m_tick++;
			m_posit.SetCount(0);
			for (ndInt32 i = 0; i < m_controller->m_effectorsPosit.GetCount(); ++i)
			{
				m_posit.PushBack(m_controller->m_effectorsPosit[i]);
			}
		}

		ndSharedPtr<ndGaitController> m_controller;
		ndSharedPtr<ndGaitController> m_walkController;
		ndSharedPtr<ndGaitController> m_trotController;
		ndSharedPtr<ndGaitController> m_standController;
		ControllerState m_state;
		ControllerState m_state0;
		ndFixSizeArray<ndEffectorPosit, 4> m_posit;
		ndInt32 m_tick;
	};

	class ndQuadrupedModel : public ndModel
	{
		public:
		D_CLASS_REFLECTION(ndQuadruped_2::ndQuadrupedModel);

		class ndQuadrupedUI : public ndUIEntity
		{
			public:
			ndQuadrupedUI(ndDemoEntityManager* const scene, ndQuadrupedModel* const quadruped)
				:ndUIEntity(scene)
				,m_quadruped(quadruped)
			{
			}

			~ndQuadrupedUI()
			{
			}

			virtual void RenderUI()
			{
			}

			virtual void RenderHelp()
			{
				ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
				m_scene->Print(color, "Control panel");

				ndState& control = m_quadruped->m_state;
				
				//bool change = false;
				ImGui::Text("position x");
				ndInt32 controllerMode = control.m_state;
				ImGui::RadioButton("stand", &controllerMode, ndState::m_stand);
				ImGui::RadioButton("walk", &controllerMode, ndState::m_walk);
				ImGui::RadioButton("trot", &controllerMode, ndState::m_trot);
				ImGui::Separator();

				bool change = controllerMode != control.m_state;
				control.m_state = ndState::ControllerState (controllerMode);
				
				if (change)
				{
					m_quadruped->m_bodyArray[0]->SetSleepState(false);
				}
			}

			ndQuadrupedModel* m_quadruped;
		};

		ndQuadrupedModel(ndDemoEntityManager* const scene, ndDemoEntity* const robotMesh, const ndMatrix& location)
			:ndModel()
			,m_invDynamicsSolver()
			,m_bodyArray()
			,m_effectorsJoints()
			,m_state()
		{
			// make a clone of the mesh and add it to the scene
			ndDemoEntity* const entity = robotMesh->CreateClone();
			scene->AddEntity(entity);
			ndWorld* const world = scene->GetWorld();
			
			ndDemoEntity* const rootEntity = entity->Find(jointsDefinition[0].m_boneName);
			
			// find the floor location 
			ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);
			ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			matrix.m_posit.m_y = floor.m_y + 0.71f;
			rootEntity->ResetMatrix(matrix);
			
			// add the root body
			ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr);
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
			
			m_localFrame = rootEntity->Find("referenceFrame")->CalculateGlobalMatrix(rootEntity);
			const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));

			ndFixSizeArray<ndEffectorPosit, 4> effectorsPosit;
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
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
							m_bodyArray.PushBack(childBody);
							const ndMatrix pivotMatrix(ndRollMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
							ndIkJointHinge* const hinge = new ndIkJointHinge(pivotMatrix, childBody, parentBody);
							hinge->SetLimitState(true);
							hinge->SetLimits(-30.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
							ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
							world->AddJoint(hingePtr);
							parentBody = childBody;
						}
						else if (definition.m_type == ndDefinition::m_spherical)
						{
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
							const ndMatrix pivotMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
							ndIkJointSpherical* const socket = new ndIkJointSpherical(pivotMatrix, childBody, parentBody);
							//socket->SetConeLimit(120.0f * ndDegreeToRad);
							//socket->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
							ndSharedPtr<ndJointBilateralConstraint> ballPtr(socket);
							world->AddJoint(ballPtr);
							parentBody = childBody;
						}
						else
						{
							char refName[256];
							sprintf(refName, "%sreference", name);
							ndAssert(rootEntity->Find(refName));
			
							ndMatrix pivotFrame(rootEntity->Find(refName)->CalculateGlobalMatrix());
							ndMatrix effectorFrame(pivotFrame);
							effectorFrame.m_posit = childEntity->CalculateGlobalMatrix().m_posit;
			
							ndFloat32 regularizer = 0.001f;
							ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame.m_posit, pivotFrame, parentBody, rootBody);

							//effector->SetSwivelMode(false);
							effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
							effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
							
							const ndVector elbowPoint(childEntity->GetParent()->CalculateGlobalMatrix().m_posit);
							const ndVector dist0(effectorFrame.m_posit - elbowPoint);
							const ndVector dist1(elbowPoint - pivotFrame.m_posit);
							const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
							effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.95f);
							
							m_effectorsJoints.PushBack(effector);
							effectorsPosit.PushBack(ndEffectorPosit(effector));
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

			m_state.Init(effectorsPosit);
		}

		ndQuadrupedModel(const ndLoadSaveBase::ndLoadDescriptor& desc)
			:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
			,m_invDynamicsSolver()
			,m_bodyArray()
			,m_effectorsJoints()
			,m_state()
		{
			const nd::TiXmlNode* const modelRootNode = desc.m_rootNode;

			//m_state.Init(effectorsPosit);

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
			ndAssert(0);
			//ndBodyLoaderCache::ndNode* const rootBodyNode = desc.m_bodyMap->Find(xmlGetInt(modelRootNode, "rootBodyHash"));
			//ndBody* const rootbody = (ndBody*)rootBodyNode->GetInfo();
			//m_rootBody = rootbody->GetAsBodyDynamic();

			// load effector joint
			const nd::TiXmlNode* const endEffectorNode = modelRootNode->FirstChild("endEffector");
			if (xmlGetInt(endEffectorNode, "hasEffector"))
			{
				ndAssert(0);
			}
		}

		~ndQuadrupedModel()
		{
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
			ndShapeInstance* const shape = entityPart->CreateCollisionFromChildren();
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

			delete shape;
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
			ndFixSizeArray<ndVector, 16> contactPoints;
			ndGaitController::ndSupportContacts support(m_state.GetSupportContacts());
			for (ndInt32 i = 0; i < m_state.m_posit.GetCount(); ++i)
			{
				const ndEffectorPosit& effectPosit = m_state.m_posit[i];
				ndJointBilateralConstraint* const joint = (ndJointBilateralConstraint*) effectPosit.m_effector;
				joint->DebugJoint(context);
				if (support.m_contact[i])
				{
					ndBodyKinematic* const body = joint->GetBody0();
					contactPoints.PushBack(body->GetMatrix().TransformVector(joint->GetLocalMatrix0().m_posit));
				}
			}

			ndMatrix comMatrix(m_localFrame * m_bodyArray[0]->GetMatrix());
			comMatrix.m_posit = CalculateCenterOfMass();
			context.DrawFrame(comMatrix);

			if (contactPoints.GetCount() >= 3)
			{
				ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
				rotation.TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
				ndInt32 supportCount = ndConvexHull2d(&contactPoints[0], contactPoints.GetCount());
				rotation.Inverse().TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
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
				ndAssert(0);
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
			ndAssert(*m_state.m_controller);
			m_state.m_controller->ExecuteStep(timestep);
			m_state.Update();

			ndSkeletonContainer* const skeleton = GetRoot()->GetSkeleton();
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

		ndMatrix m_localFrame;
		ndIkSolver m_invDynamicsSolver;
		//ndBodyDynamic* m_rootBody;
		ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
		ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;

		ndState m_state;
	};
	D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndQuadruped_2::ndQuadrupedModel);
};

using namespace ndQuadruped_2;
void ndQuadrupedTest_2(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	
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
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndSharedPtr<ndDemoEntity> modelMesh (ndDemoEntity::LoadFbx("spot.fbx", scene));
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
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

	ndQuadrupedModel::ndQuadrupedUI* const quadrupedUI = new ndQuadrupedModel::ndQuadrupedUI(scene, robot0);
	ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	scene->Set2DDisplayRenderFunction(quadrupedUIPtr);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 5.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
