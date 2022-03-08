/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequencePlayer.h"

class dQuadrupedRobotDefinition
{
	public:
	enum jointType
	{
		m_root,
		m_hinge,
		m_spherical,
		m_effector
	};

	char m_boneName[32];
	jointType m_type;
	ndFloat32 m_mass;
	ndFloat32 m_walkPhase;
};

static dQuadrupedRobotDefinition jointsDefinition[] =
{
	{ "root_Bone010", dQuadrupedRobotDefinition::m_root, 40.0f},

	//{ "rb_thigh_Bone014", dQuadrupedRobotDefinition::m_spherical, 3.0f },
	//{ "rb_knee_Bone013", dQuadrupedRobotDefinition::m_hinge, 2.0f },
	//{ "rb_effector_Bone009", dQuadrupedRobotDefinition::m_effector , 0.0f, 0.0f },
	
	{ "lb_thigh_Bone011", dQuadrupedRobotDefinition::m_spherical, 3.0f },
	{ "lb_knee_Bone012", dQuadrupedRobotDefinition::m_hinge, 2.0f },
	{ "lb_effector_Bone010", dQuadrupedRobotDefinition::m_effector , 0.0f, 0.5f },
	
	//{ "fr_thigh_Bone003", dQuadrupedRobotDefinition::m_spherical, 3.0f },
	//{ "fr_knee_Bone004", dQuadrupedRobotDefinition::m_hinge, 2.0f },
	//{ "fr_effector_Bone005", dQuadrupedRobotDefinition::m_effector , 0.0f, 0.75f },
	
	{ "fl_thigh_Bone008", dQuadrupedRobotDefinition::m_spherical, 3.0f },
	{ "fl_knee_Bone006", dQuadrupedRobotDefinition::m_hinge, 2.0f },
	{ "fl_effector_Bone007", dQuadrupedRobotDefinition::m_effector , 0.0f, 0.25f },
};

class dQuadrupedRobot : public ndModel
{
	public:
	#define D_SAMPLES_COUNT 128

	D_CLASS_REFLECTION(dQuadrupedRobot);

	class dHipJoint : public ndIkJointSpherical
	{
		public:
		dHipJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
			:ndIkJointSpherical(pinAndPivotFrame, child, parent)
		{

		}

		void JacobianDerivative(ndConstraintDescritor& desc)
		{
			ndIkJointSpherical::JacobianDerivative(desc);
		}
	};

	class dEffectorInfo
	{
		public:
		dHipJoint* m_hipSocket;
		ndIk6DofEffector* m_effector;
		ndFloat32 m_walkPhase;
		ndFloat32 m_swayAmp;
		ndInt32 m_footOnGround;
	};

	class dQuadrupedWalkSequence : public ndAnimationSequence
	{
		public:
		dQuadrupedWalkSequence()
			:ndAnimationSequence()
		{
		}

		void GenerateKeyFrames(const dEffectorInfo& info, ndVector* const walkCurve)
		{
			ndIk6DofEffector* const effector = info.m_effector;
			for (ndInt32 i = 0; i < (D_SAMPLES_COUNT + 1); ++i)
			{
				walkCurve[i] = ndVector::m_zero;
			}

			const ndInt32 splitParam = D_SAMPLES_COUNT / 2 + 1;

			// vertical leg motion
			ndFloat32 amplitud = 0.1f;
			ndFloat32 period = ndPi / (D_SAMPLES_COUNT - splitParam - 2);
			for (ndInt32 i = splitParam; i < D_SAMPLES_COUNT; ++i)
			{
				ndFloat32 j = ndFloat32(i - splitParam - 1);
				walkCurve[i].m_x = -amplitud * ndSin(period * j);
			}

			// horizontal motion
			ndFloat32 stride = 0.4f;
			for (ndInt32 i = 0; i < splitParam; ++i)
			{
				ndFloat32 j = ndFloat32(i);
				walkCurve[i].m_y = stride  * (0.5f - j / splitParam);
			}

			for (ndInt32 i = splitParam; i < D_SAMPLES_COUNT; ++i)
			{
				ndFloat32 j = ndFloat32(i - splitParam);
				walkCurve[i].m_y = -stride  * (0.5f - j / (D_SAMPLES_COUNT - splitParam));
			}
			walkCurve[D_SAMPLES_COUNT].m_y = walkCurve[0].m_y;

			// sideway motion
			ndFloat32 swayAmp = info.m_swayAmp;
swayAmp = 0.0f;
			for (ndInt32 i = 0; i < splitParam; ++i)
			{
				walkCurve[i].m_z = swayAmp;
			}
			for (ndInt32 i = splitParam; i < D_SAMPLES_COUNT; ++i)
			{
				ndFloat32 j = ndFloat32(i - splitParam - 1);
				walkCurve[i].m_z = swayAmp * (1.0f - ndSin(period * j));
			}
			walkCurve[D_SAMPLES_COUNT].m_z = walkCurve[0].m_z;

			ndMatrix offsetMatrix(effector->GetOffsetMatrix());
			for (ndInt32 i = 0; i < (D_SAMPLES_COUNT + 1); ++i)
			{
				walkCurve[i] += offsetMatrix.m_posit;
			}
		}

		void AddTrack(const dEffectorInfo& info)
		{
			ndIk6DofEffector* const effector = info.m_effector;
			ndAnimationKeyFramesTrack* const track = ndAnimationSequence::AddTrack();

			ndMatrix offsetMatrix(effector->GetOffsetMatrix());

			// set two identity rotations 
			track->m_rotation.m_param.PushBack(0.0f);
			track->m_rotation.PushBack(ndQuaternion());
			
			track->m_rotation.m_param.PushBack(1.0f);
			track->m_rotation.PushBack(ndQuaternion());

			// build the position key frames
			ndFixSizeArray<ndVector, D_SAMPLES_COUNT + 1> positionCurve;
			positionCurve.SetCount(D_SAMPLES_COUNT + 1);
			GenerateKeyFrames(info, &positionCurve[0]);

			// apply the phase angle to this sequence
			m_phase.PushBack (info.m_walkPhase);
			for (ndInt32 i = 0; i <= D_SAMPLES_COUNT; ++i)
			{
				ndInt32 index = (i + ndInt32 (info.m_walkPhase * D_SAMPLES_COUNT)) % D_SAMPLES_COUNT;
				track->m_position.PushBack(positionCurve[index]);
				track->m_position.m_param.PushBack(ndFloat32(i) / D_SAMPLES_COUNT);
			}
		}

		bool IsOnGround(ndFloat32 param, ndInt32 trackIndex)
		{
			ndInt32 index = ndInt32 ((param + m_phase[trackIndex]) * D_SAMPLES_COUNT) % D_SAMPLES_COUNT;
			return (index < D_SAMPLES_COUNT / 2 + 1);
		}
		ndFixSizeArray<ndFloat32,4> m_phase;
	};

	class dQuadrupedBalanceController: public ndAnimationBlendTreeNode, public ndIkSolver
	{
		public: 
		dQuadrupedBalanceController(ndAnimationBlendTreeNode* const input, dQuadrupedRobot* const model)
			:ndAnimationBlendTreeNode(input)
			,ndIkSolver()
			//,m_centerOfMassCorrection(ndVector::m_zero)
			,m_world(nullptr)
			,m_model(model)
			,m_timestep(ndFloat32 (0.0f))
		{
			m_centerOfMassCorrection.SetCount(4);
			for (ndInt32 i = 0; i < m_centerOfMassCorrection.GetCount(); ++i)
			{
				m_centerOfMassCorrection[i] = ndVector::m_zero;
			}
		}

		ndVector CalculateCenterOfMass() const;
		ndVector PredictCenterOfMassVelocity() const;
		void GetSupportPolygon(ndFixSizeArray<ndVector, 16>& polygon) const;
		ndVector ProjectCenterOfMass(const ndFixSizeArray<ndVector, 16>& polygon, const ndVector& com) const;

		void Evaluate(ndAnimationPose& output);

		ndFixSizeArray<ndVector, 4> m_centerOfMassCorrection;
		//ndVector m_centerOfMassCorrection;
		ndWorld* m_world;
		dQuadrupedRobot* m_model;
		ndFloat32 m_timestep;
	};

	dQuadrupedRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
		,m_referenceFrame(dGetIdentityMatrix())
		,m_rootBody(nullptr)
		,m_walkCycle(nullptr)
		,m_balanceController(nullptr)
		,m_effectors()
		,m_bodyArray()
		,m_jointArray()
		,m_walkParam(0.0f)
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = (ndDemoEntity*)robotMesh->CreateClone();
		scene->AddEntity(entity);
		ndWorld* const world = scene->GetWorld();

		ndDemoEntity* const rootEntity = entity->Find(jointsDefinition[0].m_boneName);
	
		// find the floor location 
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;

		matrix.m_posit.m_y += 0.705f;
		rootEntity->ResetMatrix(matrix);

		// add the root body
		m_rootBody = CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr);
		m_bodyArray.PushBack(m_rootBody);
		m_referenceFrame = rootEntity->Find("referenceFrame")->CalculateGlobalMatrix(rootEntity);

		ndFixSizeArray<ndBodyDynamic*, 32> parentBone;
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling())
		{
			childEntities[stack] = child;
			parentBone[stack] = m_rootBody;
			stack++;
		}

		dHipJoint* socket = nullptr;

		const ndInt32 definitionCount = ndInt32 (sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (stack) 
		{
			stack--;
			ndBodyDynamic* parentBody = parentBone[stack];
			ndDemoEntity* const childEntity = childEntities[stack];

			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; ++i) 
			{
				const dQuadrupedRobotDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//dTrace(("name: %s\n", name));
					if (definition.m_type == dQuadrupedRobotDefinition::m_hinge)
					{

						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						m_bodyArray.PushBack(childBody);
						
						const ndMatrix pivotMatrix(dRollMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
						ndIkJointHinge* const hinge = new ndIkJointHinge(pivotMatrix, childBody, parentBody);
						hinge->SetLimits(-60.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
						m_jointArray.PushBack(hinge);
						world->AddJoint(hinge);
						parentBody = childBody;
					}
					else if (definition.m_type == dQuadrupedRobotDefinition::m_spherical)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						m_bodyArray.PushBack(childBody);
						
						const ndMatrix pivotMatrix(dYawMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
						socket = new dHipJoint(pivotMatrix, childBody, parentBody);
						socket->SetConeLimit(120.0f * ndDegreeToRad);
						socket->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);

						world->AddJoint(socket);
						parentBody = childBody;
					}
					else
					{
						char refName[256];
						sprintf(refName, "%sreference", name);
						dAssert(rootEntity->Find(refName));

						const ndMatrix effectorFrame(childEntity->CalculateGlobalMatrix());
						const ndMatrix pivotFrame(rootEntity->Find(refName)->CalculateGlobalMatrix());

						ndShapeInstance sphere(new ndShapeSphere(0.05f));
						sphere.SetCollisionMode(false);

						ndBodyDynamic* const childBody = new ndBodyDynamic();
						childBody->SetMatrix(effectorFrame);
						childBody->SetCollisionShape(sphere);
						childBody->SetMassMatrix(1.0f / parentBody->GetInvMass(), sphere);
						childBody->SetNotifyCallback(new ndDemoEntityNotify(scene, nullptr, parentBody));
						m_bodyArray.PushBack(childBody);

						ndMatrix bootFrame;
						bootFrame.m_front = pivotFrame.m_up;
						bootFrame.m_up = pivotFrame.m_right;
						bootFrame.m_right = bootFrame.m_front.CrossProduct(bootFrame.m_up);
						bootFrame.m_posit = effectorFrame.m_posit;

						ndJointDoubleHinge* const bootJoint = new ndIkJointDoubleHinge(bootFrame, childBody, parentBody);
						bootJoint->SetLimits0(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
						bootJoint->SetLimits1(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
						// add body to the world
						scene->GetWorld()->AddBody(childBody);
						scene->GetWorld()->AddJoint(bootJoint);

						ndVector legSide(m_rootBody->GetMatrix().UntransformVector(pivotFrame.m_posit));
						ndFloat32 swayAmp(0.8f * legSide.m_y);

						dEffectorInfo info;
						info.m_hipSocket = socket;
						info.m_effector = new ndIk6DofEffector(effectorFrame, pivotFrame, childBody, m_rootBody);
						//ndFloat32 regularizer = 1.0e-4f;
						ndFloat32 regularizer = 1.0e-1f;
						info.m_effector->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
						info.m_effector->SetLinearSpringDamper(regularizer, 2500.0f, 50.0f);
						info.m_effector->SetAngularSpringDamper(regularizer, 2500.0f, 50.0f);

						info.m_footOnGround = 0;
						info.m_swayAmp = swayAmp;
						info.m_walkPhase = definition.m_walkPhase;
						m_effectors.PushBack(info);
					}
					break;
				}
			}

			for (ndDemoEntity* child = childEntity->GetChild(); child; child = child->GetSibling())
			{
				childEntities[stack] = child;
				parentBone[stack] = parentBody;
				stack++;
			}
		}

		BuildAnimationTree();
	}

	dQuadrupedRobot(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
		,m_rootBody(nullptr)
		,m_effectors()
		,m_walkParam(0.0f)
	{
		const nd::TiXmlNode* const modelRootNode = desc.m_rootNode;

		const nd::TiXmlNode* const bodies = modelRootNode->FirstChild("bodies");
		for (const nd::TiXmlNode* node = bodies->FirstChild(); node; node = node->NextSibling())
		{
			ndInt32 hashId;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hashId);
			ndBodyLoaderCache::ndNode* const bodyNode = desc.m_bodyMap->Find(hashId);

			ndBody* const body = (ndBody*)bodyNode->GetInfo();
			m_bodyArray.PushBack(body->GetAsBodyDynamic());
		}

		const nd::TiXmlNode* const joints = modelRootNode->FirstChild("joints");
		for (const nd::TiXmlNode* node = joints->FirstChild(); node; node = node->NextSibling())
		{
			ndInt32 hashId;
			const nd::TiXmlElement* const element = (nd::TiXmlElement*) node;
			element->Attribute("int32", &hashId);
			ndJointLoaderCache::ndNode* const jointNode = desc.m_jointMap->Find(hashId);

			ndJointBilateralConstraint* const joint = (ndJointBilateralConstraint*)jointNode->GetInfo();
			m_jointArray.PushBack((ndJointHinge*)joint);
		}

		// load root body
		ndBodyLoaderCache::ndNode* const rootBodyNode = desc.m_bodyMap->Find(xmlGetInt(modelRootNode, "rootBodyHash"));
		ndBody* const rootbody = (ndBody*)rootBodyNode->GetInfo();
		m_rootBody = rootbody->GetAsBodyDynamic();
		
		// load effector joint
		const nd::TiXmlNode* const endEffectorNode = modelRootNode->FirstChild("endEffector");
		if (xmlGetInt(endEffectorNode, "hasEffector"))
		{
			dAssert(0);
		}
	}

	~dQuadrupedRobot()
	{
		if (m_balanceController)
		{
			delete m_balanceController;
		}

		for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		{
			ndIk6DofEffector* const effector = m_effectors[i].m_effector;
			if (!effector->IsInWorld())
			{
				delete effector;
			}
		}
	}

	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
	{
		nd::TiXmlElement* const modelRootNode = new nd::TiXmlElement(ClassName());
		desc.m_rootNode->LinkEndChild(modelRootNode);
		modelRootNode->SetAttribute("hashId", desc.m_nodeNodeHash);
		ndModel::Save(ndLoadSaveBase::ndSaveDescriptor(desc, modelRootNode));

		// save all bodies.
		nd::TiXmlElement* const bodiesNode = new nd::TiXmlElement("bodies");
		modelRootNode->LinkEndChild(bodiesNode);
		for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
		{
			nd::TiXmlElement* const paramNode = new nd::TiXmlElement("body");
			bodiesNode->LinkEndChild(paramNode);

			ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const bodyPartNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_bodyArray[i]);
			paramNode->SetAttribute("int32", bodyPartNode->GetInfo());
		}

		// save all joints
		nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("joints");
		modelRootNode->LinkEndChild(jointsNode);
		for (ndInt32 i = 0; i < m_jointArray.GetCount(); ++i)
		{
			nd::TiXmlElement* const paramNode = new nd::TiXmlElement("joint");
			jointsNode->LinkEndChild(paramNode);

			ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* const jointPartNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_jointArray[i]);
			paramNode->SetAttribute("int32", jointPartNode->GetInfo());
		}

		// indicate which body is the root
		xmlSaveParam(modelRootNode, "rootBodyHash", desc.m_bodyMap->Find(m_rootBody)->GetInfo());

		// save end effector info
		nd::TiXmlElement* const endEffectorNode = new nd::TiXmlElement("endEffector");
		modelRootNode->LinkEndChild(endEffectorNode);

		dAssert(0);
		//xmlSaveParam(endEffectorNode, "hasEffector", m_effector ? 1 : 0);
		//if (m_effector)
		//{
		//	ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody0 = desc.m_bodyMap->Find(m_effector->GetBody0());
		//	ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody1 = desc.m_bodyMap->Find(m_effector->GetBody1());
		//	xmlSaveParam(endEffectorNode, "body0Hash", effectBody0->GetInfo());
		//	xmlSaveParam(endEffectorNode, "body1Hash", effectBody1->GetInfo());
		//}
	}

	void BuildAnimationTree()
	{
		// create a procedural walk cycle
		dQuadrupedWalkSequence* const walk = new dQuadrupedWalkSequence();
		for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		{
			walk->AddTrack(m_effectors[i]);
		}

		// build the pose tree blender, for now just the walk
		m_walkCycle = new ndAnimationSequencePlayer(walk);

		// bind animation tree to key frame pose
		for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		{
			ndAnimKeyframe keyFrame;
			keyFrame.m_userData = m_effectors[i].m_effector;
			m_output.PushBack(keyFrame);
		}

		m_balanceController = new dQuadrupedBalanceController (m_walkCycle, this);
	}

	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndFloat32 mass, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromChildren();
		dAssert(shape);
		
		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());
		
		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(mass, *shape);
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entityPart, parentBone));
		
		delete shape;

		// add body to the world
		scene->GetWorld()->AddBody(body);
		return body;
	}

	ndBodyDynamic* GetRoot() const
	{
		return m_rootBody;
	}

	void Debug(ndConstraintDebugCallback& context) const
	{
		ndFixSizeArray<ndVector , 4> supportPolygon;
		for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		{
			ndJointBilateralConstraint* const joint = m_effectors[i].m_effector;
			//joint->DebugJoint(context);
			if (m_effectors[i].m_footOnGround)
			{
				const ndVector posit(joint->GetBody0()->GetMatrix().TransformVector(joint->GetLocalMatrix0().m_posit));
				supportPolygon.PushBack(posit);
			}
		}

		ndVector xxx0(m_effectors[0].m_hipSocket->GetBody0()->GetMatrix().TransformVector(m_effectors[0].m_hipSocket->GetLocalMatrix0().m_posit));
		ndVector xxx1(m_effectors[1].m_hipSocket->GetBody0()->GetMatrix().TransformVector(m_effectors[1].m_hipSocket->GetLocalMatrix0().m_posit));
		context.DrawLine(xxx0, xxx1, ndVector (0.0f, 0.0f, 0.0f, 0.0f));

		// Draw support polygon
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		ndInt32 i0 = supportPolygon.GetCount() - 1;
		for (ndInt32 i = 0; i < supportPolygon.GetCount(); ++i)
		{
			context.DrawLine(supportPolygon[i], supportPolygon[i0], color);
			i0 = i;
		}

		const ndVector com(m_balanceController->CalculateCenterOfMass());
		ndMatrix rootMatrix(m_rootBody->GetMatrix());
		rootMatrix.m_posit = com;;
		context.DrawFrame(rootMatrix);

		// Draw center of mass projection
		ndBigVector p0Out;
		ndBigVector p1Out;
		ndBigVector p0(rootMatrix.m_posit);

		p0.m_y -= 1.0f;
		if (supportPolygon.GetCount() >= 3)
		{
			ndBigVector poly[16];
			for (ndInt32 i = 0; i < supportPolygon.GetCount(); ++i)
			{
				poly[i] = supportPolygon[i];
			}
			dRayToPolygonDistance(rootMatrix.m_posit, p0, poly, supportPolygon.GetCount(), p0Out, p1Out);
		} 
		else if (supportPolygon.GetCount() == 2)
		{
			dRayToRayDistance(rootMatrix.m_posit, p0, supportPolygon[0], supportPolygon[1], p0Out, p1Out);
		}
		ndVector t0(p0Out);
		ndVector t1(p1Out);
		context.DrawPoint(t0, ndVector(1.0f, 0.0f, 0.0f, 0.0f));
		context.DrawPoint(t1, ndVector(0.0f, 0.0f, 1.0f, 0.0f));
	}

	void PostUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostUpdate(world, timestep);
	}

	void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostTransformUpdate(world, timestep);
	}

	void ApplyControls(ndDemoEntityManager* const scene)
	{
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Control panel");

		bool change = false;
		//ImGui::Text("ik solver passes");

		//ImGui::Text("position x");
		//change = change | ImGui::SliderFloat("##x", &m_x, 0.0f, 5.0f);
		//ImGui::Text("position y");
		//change = change | ImGui::SliderFloat("##y", &m_y, -1.5f, 2.0f);
		//ImGui::Text("azimuth");
		//change = change | ImGui::SliderFloat("##azimuth", &m_azimuth, -180.0f, 180.0f);
		//
		//ImGui::Text("gripper");
		//change = change | ImGui::SliderFloat("##gripperPosit", &m_gripperPosit, -0.2f, 0.03f);
		//
		//ImGui::Text("pitch");
		//change = change | ImGui::SliderFloat("##pitch", &m_pitch, -180.0f, 180.0f);
		//ImGui::Text("yaw");
		//change = change | ImGui::SliderFloat("##yaw", &m_yaw, -180.0f, 180.0f);
		//ImGui::Text("roll");
		//change = change | ImGui::SliderFloat("##roll", &m_roll, -180.0f, 180.0f);

		if (change)
		{
			m_rootBody->SetSleepState(false);
		}
	}

	void Update(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::Update(world, timestep);

		ndSkeletonContainer* const skeleton = m_rootBody->GetSkeleton();
		dAssert(skeleton);

		m_rootBody->SetSleepState(false);

		if (!m_balanceController->IsSleeping(skeleton))
		{
			ndFloat32 walkSpeed = 1.0f;
			m_walkParam = ndFmod(m_walkParam + walkSpeed * timestep, ndFloat32 (1.0f));

	m_walkParam = 0.8f;
			// advance walk animation (for now, later this will be control by game play logic)
			m_walkCycle->SetParam(m_walkParam);
			dQuadrupedWalkSequence* const sequence = (dQuadrupedWalkSequence*)m_walkCycle->GetSequence();
			for (ndInt32 i = 0; i < m_output.GetCount(); ++i)
			{
				m_effectors[i].m_footOnGround = sequence->IsOnGround(m_walkParam, i);
			}
			
			m_balanceController->m_world = world;
			m_balanceController->m_timestep = timestep;
			m_balanceController->Evaluate(m_output);
		}
	}

	ndMatrix m_referenceFrame;
	ndBodyDynamic* m_rootBody;
	ndAnimationSequencePlayer* m_walkCycle;
	dQuadrupedBalanceController* m_balanceController;
	
	ndAnimationPose m_output;
	ndFixSizeArray<dEffectorInfo, 4> m_effectors;
	ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
	ndFixSizeArray<ndJointBilateralConstraint*, 16> m_jointArray;
	ndFloat32 m_walkParam;
};

D_CLASS_REFLECTION_IMPLEMENT_LOADER(dQuadrupedRobot);

void dQuadrupedRobot::dQuadrupedBalanceController::GetSupportPolygon(ndFixSizeArray<ndVector, 16>& supportPolygon) const
{
	supportPolygon.SetCount(0);
	for (ndInt32 i = 0; i < m_model->m_effectors.GetCount(); ++i)
	{
		if (m_model->m_effectors[i].m_footOnGround)
		{
			ndJointBilateralConstraint* const joint = m_model->m_effectors[i].m_effector;
			const ndVector posit(joint->GetBody0()->GetMatrix().TransformVector(joint->GetLocalMatrix0().m_posit));
			supportPolygon.PushBack(posit);
		}
	}
}

ndVector dQuadrupedRobot::dQuadrupedBalanceController::ProjectCenterOfMass(const ndFixSizeArray<ndVector, 16>& supportPolygon, const ndVector& com) const
{
	ndBigVector p0Out;
	ndBigVector p1Out;
	ndBigVector p1(com);
	p1.m_y -= 1.0f;
	if (supportPolygon.GetCount() >= 3)
	{
		ndBigVector poly[16];
		for (ndInt32 i = 0; i < supportPolygon.GetCount(); ++i)
		{
			poly[i] = supportPolygon[i];
		}
		dRayToPolygonDistance(com, p1, poly, supportPolygon.GetCount(), p0Out, p1Out);
	}
	else if (supportPolygon.GetCount() == 2)
	{
		dRayToRayDistance(com, p1, supportPolygon[0], supportPolygon[1], p0Out, p1Out);
	}
	p1Out.m_y = com.m_y;
	return p1Out;
}

ndVector dQuadrupedRobot::dQuadrupedBalanceController::CalculateCenterOfMass() const
{
	ndFloat32 toltalMass = 0.0f;
	ndVector com(ndVector::m_zero);
	for (ndInt32 i = 0; i < m_model->m_bodyArray.GetCount(); ++i)
	{
		ndBodyDynamic* const body = m_model->m_bodyArray[i];
		ndFloat32 mass = body->GetMassMatrix().m_w;
		ndVector comMass(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
		com += comMass.Scale(mass);
		toltalMass += mass;
	}
	com = com.Scale(1.0f / toltalMass);
	com.m_w = 1.0f;
	return com;
}

ndVector dQuadrupedRobot::dQuadrupedBalanceController::PredictCenterOfMassVelocity() const
{
	ndFloat32 toltalMass = 0.0f;
	ndVector momentum(ndVector::m_zero);
	for (ndInt32 i = 0; i < m_model->m_bodyArray.GetCount(); ++i)
	{
		ndBodyDynamic* const body = m_model->m_bodyArray[i];
		ndFloat32 mass = body->GetMassMatrix().m_w;
		const ndVector invMass(body->GetInvMass());
		ndVector veloc(body->GetVelocity() + body->GetAccel().Scale (body->GetInvMass() * m_timestep));
		momentum += veloc.Scale(mass);
		toltalMass += mass;
	}
	ndVector veloc (momentum.Scale(1.0f / toltalMass));
	return veloc;
}

void dQuadrupedRobot::dQuadrupedBalanceController::Evaluate(ndAnimationPose& output)
{
	// get the animation pose
	ndAnimationBlendTreeNode::Evaluate(output);

	// calculate thee accelerations needed for this pose
	ndSkeletonContainer* const skeleton = m_model->GetRoot()->GetSkeleton();
	dAssert(skeleton);

	const ndJointBilateralConstraint* joints[4];

	// here we will integrate the model to get the center of mass velocity 
	// and with that the final pose will be adjusted to keep the balance
	// for now just assume the pose is valid and return. 

static int xxxxx;
xxxxx++;
	ndVector step(ndVector::m_zero);
	ndFixSizeArray<ndVector, 16> polygon;
	GetSupportPolygon(polygon);
	if (polygon.GetCount())
	{
		const ndVector origin(CalculateCenterOfMass());
		const ndVector veloc(PredictCenterOfMassVelocity() & ndVector::m_triplexMask);
		const ndVector com(origin + veloc.Scale(m_timestep));
		const ndVector target(ProjectCenterOfMass(polygon, com));

		ndFloat32 correctionFactor = 0.1f;
		step = ndVector::m_triplexMask & (com - target).Scale(correctionFactor);
	}
		
	for (ndInt32 i = 0; i < output.GetCount(); ++i)
	{
		const ndAnimKeyframe& keyFrame = output[i];
		const dEffectorInfo& info = m_model->m_effectors[i];

		ndMatrix poseMatrix(keyFrame.m_rotation, keyFrame.m_posit);
		ndMatrix matrix(poseMatrix);
		matrix.m_posit += m_centerOfMassCorrection[i];

		ndIk6DofEffector* const effector = info.m_effector;
		joints[i] = effector;
		dAssert(effector == output[i].m_userData);
		
		if (info.m_footOnGround)
		{
			ndMatrix pivotMatrix(effector->GetLocalMatrix1() * effector->GetBody1()->GetMatrix());
			const ndMatrix effectorMatrix(matrix * pivotMatrix);
			pivotMatrix.m_posit -= step;
			matrix = poseMatrix;
			matrix.m_posit = (effectorMatrix * pivotMatrix.Inverse()).m_posit;
			effector->SetOffsetMatrix(matrix);
			m_centerOfMassCorrection[i] = matrix.m_posit - poseMatrix.m_posit;
		}
		effector->SetOffsetMatrix(matrix);
	}

	SolverBegin(skeleton, joints, output.GetCount(), m_world, m_timestep);
	Solve();
	SolverEnd();
}

void RobotControlPanel(ndDemoEntityManager* const scene, void* const context)
{
	dQuadrupedRobot* const me = (dQuadrupedRobot*)context;
	me->ApplyControls(scene);
}

void ndQuadrupedRobot(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const robotEntity = scene->LoadFbxMesh("spot.fbx");

	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(dYawMatrix(-0.0f * ndDegreeToRad));

	dQuadrupedRobot* const robot0 = new dQuadrupedRobot(scene, robotEntity, matrix);
	scene->SetSelectedModel(robot0);
	world->AddModel(robot0);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//dQuadrupedRobot* const robot1 = new dQuadrupedRobot(scene, robotEntity, matrix);
	//world->AddModel(robot1);

	delete robotEntity;

	//ndVector posit(matrix.m_posit);
	//posit.m_x += 1.5f;
	//posit.m_z += 1.5f;
	//AddBox(scene, posit, 2.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, posit, 1.0f, 0.3f, 0.4f, 0.7f);

	//posit.m_x += 0.6f;
	//posit.m_z += 0.2f;
	//AddBox(scene, posit, 8.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, posit, 4.0f, 0.3f, 0.4f, 0.7f);

	//world->AddJoint(new ndJointFix6dof(robot0->GetRoot()->GetMatrix(), robot0->GetRoot(), world->GetSentinelBody()));
	//scene->Set2DDisplayRenderFunction(RobotControlPanel, nullptr, robot0);

	matrix.m_posit.m_x -= 2.5f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
