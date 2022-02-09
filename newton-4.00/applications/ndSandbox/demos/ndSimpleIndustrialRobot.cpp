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
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

class dSimpleRobotDefinition
{
	public:
	enum jointType
	{
		m_root,
		m_hinge,
		m_effector
	};

	char m_boneName[32];
	ndFloat32 m_mass;
	jointType m_type;
};

static dSimpleRobotDefinition jointsDefinition[] =
{
	{ "base", 100.0f, dSimpleRobotDefinition::m_root},
	{ "base_rotator", 50.0f, dSimpleRobotDefinition::m_hinge },
	{ "arm_0", 5.0f, dSimpleRobotDefinition::m_hinge },
	{ "arm_1", 5.0f, dSimpleRobotDefinition::m_hinge },
	//{ "arm_2", 5.0f, dSimpleRobotDefinition::m_hinge },
	//{ "arm_3", 3.0f, dSimpleRobotDefinition::m_hinge },
	//{ "arm_4", 2.0f, dSimpleRobotDefinition::m_hinge },
	{ "effector", 0.0f, dSimpleRobotDefinition::m_effector },
};

class dSimpleIndustrialRobot : public ndModel
{
	public:
	D_CLASS_REFLECTION(dSimpleIndustrialRobot);

	dSimpleIndustrialRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
		,m_rootBody(nullptr)
		,m_effector(nullptr)
		,m_x(0.5f)
		,m_y(0.0f)
		,m_azimuth(0.0f)
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = robotMesh->CreateClone();
		scene->AddEntity(entity);
		ndWorld* const world = scene->GetWorld();
		
		// find the floor location 
		ndMatrix matrix(location);
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;

		matrix.m_posit.m_y += 1.0f;
		entity->ResetMatrix(matrix);
		
		// add the root body
		m_rootBody = CreateBodyPart(scene, entity, jointsDefinition[0].m_mass, nullptr);
		m_bodyArray.PushBack(m_rootBody);
		
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndBodyDynamic*, 32> parentBone;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = entity->GetChild(); child; child = child->GetSibling())
		{
			childEntities[stack] = child;
			parentBone[stack] = m_rootBody;
			stack++;
		}

		const ndInt32 definitionCount = ndInt32 (sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (stack) 
		{
			stack--;
			ndBodyDynamic* parentBody = parentBone[stack];
			ndDemoEntity* const childEntity = childEntities[stack];

			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; i++) 
			{
				const dSimpleRobotDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					dTrace(("name: %s\n", name));
					if (definition.m_type == dSimpleRobotDefinition::m_hinge)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						m_bodyArray.PushBack(childBody);
						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndJointHinge* const hinge = new ndJointHinge(pivotMatrix, childBody, parentBody);
						m_jointArray.PushBack(hinge);
						world->AddJoint(hinge);
						parentBody = childBody;
					}
					else
					{
						ndMatrix pivotMatrix(childEntity->CalculateGlobalMatrix());
						m_effector = new ndJointKinematicChain(pivotMatrix, parentBody, m_rootBody);
						m_effector->SetMode(true, false);

						ndFloat32 regularizer;
						ndFloat32 springConst;
						ndFloat32 damperConst;
						m_effector->GetLinearSpringDamper(regularizer, springConst, damperConst);
						m_effector->SetLinearSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
						world->AddJoint(m_effector);
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
	}

	dSimpleIndustrialRobot(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
		,m_rootBody(nullptr)
		,m_effector(nullptr)
		,m_x(0.5f)
		,m_y(0.0f)
		,m_azimuth(0.0f)
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
			ndBodyLoaderCache::ndNode* const effectorBodyNode0 = desc.m_bodyMap->Find(xmlGetInt(endEffectorNode, "body0Hash"));
			ndBodyLoaderCache::ndNode* const effectorBodyNode1 = desc.m_bodyMap->Find(xmlGetInt(endEffectorNode, "body1Hash"));

			ndBody* const body0 = (ndBody*)effectorBodyNode0->GetInfo();
			ndBody* const body1 = (ndBody*)effectorBodyNode1->GetInfo();
			dAssert(body1 == m_rootBody);

			const ndMatrix pivotMatrix(body0->GetMatrix());
			m_effector = new ndJointKinematicChain(pivotMatrix, body0->GetAsBodyDynamic(), body1->GetAsBodyDynamic());
		}
	}

	~dSimpleIndustrialRobot()
	{
		if (m_effector && !m_effector->IsInWorld())
		{
			delete m_effector;
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
		for (ndInt32 i = 0; i < m_bodyArray.GetCount(); i++)
		{
			nd::TiXmlElement* const paramNode = new nd::TiXmlElement("body");
			bodiesNode->LinkEndChild(paramNode);

			ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const bodyPartNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_bodyArray[i]);
			paramNode->SetAttribute("int32", bodyPartNode->GetInfo());
		}

		// save all joints
		nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("joints");
		modelRootNode->LinkEndChild(jointsNode);
		for (ndInt32 i = 0; i < m_jointArray.GetCount(); i++)
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

		xmlSaveParam(endEffectorNode, "hasEffector", m_effector ? 1 : 0);
		if (m_effector)
		{
			ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody0 = desc.m_bodyMap->Find(m_effector->GetBody0());
			ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody1 = desc.m_bodyMap->Find(m_effector->GetBody1());
			xmlSaveParam(endEffectorNode, "body0Hash", effectBody0->GetInfo());
			xmlSaveParam(endEffectorNode, "body1Hash", effectBody1->GetInfo());
		}
	}
	
	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndFloat32 mass, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromchildren();
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
		if (m_effector)
		{
			((ndJointBilateralConstraint*)m_effector)->DebugJoint(context);
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

	void ApplyControls(ndDemoEntityManager* const scene)
	{
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Control panel");

		bool change = false;
		ImGui::Text("solver sub steps");
		change = change | ImGui::SliderFloat("##x", &m_x, 0.5f, 3.5f);
		change = change | ImGui::SliderFloat("##y", &m_y, -1.0f, 1.0f);
		change = change | ImGui::SliderFloat("##azimuth", &m_azimuth, -180.0f, 180.0f);

		if (change)
		{ 
			m_rootBody->SetSleepState(false);
		}
	}

	void Update(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::Update(world, timestep);
		if (m_effector)
		{
			// apply target position collected by control panel

			const ndMatrix aximuthMatrix(dYawMatrix(m_azimuth * ndDegreeToRad));
			ndMatrix targetMatrix(m_effector->GetReferenceMatrix());

			// get the reference matrix in local space 
			// (this is because the robot has a build rotation in the model) 
			ndVector localPosit(targetMatrix.UnrotateVector(targetMatrix.m_posit));

			// add the local frame displacement)
			localPosit.m_x += m_x;
			localPosit.m_y += m_y;
			localPosit = aximuthMatrix.RotateVector(localPosit);

			// take new position back to target space
			const ndVector newPosit(targetMatrix.RotateVector(localPosit) + ndVector::m_wOne);
			targetMatrix.m_posit = newPosit;

			//m_x = 2.0f + 1.0f * ndCos(m_azimuth * 0.43f);
			//m_y = 1.0f * ndSin(m_azimuth);
			//targetMatrix.m_posit += targetMatrix.m_front.Scale(m_x * ndCos(m_azimuth));
			//targetMatrix.m_posit += targetMatrix.m_right.Scale(m_x * ndSin(m_azimuth));
			//targetMatrix.m_posit += targetMatrix.m_up.Scale(m_y);
			m_effector->SetTargetMatrix(targetMatrix);
		}
	}

	ndBodyDynamic* m_rootBody;
	ndJointKinematicChain* m_effector;
	ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
	ndFixSizeArray<ndJointBilateralConstraint*, 16> m_jointArray;

	ndFloat32 m_x;
	ndFloat32 m_y;
	ndFloat32 m_azimuth;
};

D_CLASS_REFLECTION_IMPLEMENT_LOADER(dSimpleIndustrialRobot);

static void RobotControlPanel(ndDemoEntityManager* const scene, void* const context)
{
	dSimpleIndustrialRobot* const me = (dSimpleIndustrialRobot*)context;
	me->ApplyControls(scene);
}

void ndSimpleIndustrialRobot (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const robotEntity = scene->LoadFbxMesh("robot.fbx");

	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(dYawMatrix(-90.0f * ndDegreeToRad));
	dSimpleIndustrialRobot* const robot = new dSimpleIndustrialRobot(scene, robotEntity, matrix);
	scene->SetSelectedModel(robot);
	world->AddModel(robot);
	ndBodyDynamic* const root = robot->GetRoot();
	world->AddJoint (new ndJointFix6dof(root->GetMatrix(), root, world->GetSentinelBody()));

	scene->Set2DDisplayRenderFunction(RobotControlPanel, nullptr, robot);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new dSimpleIndustrialRobot(scene, robotEntity, matrix));

	delete robotEntity;

	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(ndQuaternion(), matrix.m_posit);
}
