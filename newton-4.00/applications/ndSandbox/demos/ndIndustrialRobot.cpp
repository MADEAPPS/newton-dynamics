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

class dRobotDefinition
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

static dRobotDefinition jointsDefinition[] =
{
	{ "base", 100.0f, dRobotDefinition::m_root},
	{ "base_rotator", 50.0f, dRobotDefinition::m_hinge },
	{ "arm_0", 5.0f, dRobotDefinition::m_hinge },
	{ "arm_1", 5.0f, dRobotDefinition::m_hinge },
	{ "arm_2", 5.0f, dRobotDefinition::m_hinge },
	{ "arm_3", 3.0f, dRobotDefinition::m_hinge },
	{ "arm_4", 2.0f, dRobotDefinition::m_hinge },
	{ "effector", 0.0f, dRobotDefinition::m_effector },
};

class ndIndustrialRobot : public ndModel
{
	public:
	D_CLASS_REFLECTION(ndIndustrialRobot);

	ndIndustrialRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
		,m_effector(nullptr)
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = robotMesh->CreateClone();
		scene->AddEntity(entity);
		ndWorld* const world = scene->GetWorld();
		
		// find the floor location 
		ndMatrix matrix(location);
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;
		entity->ResetMatrix(matrix);
		
		// add the root body
		ndBodyDynamic* const rootBody = CreateBodyPart(scene, entity, jointsDefinition[0].m_mass, nullptr);
		m_bodyArray.PushBack(rootBody);
		
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndBodyDynamic*, 32> parentBone;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = entity->GetChild(); child; child = child->GetSibling())
		{
			childEntities[stack] = child;
			parentBone[stack] = rootBody;
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
				const dRobotDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					dTrace(("name: %s\n", name));
					if (definition.m_type == dRobotDefinition::m_hinge)
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
						const ndMatrix pivotMatrix(parentBody->GetMatrix());
						m_effector = new ndJointKinematicController(rootBody, parentBody, pivotMatrix);
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

	ndIndustrialRobot(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
	{
		//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
		//
		//ndTree<const ndCharacterNode*, ndUnsigned32> limbMap;
		//for (const nd::TiXmlNode* node = xmlNode->FirstChild(); node; node = node->NextSibling())
		//{
		//	const char* const partName = node->Value();
		//	if (strcmp(partName, "ndCharacterRootNode") == 0)
		//	{
		//		ndCharacterLoadDescriptor loadDesc(desc, &limbMap);
		//		loadDesc.m_rootNode = node;
		//		m_rootNode = new ndCharacterRootNode(loadDesc);
		//		m_rootNode->m_owner = this;
		//	}
		//}
	}

	~ndIndustrialRobot()
	{
		if (m_effector)
		{
			delete m_effector;
		}
	}

	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
	{
		nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
		desc.m_rootNode->LinkEndChild(childNode);
		childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
		ndModel::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

		nd::TiXmlElement* const bodiesNode = new nd::TiXmlElement("bodies");
		childNode->LinkEndChild(bodiesNode);
		for (ndInt32 i = 0; i < m_bodyArray.GetCount(); i++)
		{
			nd::TiXmlElement* const paramNode = new nd::TiXmlElement("body");
			bodiesNode->LinkEndChild(paramNode);

			ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const bodyPartNode = desc.m_bodyMap->Insert(desc.m_bodyMap->GetCount(), m_bodyArray[i]);
			paramNode->SetAttribute("int32", bodyPartNode->GetInfo());
		}

		nd::TiXmlElement* const jointsNode = new nd::TiXmlElement("joints");
		childNode->LinkEndChild(jointsNode);
		for (ndInt32 i = 0; i < m_jointArray.GetCount(); i++)
		{
			nd::TiXmlElement* const paramNode = new nd::TiXmlElement("joint");
			jointsNode->LinkEndChild(paramNode);

			ndTree<ndInt32, const ndJointBilateralConstraint*>::ndNode* const jointPartNode = desc.m_jointMap->Insert(desc.m_jointMap->GetCount(), m_jointArray[i]);
			paramNode->SetAttribute("int32", jointPartNode->GetInfo());
		}

		nd::TiXmlElement* const endEffectorNode = new nd::TiXmlElement("endEffector");
		childNode->LinkEndChild(endEffectorNode);

		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody0 = desc.m_bodyMap->Find(m_effector->GetBody0());
		ndTree<ndInt32, const ndBodyKinematic*>::ndNode* const effectBody1 = desc.m_bodyMap->Find(m_effector->GetBody1());
		xmlSaveParam(endEffectorNode, "body0Hash", effectBody0->GetInfo());
		xmlSaveParam(endEffectorNode, "body1Hash", effectBody1->GetInfo());
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

	void Update(ndWorld* const world, ndFloat32 timestep) 
	{
		//dAssert(0);
		ndModel::Update(world, timestep);
	}

	void PostUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostUpdate(world, timestep);
	}

	void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostTransformUpdate(world, timestep);
	}

	ndJointKinematicController* m_effector;
	ndFixSizeArray<ndJointHinge*, 16> m_jointArray;
	ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
};

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIndustrialRobot);

void ndInsdustrialRobot (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const robotEntity = scene->LoadFbxMesh("robot.fbx");

	ndMatrix matrix(dYawMatrix(-90.0f * ndDegreeToRad));
	ndIndustrialRobot* const robot = new ndIndustrialRobot(scene, robotEntity, matrix);
	scene->SetSelectedModel(robot);
	scene->GetWorld()->AddModel(robot);
	
	matrix.m_posit.m_x += 2.0f;
	matrix.m_posit.m_z -= 2.0f;
	scene->GetWorld()->AddModel(new ndIndustrialRobot(scene, robotEntity, matrix));

	delete robotEntity;

	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(ndQuaternion(), matrix.m_posit);
}
