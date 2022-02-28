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

class dAdvancedRobotDefinition
{
	public:
	enum jointType
	{
		m_root,
		m_hinge,
		m_slider,
		m_effector
	};

	char m_boneName[32];
	jointType m_type;
	ndFloat32 m_mass;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	ndFloat32 m_maxTorque;
};

static dAdvancedRobotDefinition jointsDefinition[] =
{
	{ "base", dAdvancedRobotDefinition::m_root, 100.0f, 0.0f, 0.0f, 1.0e5f},
	{ "base_rotator", dAdvancedRobotDefinition::m_hinge, 50.0f, -1.0e10f, 1.0e10f, 1.0e5f },
	{ "arm_0", dAdvancedRobotDefinition::m_hinge , 5.0f, -140.0f * ndDegreeToRad, 1.0f * ndDegreeToRad, 1.0e5f },
	{ "arm_1", dAdvancedRobotDefinition::m_hinge , 5.0f, - 5.0f * ndDegreeToRad, 120.0f * ndDegreeToRad, 1.0e5f },
	{ "arm_2", dAdvancedRobotDefinition::m_hinge , 5.0f, -360.0f * ndDegreeToRad, 360.0f * ndDegreeToRad, 1.0e5f },
	{ "arm_3", dAdvancedRobotDefinition::m_hinge , 3.0f, -360.0f * ndDegreeToRad, 360.0f * ndDegreeToRad, 1.0e5f },
	{ "arm_4", dAdvancedRobotDefinition::m_hinge , 2.0f, -360.0f * ndDegreeToRad, 360.0f * ndDegreeToRad, 1.0e5f },
	{ "gripperLeft", dAdvancedRobotDefinition::m_slider , 1.0f, -0.2f, 0.03f, 1.0e5f },
	{ "gripperRight", dAdvancedRobotDefinition::m_slider , 1.0f, -0.2f, 0.03f, 1.0e5f },
	{ "effector", dAdvancedRobotDefinition::m_effector , 0.0f, 0.0f, 0.0f, 1.0e5f },
};

class dAdvancedIndustrialRobot : public ndModel
{
	public:
	D_CLASS_REFLECTION(dAdvancedIndustrialRobot);

	dAdvancedIndustrialRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
		,m_rootBody(nullptr)
		,m_leftGripper(nullptr)
		,m_rightGripper(nullptr)
		,m_effector(nullptr)
		,m_invDynamicsSolver()
		,m_baseRotation(dGetIdentityMatrix())
		,m_x(0.0f)
		,m_y(0.0f)
		,m_azimuth(0.0f)
		,m_gripperPosit(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const rootEntity = (ndDemoEntity*)robotMesh->CreateClone();
		scene->AddEntity(rootEntity);
		ndWorld* const world = scene->GetWorld();
		
		// find the floor location 
		ndMatrix matrix(location);
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;

		//matrix.m_posit.m_y += 1.0f;
		rootEntity->ResetMatrix(matrix);
		
		// add the root body
		m_rootBody = CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr);
		m_bodyArray.PushBack(m_rootBody);
		
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndBodyDynamic*, 32> parentBone;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling())
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
				const dAdvancedRobotDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//dTrace(("name: %s\n", name));
					if (definition.m_type == dAdvancedRobotDefinition::m_hinge)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						m_bodyArray.PushBack(childBody);

						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndJointIkHinge* const hinge = new ndJointIkHinge(pivotMatrix, childBody, parentBody);
						hinge->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						m_jointArray.PushBack(hinge);
						world->AddJoint(hinge);
						parentBody = childBody;
					}
					else if (definition.m_type == dAdvancedRobotDefinition::m_slider)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						m_bodyArray.PushBack(childBody);

						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndJointSlider* const slider = new ndJointSlider(pivotMatrix, childBody, parentBody);
						slider->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						slider->SetAsSpringDamper(0.01f, 2000.0f, 100.0f);

						if (!strstr(definition.m_boneName, "Left"))
						{
							m_leftGripper = slider;
						}
						else
						{
							m_rightGripper = slider;
						}
						world->AddJoint(slider);
						parentBody = childBody;
					}
					else
					{
						//ndMatrix pivotMatrix(childEntity->CalculateGlobalMatrix());
						ndMatrix pivotMatrix(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
						pivotMatrix.m_posit = childEntity->CalculateGlobalMatrix().m_posit;
						m_effector = new ndIk6DofEffector(pivotMatrix, parentBody, m_rootBody);
						m_effector->SetMode(true, true);

						m_baseRotation = m_effector->GetReferenceMatrix();

						ndFloat32 regularizer;
						ndFloat32 springConst;
						ndFloat32 damperConst;

						m_effector->GetLinearSpringDamper(regularizer, springConst, damperConst);
						m_effector->SetLinearSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);

						m_effector->GetAngularSpringDamper(regularizer, springConst, damperConst);
						m_effector->SetAngularSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
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

	dAdvancedIndustrialRobot(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
		,m_rootBody(nullptr)
		,m_leftGripper(nullptr)
		,m_rightGripper(nullptr)
		,m_effector(nullptr)
		,m_invDynamicsSolver()
		,m_baseRotation(dGetIdentityMatrix())
		,m_x(0.0f)
		,m_y(0.0f)
		,m_azimuth(0.0f)
		,m_gripperPosit(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
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
			m_effector = new ndIk6DofEffector(pivotMatrix, body0->GetAsBodyDynamic(), body1->GetAsBodyDynamic());
			m_effector->SetMode(true, true);

			ndFloat32 regularizer;
			ndFloat32 springConst;
			ndFloat32 damperConst;

			m_effector->GetLinearSpringDamper(regularizer, springConst, damperConst);
			m_effector->SetLinearSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);

			m_effector->GetAngularSpringDamper(regularizer, springConst, damperConst);
			m_effector->SetAngularSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
		}
	}

	~dAdvancedIndustrialRobot()
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

		ImGui::Text("position x");
		change = change | ImGui::SliderFloat("##x", &m_x, 0.0f, 5.0f);
		ImGui::Text("position y");
		change = change | ImGui::SliderFloat("##y", &m_y, -1.5f, 2.0f);
		ImGui::Text("azimuth");
		change = change | ImGui::SliderFloat("##azimuth", &m_azimuth, -180.0f, 180.0f);

		ImGui::Text("gripper");
		change = change | ImGui::SliderFloat("##gripperPosit", &m_gripperPosit, -0.2f, 0.03f);

		ImGui::Text("pitch");
		change = change | ImGui::SliderFloat("##pitch", &m_pitch, -180.0f, 180.0f);
		ImGui::Text("yaw");
		change = change | ImGui::SliderFloat("##yaw", &m_yaw, -180.0f, 180.0f);
		ImGui::Text("roll");
		change = change | ImGui::SliderFloat("##roll", &m_roll, -180.0f, 180.0f);

		if (change)
		{
			m_rootBody->SetSleepState(false);
		}
	}

	void PlaceEffector()
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

		targetMatrix = 
			dRollMatrix(90.0f * ndDegreeToRad) *
			dPitchMatrix(m_pitch * ndDegreeToRad) * dYawMatrix(m_yaw * ndDegreeToRad) * dRollMatrix(m_roll * ndDegreeToRad) * 
			dRollMatrix(-90.0f * ndDegreeToRad) * m_baseRotation;
		targetMatrix.m_posit = newPosit;

		m_effector->SetTargetMatrix(targetMatrix);
		m_leftGripper->SetOffsetPosit(m_gripperPosit);
		m_rightGripper->SetOffsetPosit(m_gripperPosit);
	}

	void Update(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::Update(world, timestep);

		ndSkeletonContainer* const skeleton = m_rootBody->GetSkeleton();
		dAssert(skeleton);

		m_invDynamicsSolver.SetMaxIterations(4);

		if (m_effector && !m_invDynamicsSolver.IsSleeping(skeleton))
		{
			PlaceEffector();
			m_invDynamicsSolver.AddEffector(skeleton, m_effector);
			m_invDynamicsSolver.Solve(skeleton, world, timestep);
		}
	}

	ndBodyDynamic* m_rootBody;
	ndJointSlider* m_leftGripper;
	ndJointSlider* m_rightGripper;
	ndIk6DofEffector* m_effector;
	ndIkSolver m_invDynamicsSolver;
	ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
	ndFixSizeArray<ndJointBilateralConstraint*, 16> m_jointArray;

	ndMatrix m_baseRotation;
	ndFloat32 m_x;
	ndFloat32 m_y;
	ndFloat32 m_azimuth;
	ndFloat32 m_gripperPosit;
	ndFloat32 m_pitch;
	ndFloat32 m_yaw;
	ndFloat32 m_roll;
};

D_CLASS_REFLECTION_IMPLEMENT_LOADER(dAdvancedIndustrialRobot);

static void RobotControlPanel(ndDemoEntityManager* const scene, void* const context)
{
	dAdvancedIndustrialRobot* const me = (dAdvancedIndustrialRobot*)context;
	me->ApplyControls(scene);
}

void ndAdvancedIndustrialRobot(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const robotEntity = scene->LoadFbxMesh("robot.fbx");

	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(dYawMatrix(-90.0f * ndDegreeToRad));
	dAdvancedIndustrialRobot* const robot = new dAdvancedIndustrialRobot(scene, robotEntity, matrix);
	scene->SetSelectedModel(robot);
	world->AddModel(robot);
	ndBodyDynamic* const root = robot->GetRoot();
	world->AddJoint (new ndJointFix6dof(root->GetMatrix(), root, world->GetSentinelBody()));

	scene->Set2DDisplayRenderFunction(RobotControlPanel, nullptr, robot);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new dAdvancedIndustrialRobot(scene, robotEntity, matrix));

	delete robotEntity;

	ndVector posit(matrix.m_posit);
	posit.m_x += 1.5f;
	posit.m_z += 1.5f;
	AddBox(scene, posit, 2.0f, 0.3f, 0.4f, 0.7f);
	AddBox(scene, posit, 1.0f, 0.3f, 0.4f, 0.7f);

	posit.m_x += 0.6f;
	posit.m_z += 0.2f;
	AddBox(scene, posit, 8.0f, 0.3f, 0.4f, 0.7f);
	AddBox(scene, posit, 4.0f, 0.3f, 0.4f, 0.7f);

	matrix.m_posit.m_x -= 6.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 6.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 45.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
