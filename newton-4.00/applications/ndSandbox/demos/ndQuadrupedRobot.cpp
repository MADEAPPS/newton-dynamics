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

class dQuadrupedRobotDefinition
{
	public:
	enum jointType
	{
		m_root,
		m_hinge,
		m_socket,
		m_effector
	};

	char m_boneName[32];
	jointType m_type;
	ndFloat32 m_mass;
};

static dQuadrupedRobotDefinition jointsDefinition[] =
{
	{ "root_Bone010", dQuadrupedRobotDefinition::m_root, 40.0f},

	{ "fr_thigh_Bone003", dQuadrupedRobotDefinition::m_socket, 4.0f},
	//{ "fr_knee_Bone004", dQuadrupedRobotDefinition::m_hinge, 2.5f},
	//
	//{ "fl_thigh_Bone008", dQuadrupedRobotDefinition::m_socket, 4.0f},
	//{ "fl_knee_Bone006", dQuadrupedRobotDefinition::m_hinge, 2.5f},
	//
	//{ "lb_thigh_Bone011", dQuadrupedRobotDefinition::m_socket, 4.0f},
	//{ "lb_knee_Bone012", dQuadrupedRobotDefinition::m_hinge, 2.5f},
	//
	//{ "rb_thigh_Bone014", dQuadrupedRobotDefinition::m_socket, 4.0f},
	//{ "rb_knee_Bone013", dQuadrupedRobotDefinition::m_hinge, 2.5f},
};

class dQuadrupedRobot : public ndModel
{
	public:
	D_CLASS_REFLECTION(dQuadrupedRobot);

	dQuadrupedRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
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
		ndDemoEntity* const entity = (ndDemoEntity*)robotMesh->CreateClone();
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
				const dQuadrupedRobotDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//dTrace(("name: %s\n", name));
					if (definition.m_type == dQuadrupedRobotDefinition::m_hinge)
					{
						dAssert(0);
						//ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						//MakeLegSphericalInretia(childBody);
						//m_bodyArray.PushBack(childBody);
						//
						//const ndMatrix pivotMatrix(dRollMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
						//ndJointIkHinge* const hinge = new ndJointIkHinge(pivotMatrix, childBody, parentBody);
						//hinge->EnableLimits(true, -60.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
						//m_jointArray.PushBack(hinge);
						//world->AddJoint(hinge);
						//parentBody = childBody;
					}
					else if (definition.m_type == dQuadrupedRobotDefinition::m_socket)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						MakeLegSphericalInretia(childBody);
						m_bodyArray.PushBack(childBody);
						
						const ndMatrix pivotMatrix(dYawMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
						ndIkJointSpherical* const socket = new ndIkJointSpherical(pivotMatrix, childBody, parentBody);
						socket->SetConeLimit(120.0f * ndDegreeToRad);
						socket->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);

						world->AddJoint(socket);
						parentBody = childBody;
					}
					else
					{
						dAssert(0);
						//ndMatrix pivotMatrix(childEntity->CalculateGlobalMatrix());
						//m_effector = new ndIk6DofEffector(pivotMatrix, parentBody, m_rootBody);
						//m_effector->SetMode(true, true);
						//
						//m_baseRotation = m_effector->GetReferenceMatrix();
						//
						//ndFloat32 regularizer;
						//ndFloat32 springConst;
						//ndFloat32 damperConst;
						//
						//m_effector->GetLinearSpringDamper(regularizer, springConst, damperConst);
						//m_effector->SetLinearSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
						//
						//m_effector->GetAngularSpringDamper(regularizer, springConst, damperConst);
						//m_effector->SetAngularSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
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

	dQuadrupedRobot(const ndLoadSaveBase::ndLoadDescriptor& desc)
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
			dAssert(0);
			//ndBodyLoaderCache::ndNode* const effectorBodyNode0 = desc.m_bodyMap->Find(xmlGetInt(endEffectorNode, "body0Hash"));
			//ndBodyLoaderCache::ndNode* const effectorBodyNode1 = desc.m_bodyMap->Find(xmlGetInt(endEffectorNode, "body1Hash"));
			//
			//ndBody* const body0 = (ndBody*)effectorBodyNode0->GetInfo();
			//ndBody* const body1 = (ndBody*)effectorBodyNode1->GetInfo();
			//dAssert(body1 == m_rootBody);
			//
			//const ndMatrix pivotMatrix(body0->GetMatrix());
			//m_effector = new ndIk6DofEffector(pivotMatrix, body0->GetAsBodyDynamic(), body1->GetAsBodyDynamic());
			//m_effector->SetMode(true, true);
			//
			//ndFloat32 regularizer;
			//ndFloat32 springConst;
			//ndFloat32 damperConst;
			//
			//m_effector->GetLinearSpringDamper(regularizer, springConst, damperConst);
			//m_effector->SetLinearSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
			//
			//m_effector->GetAngularSpringDamper(regularizer, springConst, damperConst);
			//m_effector->SetAngularSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);
		}
	}

	~dQuadrupedRobot()
	{
		if (m_effector && !m_effector->IsInWorld())
		{
			delete m_effector;
		}
	}

	void MakeLegSphericalInretia(ndBodyDynamic* const body)
	{
		ndVector massMatrix(body->GetMassMatrix());
		ndFloat32 sphericalInertia = 0.5f * dMax(dMax(massMatrix.m_x, massMatrix.m_y), massMatrix.m_z);
		massMatrix.m_x = sphericalInertia;
		massMatrix.m_y = sphericalInertia;
		massMatrix.m_z = sphericalInertia;
		body->SetMassMatrix(massMatrix);
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
		dAssert(0);
		//// apply target position collected by control panel
		//const ndMatrix aximuthMatrix(dYawMatrix(m_azimuth * ndDegreeToRad));
		//ndMatrix targetMatrix(m_effector->GetReferenceMatrix());
		//
		//// get the reference matrix in local space 
		//// (this is because the robot has a build rotation in the model) 
		//ndVector localPosit(targetMatrix.UnrotateVector(targetMatrix.m_posit));
		//
		//// add the local frame displacement)
		//localPosit.m_x += m_x;
		//localPosit.m_y += m_y;
		//localPosit = aximuthMatrix.RotateVector(localPosit);
		//
		//// take new position back to target space
		//const ndVector newPosit(targetMatrix.RotateVector(localPosit) + ndVector::m_wOne);
		//
		//targetMatrix = 
		//	dRollMatrix(90.0f * ndDegreeToRad) *
		//	dPitchMatrix(m_pitch * ndDegreeToRad) * dYawMatrix(m_yaw * ndDegreeToRad) * dRollMatrix(m_roll * ndDegreeToRad) * 
		//	dRollMatrix(-90.0f * ndDegreeToRad) * m_baseRotation;
		//targetMatrix.m_posit = newPosit;
		//
		//m_effector->SetTargetMatrix(targetMatrix);
		//m_leftGripper->SetOffsetPosit(m_gripperPosit);
		//m_rightGripper->SetOffsetPosit(m_gripperPosit);
	}

	void Update(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::Update(world, timestep);

return;
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

D_CLASS_REFLECTION_IMPLEMENT_LOADER(dQuadrupedRobot);



static void BuildBallSocket(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndFloat32 mass = 1.0f;
	ndFloat32 diameter = 1.0f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.125f, diameter * 0.125f, diameter * 1.0f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	ndMatrix matrix(dYawMatrix(-90.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_y = 1.2f;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	world->AddBody(body);
	scene->AddEntity(entity);

	body->SetOmega(matrix.RotateVector(ndVector(0.4f, 1.5f, 0.0f, 0.0f)));

	ndVector massMatrix(body->GetMassMatrix());
	ndFloat32 sphericalInertia = 0.5f * dMax(dMax(massMatrix.m_x, massMatrix.m_y), massMatrix.m_z);
	massMatrix.m_x = sphericalInertia;
	massMatrix.m_y = sphericalInertia;
	massMatrix.m_z = sphericalInertia;
	body->SetMassMatrix(massMatrix);

	matrix.m_posit -= matrix.m_front.Scale (0.5f);
	ndIkJointSpherical* const socket = new ndIkJointSpherical(matrix, body, world->GetSentinelBody());
	world->AddJoint(socket);
	mesh->Release();
}

static void RobotControlPanel(ndDemoEntityManager* const scene, void* const context)
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
	ndMatrix matrix(dYawMatrix(-90.0f * ndDegreeToRad));

	//dQuadrupedRobot* const robot = new dQuadrupedRobot(scene, robotEntity, matrix);
	//scene->SetSelectedModel(robot);
	//world->AddModel(robot);
	//ndBodyDynamic* const root = robot->GetRoot();
	//world->AddJoint (new ndJointFix6dof(root->GetMatrix(), root, world->GetSentinelBody()));
	//scene->Set2DDisplayRenderFunction(RobotControlPanel, nullptr, robot);

	BuildBallSocket(scene, matrix.m_posit);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new dQuadrupedRobot(scene, robotEntity, matrix));

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

	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.0f;
	matrix.m_posit.m_z += 0.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
