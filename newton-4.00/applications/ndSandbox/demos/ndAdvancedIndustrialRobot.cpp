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
		m_effector
	};

	char m_boneName[32];
	ndFloat32 m_mass;
	ndFloat32 m_minLimit;
	ndFloat32 m_maxLimit;
	jointType m_type;
};

static dAdvancedRobotDefinition jointsDefinition[] =
{
	{ "base", 100.0f, 0.0f, 0.0f, dAdvancedRobotDefinition::m_root},
	{ "base_rotator", 50.0f, -1.0e10f, 1.0e10f, dAdvancedRobotDefinition::m_hinge },
	{ "arm_0", 5.0f, -140.0f * ndDegreeToRad, 1.0f * ndDegreeToRad, dAdvancedRobotDefinition::m_hinge },
	{ "arm_1", 5.0f, - 5.0f * ndDegreeToRad, 120.0f * ndDegreeToRad, dAdvancedRobotDefinition::m_hinge },
	//{ "arm_2", 5.0f, dAdvancedRobotDefinition::m_hinge },
	//{ "arm_3", 3.0f, dAdvancedRobotDefinition::m_hinge },
	//{ "arm_4", 2.0f, dAdvancedRobotDefinition::m_hinge },
	{ "effector", 0.0f, 0.0f, 0.0f, dAdvancedRobotDefinition::m_effector },
};

class dAdvancedIndustrialRobot : public ndModel
{
	public:
	D_CLASS_REFLECTION(dAdvancedIndustrialRobot);

	dAdvancedIndustrialRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
		,m_rootBody(nullptr)
		,m_effector(nullptr)
		,m_invDynamicsSolver()
		,m_x(0.0f)
		,m_y(0.0f)
		,m_azimuth(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
		,m_pitch0(0.0f)
		,m_yaw0(0.0f)
		,m_roll0(0.0f)
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
				const dAdvancedRobotDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//dTrace(("name: %s\n", name));
					if (definition.m_type == dAdvancedRobotDefinition::m_hinge)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						m_bodyArray.PushBack(childBody);

						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndJointHinge* const hinge = new ndJointHinge(pivotMatrix, childBody, parentBody);
						hinge->EnableLimits(true, definition.m_minLimit, definition.m_maxLimit);
						m_jointArray.PushBack(hinge);
						world->AddJoint(hinge);
						parentBody = childBody;
					}
					else
					{
						ndMatrix pivotMatrix(childEntity->CalculateGlobalMatrix());
						m_effector = new ndJointKinematicChain(pivotMatrix, parentBody, m_rootBody);
						m_effector->SetMode(true, false);

						ndVector euler0;
						ndVector euler1;
						m_effector->GetReferenceMatrix().CalcPitchYawRoll(euler0, euler1);
						m_pitch0 = euler0.m_x * ndRadToDegree;
						m_yaw0 = euler0.m_y * ndRadToDegree;
						m_roll0 = euler0.m_z * ndRadToDegree;

						ndFloat32 regularizer;
						ndFloat32 springConst;
						ndFloat32 damperConst;

						m_effector->GetLinearSpringDamper(regularizer, springConst, damperConst);
						m_effector->SetLinearSpringDamper(regularizer * 0.5f, springConst * 10.0f, damperConst * 10.0f);

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
		,m_effector(nullptr)
		,m_invDynamicsSolver()
		,m_x(0.0f)
		,m_y(0.0f)
		,m_azimuth(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
		,m_pitch0(0.0f)
		,m_yaw0(0.0f)
		,m_roll0(0.0f)
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

		ImGui::Text("position x");
		//change = change | ImGui::SliderFloat("##x", &m_x, 0.0f, 3.5f);
		change = change | ImGui::SliderFloat("##x", &m_x, 0.0f, 5.0f);
		ImGui::Text("position y");
		change = change | ImGui::SliderFloat("##y", &m_y, -1.5f, 2.0f);
		ImGui::Text("azimuth");
		change = change | ImGui::SliderFloat("##azimuth", &m_azimuth, -180.0f, 180.0f);

		ImGui::Text("pitch");
		change = change | ImGui::SliderFloat("##pitch", &m_pitch, -180.0f, 180.0f);
		ImGui::Text("yaw");
		change = change | ImGui::SliderFloat("##yaw", &m_yaw, -90.0f, 90.0f);
		ImGui::Text("roll");
		change = change | ImGui::SliderFloat("##roll", &m_roll, -180.0f, 180.0f);

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

		if (m_effector && !m_invDynamicsSolver.IsSleeping(skeleton))
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
				dPitchMatrix((m_pitch + m_pitch0) * ndDegreeToRad) *
				dYawMatrix((m_yaw + m_yaw0) * ndDegreeToRad) *
				dRollMatrix((m_roll + m_roll0) * ndDegreeToRad);
			targetMatrix.m_posit = newPosit;
			m_effector->SetTargetMatrix(targetMatrix);

			for (ndInt32 i = 0; i < m_jointArray.GetCount(); ++i)
			{
				ndJointHinge* const joint = (ndJointHinge*)m_jointArray[i];
				joint->EnableMotorAccel(false, ndFloat32(0.0f));
			}
			m_invDynamicsSolver.AddCloseLoopJoint(skeleton, m_effector);

			m_invDynamicsSolver.BeginSolve(skeleton, world, timestep);
			m_invDynamicsSolver.Solve();

			dTrace(("frame:\n"));

			ndInt32 maxPasses = 4;
			bool accelerationsAreValid = false;

			ndFixSizeArray<ndFloat32, 64> accelerations;
			accelerations.SetCount(m_jointArray.GetCount());
			while (!accelerationsAreValid && maxPasses)
			{
				maxPasses--;
				accelerationsAreValid = true;
				for (ndInt32 i = 0; i < m_jointArray.GetCount(); ++i)
				{
					ndJointHinge* const joint = (ndJointHinge*)m_jointArray[i];
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();

					const ndMatrix& invInertia0 = body0->GetInvInertiaMatrix();
					const ndMatrix& invInertia1 = body1->GetInvInertiaMatrix();

					const ndVector torque0(m_invDynamicsSolver.GetBodyTorque(body0));
					const ndVector torque1(m_invDynamicsSolver.GetBodyTorque(body1));
					const ndVector alpha0(invInertia0.RotateVector(torque0));
					const ndVector alpha1(invInertia1.RotateVector(torque1));

					ndFloat32 minLimit;
					ndFloat32 maxLimit;
					joint->GetLimits(minLimit, maxLimit);
					ndJacobianPair jacobian(joint->GetPinJacobian());
					ndFloat32 accel = (jacobian.m_jacobianM0.m_angular * alpha0 + jacobian.m_jacobianM1.m_angular * alpha1).AddHorizontal().GetScalar();
					ndFloat32 angle = joint->GetAngle() + joint->GetOmega() * timestep + accel * timestep * timestep;
					if (!joint->IsMotor() && ((angle < minLimit) || (angle > maxLimit)))
					{
maxPasses = 0;
						accelerationsAreValid = false;
						accel = -joint->GetOmega() / timestep;
						joint->EnableMotorAccel(true, accel);
					}
					accelerations[i] = accel;
					dTrace(("joint (%d %d)  accel=%f  omega=%f angle=%f\n", body0->GetId(), body1->GetId(), accel, joint->GetOmega(), joint->GetAngle() * ndRadToDegree));
				}

				if (!maxPasses)
				{
					for (ndInt32 i = 0; i < m_jointArray.GetCount(); ++i)
					{
						ndJointHinge* const joint = (ndJointHinge*)m_jointArray[i];
						accelerations[i] = -joint->GetOmega() / timestep;;
					}
					break;
				}
				else if (!accelerationsAreValid)
				{
					dAssert(0);
				}
			}

			for (ndInt32 i = 0; i < m_jointArray.GetCount(); ++i)
			{
				ndJointHinge* const joint = (ndJointHinge*)m_jointArray[i];
				joint->EnableMotorAccel(true, accelerations[i]);
			}

			m_invDynamicsSolver.EndSolve();
		}
	}

	ndBodyDynamic* m_rootBody;
	ndJointKinematicChain* m_effector;
	ndSkelIkSolver m_invDynamicsSolver;
	ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
	ndFixSizeArray<ndJointBilateralConstraint*, 16> m_jointArray;

	ndFloat32 m_x;
	ndFloat32 m_y;
	ndFloat32 m_azimuth;
	ndFloat32 m_pitch;
	ndFloat32 m_yaw;
	ndFloat32 m_roll;
	ndFloat32 m_pitch0;
	ndFloat32 m_yaw0;
	ndFloat32 m_roll0;
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

	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(ndQuaternion(), matrix.m_posit);
}
