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
#include "ndAnimationPose.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequenceBase.h"
#include "ndAnimationSequencePlayer.h"

class ndAiQuadrupedTest_3_Definition
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

static ndAiQuadrupedTest_3_Definition jointsDefinition[] =
{
	{ "root_Bone010", ndAiQuadrupedTest_3_Definition::m_root, 40.0f},

	{ "rb_thigh_Bone014", ndAiQuadrupedTest_3_Definition::m_spherical, 3.0f },
	{ "rb_knee_Bone013", ndAiQuadrupedTest_3_Definition::m_hinge, 2.0f },
	{ "rb_effector_Bone009", ndAiQuadrupedTest_3_Definition::m_effector , 0.0f, 0.0f },
	
	{ "lb_thigh_Bone011", ndAiQuadrupedTest_3_Definition::m_spherical, 3.0f },
	{ "lb_knee_Bone012", ndAiQuadrupedTest_3_Definition::m_hinge, 2.0f },
	{ "lb_effector_Bone010", ndAiQuadrupedTest_3_Definition::m_effector , 0.0f, 0.5f },
	
	{ "fr_thigh_Bone003", ndAiQuadrupedTest_3_Definition::m_spherical, 3.0f },
	{ "fr_knee_Bone004", ndAiQuadrupedTest_3_Definition::m_hinge, 2.0f },
	{ "fr_effector_Bone005", ndAiQuadrupedTest_3_Definition::m_effector , 0.0f, 0.75f },
	
	{ "fl_thigh_Bone008", ndAiQuadrupedTest_3_Definition::m_spherical, 3.0f },
	{ "fl_knee_Bone006", ndAiQuadrupedTest_3_Definition::m_hinge, 2.0f },
	{ "fl_effector_Bone007", ndAiQuadrupedTest_3_Definition::m_effector , 0.0f, 0.25f },
};

class ndAiQuadrupedTest_3 : public ndModel
{
	public:
	#define D_SAMPLES_COUNT 128

	D_CLASS_REFLECTION(ndAiQuadrupedTest_3);

	class ndParamMapper
	{
		public:
		ndParamMapper()
			:m_x0(0.0f)
			,m_scale(0.0f)
		{
		}

		ndParamMapper(ndFloat32 x0, ndFloat32 x1)
			:m_x0(x0 + (x1 - x0) * 0.5f)
			,m_scale((x1 - x0) * 0.5f)
		{
		}

		ndFloat32 Interpolate(const ndFloat32 t)
		{
			return m_x0 + m_scale * t;
		}

		ndFloat32 m_x0;
		ndFloat32 m_scale;
	};

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

	ndAiQuadrupedTest_3(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
		,m_rootBody(nullptr)
		,m_effectors()
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

		matrix.m_posit.m_y += 0.71f;
		rootEntity->ResetMatrix(matrix);

		// add the root body
		m_rootBody = CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr);
		
		ndFixSizeArray<ndBodyDynamic*, 32> parentBone;
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling())
		{
			childEntities[stack] = child;
			parentBone[stack] = m_rootBody;
			stack++;
		}

		const ndMatrix referenceFrame = rootEntity->Find("referenceFrame")->CalculateGlobalMatrix();
		const ndInt32 definitionCount = ndInt32 (sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (stack) 
		{
			stack--;
			ndBodyDynamic* parentBody = parentBone[stack];
			ndDemoEntity* const childEntity = childEntities[stack];

			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; ++i) 
			{
				const ndAiQuadrupedTest_3_Definition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//dTrace(("name: %s\n", name));
					if (definition.m_type == ndAiQuadrupedTest_3_Definition::m_hinge)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						const ndMatrix pivotMatrix(ndRollMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
						ndIkJointHinge* const hinge = new ndIkJointHinge(pivotMatrix, childBody, parentBody);
						hinge->SetLimitState(true);
						hinge->SetLimits(-30.0f * ndDegreeToRad, 120.0f * ndDegreeToRad);
						world->AddJoint(hinge);
						parentBody = childBody;
					}
					else if (definition.m_type == ndAiQuadrupedTest_3_Definition::m_spherical)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
						const ndMatrix pivotMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * childBody->GetMatrix());
						ndIkJointSpherical* const socket = new ndIkJointSpherical(pivotMatrix, childBody, parentBody);
						//socket->SetConeLimit(120.0f * ndDegreeToRad);
						//socket->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);

						world->AddJoint(socket);
						parentBody = childBody;
					}
					else
					{
						char refName[256];
						sprintf(refName, "%sreference", name);
						ndAssert(rootEntity->Find(refName));

						ndMatrix pivotFrame(referenceFrame);
						ndMatrix effectorFrame(referenceFrame);
						effectorFrame.m_posit = childEntity->CalculateGlobalMatrix().m_posit;
						pivotFrame.m_posit = rootEntity->Find(refName)->CalculateGlobalMatrix().m_posit;

						ndMatrix swivelFrame(ndGetIdentityMatrix());
						swivelFrame.m_front = (effectorFrame.m_posit - pivotFrame.m_posit).Normalize();
						swivelFrame.m_up = referenceFrame.m_front;
						swivelFrame.m_right = (swivelFrame.m_front.CrossProduct(swivelFrame.m_up)).Normalize();
						swivelFrame.m_up = swivelFrame.m_right.CrossProduct(swivelFrame.m_front);

						ndFloat32 regularizer = 0.001f;
						ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame, pivotFrame, swivelFrame, parentBody, m_rootBody);
						effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
						effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);

						const ndVector elbowPoint(childEntity->GetParent()->CalculateGlobalMatrix().m_posit);
						const ndVector dist0(effectorFrame.m_posit - elbowPoint);
						const ndVector dist1(elbowPoint - pivotFrame.m_posit);
						const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
						effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.95f);

						world->AddJoint(effector);

						ndEffectorInfo info(effector);
						info.m_x_mapper = ndParamMapper(-0.2f, 0.2f);
						info.m_y_mapper = ndParamMapper(-0.06f, 0.4f);
						info.m_z_mapper = ndParamMapper(-0.1f, 0.1f);
						info.m_swivel_mapper = ndParamMapper(-20.0f * ndDegreeToRad, 20.0f * ndDegreeToRad);
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
	}

	ndAiQuadrupedTest_3(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndModel(ndLoadSaveBase::ndLoadDescriptor(desc))
		,m_rootBody(nullptr)
		,m_effectors()
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
		ndBodyLoaderCache::ndNode* const rootBodyNode = desc.m_bodyMap->Find(xmlGetInt(modelRootNode, "rootBodyHash"));
		ndBody* const rootbody = (ndBody*)rootBodyNode->GetInfo();
		m_rootBody = rootbody->GetAsBodyDynamic();
		
		// load effector joint
		const nd::TiXmlNode* const endEffectorNode = modelRootNode->FirstChild("endEffector");
		if (xmlGetInt(endEffectorNode, "hasEffector"))
		{
			ndAssert(0);
		}
	}

	~ndAiQuadrupedTest_3()
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
		//for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		for (ndInt32 i = 0; i < 1; ++i)
		{
			const ndEffectorInfo& info = m_effectors[i];
			ndJointBilateralConstraint* const joint = info.m_effector;
			joint->DebugJoint(context);

			//ndMatrix swivelMatrix0;
			//ndMatrix swivelMatrix1;
			//info.m_effector->CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);

			//ndVector posit1(swivelMatrix1.m_posit);
			//posit1.m_y += 1.0f;
			//context.DrawLine(swivelMatrix1.m_posit, posit1, ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f)));
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

		ndEffectorInfo& info = m_effectors[0];

		bool change = false;
		ImGui::Text("position x");
		change = change | ImGui::SliderFloat("##x", &info.m_x, -1.0f, 1.0f);
		ImGui::Text("position y");
		change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
		ImGui::Text("position z");
		change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);

		ImGui::Text("swivel");
		change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);

		if (change)
		{
			m_rootBody->SetSleepState(false);

			for (ndInt32 i = 1; i < m_effectors.GetCount(); ++i)
			{
				m_effectors[i].m_x = info.m_x;
				m_effectors[i].m_y = info.m_y;
				m_effectors[i].m_z = info.m_z;
				m_effectors[i].m_swivel = info.m_swivel;
			}
		}
	}

	void Update(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::Update(world, timestep);

		const ndVector frontVector(m_rootBody->GetMatrix().m_front.Scale (-1.0f));
		for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		{
			ndEffectorInfo& info = m_effectors[i];
			ndVector posit(info.m_basePosition);
			posit.m_x += info.m_x_mapper.Interpolate(info.m_x);
			posit.m_y += info.m_y_mapper.Interpolate(info.m_y);
			posit.m_z += info.m_z_mapper.Interpolate(info.m_z);
			info.m_effector->SetPosition(posit);

			ndMatrix swivelMatrix0;
			ndMatrix swivelMatrix1;
			info.m_effector->CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);
			const ndFloat32 angle = info.m_effector->CalculateAngle(frontVector, swivelMatrix1[1], swivelMatrix1[0]);
			info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel) - angle);
		}
	}

	static void ControlPanel(ndDemoEntityManager* const scene, void* const context)
	{
		ndAiQuadrupedTest_3* const me = (ndAiQuadrupedTest_3*)context;
		me->ApplyControls(scene);
	}

	ndBodyDynamic* m_rootBody;
	ndFixSizeArray<ndEffectorInfo, 4> m_effectors;
};
D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndAiQuadrupedTest_3);

void ndQuadrupedTest_3(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	fbxDemoEntity* const robotEntity = scene->LoadFbxMesh("spot.fbx");

	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

	ndAiQuadrupedTest_3* const robot0 = new ndAiQuadrupedTest_3(scene, robotEntity, matrix);
	scene->SetSelectedModel(robot0);
	world->AddModel(robot0);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//ndAiQuadrupedTest_3* const robot1 = new ndAiQuadrupedTest_3(scene, robotEntity, matrix);
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
	scene->Set2DDisplayRenderFunction(ndAiQuadrupedTest_3::ControlPanel, nullptr, robot0);

	matrix.m_posit.m_x -= 5.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
