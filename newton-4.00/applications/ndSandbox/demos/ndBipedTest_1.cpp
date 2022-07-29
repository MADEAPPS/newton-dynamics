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

class ndAiBipedTest_1_Definition
{
	public:
	enum ndjointType
	{
		m_root,
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
	ndJointLimit m_jointLimits;
	ndFrameMatrix m_frameBasics;
};

static ndAiBipedTest_1_Definition mannequinDefinition[] =
{
	{ "pelvis", ndAiBipedTest_1_Definition::m_root, {}, {} },
	//{ "Bip001 Spine1", 1.0f,{ -30.0f, 30.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },
	//{ "Bip001 Head", 1.0f,{ -60.0f, 60.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },

	{ "rightLeg", ndAiBipedTest_1_Definition::m_spherical, { -45.0f, 45.0f, 80.0f }, { 0.0f, 90.0f, 0.0f } },
	{ "rightCalf", ndAiBipedTest_1_Definition::m_hinge, { 0.0f, 120.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
	{ "rightFoot", ndAiBipedTest_1_Definition::m_doubleHinge, { 0.0f, 0.0f, 60.0f }, { 0.0f, 90.0f, 0.0f } },
	{ "rightCalfEffector", ndAiBipedTest_1_Definition::m_effector, { 0.0f, 0.0f, 60.0f }, { 0.0f, 90.0f, 0.0f } },

	//{ "leftLeg", ndAiBipedTest_1_Definition::m_spherical, { -45.0f, 45.0f, 80.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "leftCalf", ndAiBipedTest_1_Definition::m_hinge, { 0.0f, 120.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
	//{ "leftFoot", ndAiBipedTest_1_Definition::m_doubleHinge, { 0.0f, 0.0f, 60.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "leftCalfEffector", ndAiBipedTest_1_Definition::m_effector,{ 0.0f, 0.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },

	{ "", ndAiBipedTest_1_Definition::m_root,{ 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f } },
};

class ndAiBipedTest_1 : public ndModel
{
	public:

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

	ndAiBipedTest_1(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location, ndAiBipedTest_1_Definition* const definition)
		:ndModel()
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const rootEntity = (ndDemoEntity*)robotMesh->CreateClone();
		scene->AddEntity(rootEntity);
		ndWorld* const world = scene->GetWorld();

		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);
		rootEntity->ResetMatrix(dGetIdentityMatrix());

		// find the floor location 
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 1.03f;

		rootEntity->ResetMatrix(matrix);

		ndFixSizeArray<ndBodyDynamic*, 64> bodies;
		// add the root body
		ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, nullptr);
		m_rootBody = rootBody;
		bodies.PushBack(m_rootBody);

		ndInt32 stack = 0;
		ndFixSizeArray<ndBodyDynamic*, 32> parentBones;
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		parentBones.SetCount(32);
		childEntities.SetCount(32);
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) 
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
			//dTrace(("name: %s\n", name));
			for (ndInt32 i = 0; definition[i].m_boneName[0]; ++i)
			{
				if (!strcmp(definition[i].m_boneName, name))
				{
					if (definition[i].m_type != ndAiBipedTest_1_Definition::m_effector)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBody);
						bodies.PushBack(childBody);

						// connect this body part to its parentBody with a robot joint
						ndJointBilateralConstraint* const joint = ConnectBodyParts(childBody, parentBody, definition[i]);
						world->AddJoint(joint);
						parentBody = childBody;
					}
					else
					{ 
						//ndMatrix pivotFrame(m_rootBody->GetMatrix());
						//ndMatrix effectorFrame(m_rootBody->GetMatrix());

						ndDemoEntityNotify* notify = (ndDemoEntityNotify*)parentBody->GetNotifyCallback();
						notify = (ndDemoEntityNotify*)notify->m_parentBody->GetNotifyCallback();

						//pivotFrame.m_posit = notify->GetBody()->GetMatrix().m_posit;
						ndMatrix pivotFrame(notify->GetBody()->GetMatrix());
						ndMatrix effectorFrame(pivotFrame);
						pivotFrame = ndRollMatrix(-90.0f * ndDegreeToRad) * pivotFrame;
						effectorFrame.m_posit = childEntity->CalculateGlobalMatrix().m_posit;

						ndMatrix swivelFrame(dGetIdentityMatrix());
						swivelFrame.m_front = (effectorFrame.m_posit - pivotFrame.m_posit).Normalize();
						swivelFrame.m_up = m_rootBody->GetMatrix().m_front;
						swivelFrame.m_right = (swivelFrame.m_front.CrossProduct(swivelFrame.m_up)).Normalize();
						swivelFrame.m_up = swivelFrame.m_right.CrossProduct(swivelFrame.m_front);

						ndFloat32 regularizer = 0.001f;
						ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame, pivotFrame, swivelFrame, parentBody, m_rootBody);
						effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
						effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);

						const ndVector kneePoint(childEntity->GetParent()->CalculateGlobalMatrix().m_posit);
						const ndVector dist0(effectorFrame.m_posit - kneePoint);
						const ndVector dist1(kneePoint - pivotFrame.m_posit);
						const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
						effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.999f);

						world->AddJoint(effector);

						ndEffectorInfo info(effector);
						info.m_x_mapper = ndParamMapper(-0.1f, workSpace * 0.8f);
						info.m_y_mapper = ndParamMapper(-ndPi, ndPi);
						info.m_z_mapper = ndParamMapper(-ndPi, ndPi);
						info.m_swivel_mapper = ndParamMapper(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
						m_effectors.PushBack(info);
					}
					break;
				}
			}
		
			for (ndDemoEntity* child = childEntity->GetChild(); child; child = child->GetSibling())
			{
				childEntities[stack] = child;
				parentBones[stack] = parentBody;
				stack++;
			}
		}
		
		SetModelMass(bodies, 100.0f);
	}

	void SetModelMass(const ndFixSizeArray<ndBodyDynamic*, 64>& bodies, ndFloat32 mass) const
	{
		ndFloat32 maxVolume = -1.0e10f;
		for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
		{
			ndFloat32 volume = bodies[i]->GetCollisionShape().GetVolume();
			maxVolume = ndMax(maxVolume, volume);
		}

		ndFloat32 totalVolume = 0.0f;
		for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
		{
			ndFloat32 volume = bodies[i]->GetCollisionShape().GetVolume();
			if (volume < 0.01f * maxVolume)
			{
				volume = 0.01f * maxVolume;
			}
			totalVolume += volume;
		}

		ndFloat32 density = mass / totalVolume;

		for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
		{
			ndBodyDynamic* const body = bodies[i];
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
				for (ndInt32 j = 0; j < 3; j++)
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
	
	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromChildren();
		dAssert(shape);

		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entityPart, parentBone));

		scene->GetWorld()->AddBody(body);
		delete shape;
		return body;
	}

	ndJointBilateralConstraint* ConnectBodyParts(ndBodyDynamic* const childBody, ndBodyDynamic* const parentBone, const ndAiBipedTest_1_Definition& definition)
	{
		ndMatrix matrix(childBody->GetMatrix());
		ndAiBipedTest_1_Definition::ndFrameMatrix frameAngle(definition.m_frameBasics);
		ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

		switch (definition.m_type)
		{
			case ndAiBipedTest_1_Definition::m_spherical:
			{
				ndIkJointSpherical* const joint = new ndIkJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBone);
				//ndAiBipedTest_1_Definition::ndJointLimit jointLimits(definition.m_jointLimits);
				//joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
				//joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				return joint;
			}

			case ndAiBipedTest_1_Definition::m_hinge:
			{
				ndIkJointHinge* const joint = new ndIkJointHinge(pinAndPivotInGlobalSpace, childBody, parentBone);

				ndAiBipedTest_1_Definition::ndJointLimit jointLimits(definition.m_jointLimits);
				joint->SetLimitState(true);
				joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				return joint;
			}

			case ndAiBipedTest_1_Definition::m_doubleHinge:
			{
				ndIkJointDoubleHinge* const joint = new ndIkJointDoubleHinge(pinAndPivotInGlobalSpace, childBody, parentBone);

				ndAiBipedTest_1_Definition::ndJointLimit jointLimits(definition.m_jointLimits);
				//joint->SetLimitState(true);
				//joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				return joint;
			}

			default:
				dAssert(0);
		}
		return nullptr;
	}

	void Debug(ndConstraintDebugCallback& context) const
	{
		for (ndInt32 i = 0; i < 1; ++i)
		{
			const ndEffectorInfo& info = m_effectors[i];
			ndJointBilateralConstraint* const joint = info.m_effector;
			joint->DebugJoint(context);
		}
	}

	void Update(ndWorld* const world, ndFloat32 timestep) 
	{
		ndModel::Update(world, timestep);

		const ndVector frontVector(m_rootBody->GetMatrix().m_front);
		for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
		{
			ndEffectorInfo& info = m_effectors[i];

			//ndMatrix matrix0;
			//ndMatrix matrix1;
			//info.m_effector->CalculateGlobalMatrix(matrix0, matrix1);
			//matrix0 = matrix0 * matrix1.Inverse();

			//ndFloat32 dist = info.m_x_mapper.Interpolate(info.m_x);

			ndFloat32 yawSin = ndSin(info.m_y_mapper.Interpolate(info.m_y));
			ndFloat32 yawCos = ndCos(info.m_y_mapper.Interpolate(info.m_y));

			ndFloat32 rollSin = ndSin(info.m_z_mapper.Interpolate(info.m_z));
			ndFloat32 rollCos = ndCos(info.m_z_mapper.Interpolate(info.m_z));

			ndVector posit(info.m_basePosition);
			ndFloat32 dist = posit.m_y + info.m_x_mapper.Interpolate(info.m_x);

			ndFloat32 x = dist * yawCos * rollCos;
			ndFloat32 y = dist * yawCos * rollSin;
			ndFloat32 z = -dist * yawSin;
			
			//posit.m_x += info.m_x_mapper.Interpolate(info.m_x);
			//posit.m_y += info.m_y_mapper.Interpolate(info.m_y);
			//posit.m_z += info.m_z_mapper.Interpolate(info.m_z);
			posit.m_y = x;
			posit.m_x = z;
			posit.m_z = y;
			info.m_effector->SetPosition(posit);

			ndMatrix swivelMatrix0;
			ndMatrix swivelMatrix1;
			info.m_effector->CalculateSwivelMatrices(swivelMatrix0, swivelMatrix1);
			const ndFloat32 angle = info.m_effector->CalculateAngle(frontVector, swivelMatrix1[1], swivelMatrix1[0]);
			info.m_effector->SetSwivelAngle(info.m_swivel_mapper.Interpolate(info.m_swivel) - angle);
		}
	}

	void ApplyControls(ndDemoEntityManager* const scene)
	{
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Control panel");

		ndEffectorInfo& info = m_effectors[0];

		bool change = false;
		ImGui::Text("distance");
		change = change | ImGui::SliderFloat("##x", &info.m_x, 0.0f, 1.0f);
		ImGui::Text("yaw");
		change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
		ImGui::Text("roll");
		change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);

		ImGui::Text("swivel");
		change = change | ImGui::SliderFloat("##swivel", &info.m_swivel, -1.0f, 1.0f);

		if (change)
		{
			m_rootBody->SetSleepState(false);
		}
	}

	static void ControlPanel(ndDemoEntityManager* const scene, void* const context)
	{
		ndAiBipedTest_1* const me = (ndAiBipedTest_1*)context;
		me->ApplyControls(scene);
	}

	void PostUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostUpdate(world, timestep);
	}

	void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostTransformUpdate(world, timestep);
	}

	ndBodyDynamic* m_rootBody;
	ndFixSizeArray<ndEffectorInfo, 4> m_effectors;
};

void BuildMannequin(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	matrix.m_posit.m_y = 0.5f;
	fbxDemoEntity* const robotMesh = scene->LoadFbxMesh("mannequin.fbx");

	ndMatrix entMatrix(ndYawMatrix(-90.0f * ndDegreeToRad) * robotMesh->GetRenderMatrix());
	robotMesh->ResetMatrix(entMatrix);

	ndWorld* const world = scene->GetWorld();
	ndAiBipedTest_1* const robot = new ndAiBipedTest_1(scene, robotMesh, matrix, mannequinDefinition);
	world->AddModel(robot);
	scene->Set2DDisplayRenderFunction(ndAiBipedTest_1::ControlPanel, nullptr, robot);

	world->AddJoint(new ndJointFix6dof(robot->m_rootBody->GetMatrix(), robot->m_rootBody, world->GetSentinelBody()));

	delete robotMesh;
}

void ndBipedTest (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndMatrix origin(dGetIdentityMatrix());
	origin.m_posit.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	origin.m_posit.m_x -= 20.0f;
	BuildMannequin(scene, origin.m_posit); 

	ndQuaternion rot;
	origin.m_posit.m_x -= 5.0f;
	origin.m_posit.m_y = 2.0f;
	scene->SetCameraMatrix(rot, origin.m_posit);
}
