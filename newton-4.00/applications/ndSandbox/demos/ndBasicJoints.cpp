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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoSplinePathMesh.h"
#include "ndDemoInstanceEntity.h"

class ndFollowSplinePath : public ndJointFollowPath
{
	public:
	D_CLASS_REFLECTION(ndFollowSplinePath);
	ndFollowSplinePath(const ndMatrix& pinAndPivotFrame, ndBodyDynamic* const child, ndBodyDynamic* const pathBody)
		:ndJointFollowPath(pinAndPivotFrame, child, pathBody)
	{
	}

	ndFollowSplinePath(const ndLoadSaveBase::ndLoadDescriptor& desc)
		:ndJointFollowPath(ndLoadSaveBase::ndLoadDescriptor(desc))
	{
//		dAssert(0);
	}

	void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
	{
		nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
		desc.m_rootNode->LinkEndChild(childNode);
		childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
		ndJointFollowPath::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));
	}

	void GetPointAndTangentAtLocation(const ndVector& location, ndVector& positOut, ndVector& tangentOut) const
	{
		ndDemoEntity* const pathEntity = (ndDemoEntity*)GetBody1()->GetNotifyCallback()->GetUserData();
		ndDemoSplinePathMesh* const mesh = (ndDemoSplinePathMesh*)pathEntity->GetMesh();
		const ndBezierSpline& spline = mesh->m_curve;
		
		ndMatrix matrix(GetBody1()->GetMatrix());
		
		ndVector p(matrix.UntransformVector(location));
		ndBigVector point;
		ndFloat64 knot = spline.FindClosestKnot(point, p, 4);
		ndBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale(1.0 / ndSqrt(tangent.DotProduct(tangent).GetScalar()));
		positOut = matrix.TransformVector(point);
		tangentOut = tangent;
	}
};
D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndFollowSplinePath)

static ndBodyDynamic* MakePrimitive(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndShapeInstance& shape, ndDemoMesh* const mesh, ndFloat32 mass)
{
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
	return body;
}

static void BuildBallSocket(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndFloat32 mass = 1.0f;
	ndFloat32 diameter = 0.5f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.25f, diameter * 0.25f, diameter * 1.0f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndPhysicsWorld* const world = scene->GetWorld();
	ndMatrix matrix(dRollMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	{
		class ndJointSphericalMotor : public ndJointSpherical
		{
			public:
			ndJointSphericalMotor(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
				:ndJointSpherical(pinAndPivotFrame, child, parent)
				,m_rollAngle(0.0f)
				,m_pitchAngle(0.0f)
				,m_rollOmega(5.0f)
				,m_pitchOmega(6.0f)
			{
				ndFloat32 friction = 10.0f;
				ndFloat32 spring = 1500.0f;
				ndFloat32 regularizer = 0.01f;
				SetAsSpringDamper(regularizer, spring, friction);
			}

			void JacobianDerivative(ndConstraintDescritor& desc)
			{
				m_rollAngle = ndFmod(m_rollAngle + m_rollOmega * desc.m_timestep, 2.0f * ndPi);
				m_pitchAngle = ndFmod(m_pitchAngle + m_pitchOmega * desc.m_timestep, 2.0f * ndPi);

				const ndMatrix rotaion(dPitchMatrix(m_pitchAngle) * dRollMatrix(m_rollAngle));
				SetOffsetRotation(rotaion);
				ndJointSpherical::JacobianDerivative(desc);
			}

			ndFloat32 m_rollAngle;
			ndFloat32 m_pitchAngle;
			ndFloat32 m_rollOmega;
			ndFloat32 m_pitchOmega;
		};


		// add a spherical motor.
		matrix.m_posit.m_y = floor.m_y;
		matrix.m_posit.m_y += diameter;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndMatrix pinAlign(dRollMatrix(180.0f * ndDegreeToRad));
		ndMatrix bodyMatrix0(pinAlign * body->GetMatrix());
		bodyMatrix0.m_posit.m_y += diameter * 0.5f + diameter * 0.25f;
		ndBodyKinematic* const fixBody = world->GetSentinelBody();
		ndJointSpherical* const joint = new ndJointSphericalMotor(bodyMatrix0, body, fixBody);
		world->AddJoint(joint);
	}

	if (0)
	{
		const ndInt32 count = 6;
		// add flexible chain with spring damper.
		matrix.m_posit.m_z -= 2.0f;
		matrix.m_posit.m_y = floor.m_y;
		ndBodyDynamic* array[count];
		for (ndInt32 i = 0; i < count; i++)
		{
			matrix.m_posit.m_y += diameter;
			ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
			array[i] = body;
		}

		ndFloat32 friction = 10.0f;
		ndFloat32 spring = 1500.0f;
		ndFloat32 regularizer = 0.01f;

		ndMatrix pinAlign(dRollMatrix(180.0f * ndDegreeToRad));
		for (ndInt32 i = 1; i < count; i++)
		{
			ndMatrix bodyMatrix0(array[i - 1]->GetMatrix());
			ndMatrix bodyMatrix1(array[i - 0]->GetMatrix());
			ndMatrix pinMatrix(pinAlign * bodyMatrix0);
			pinMatrix.m_posit = (bodyMatrix0.m_posit + bodyMatrix1.m_posit).Scale(0.5f);
			ndJointSpherical* const joint = new ndJointSpherical(pinMatrix, array[i - 1], array[i - 0]);
			joint->SetAsSpringDamper(regularizer, spring, friction);
			//joint->SetConeLimit(60.0f * ndDegreeToRad);
			//joint->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
			world->AddJoint(joint);
		}

		ndMatrix bodyMatrix0(pinAlign * array[count - 1]->GetMatrix());
		bodyMatrix0.m_posit.m_y += diameter * 0.5f + diameter * 0.25f;
		ndBodyKinematic* const fixBody = world->GetSentinelBody();
		ndJointSpherical* const joint = new ndJointSpherical(bodyMatrix0, array[count - 1], fixBody);
		joint->SetAsSpringDamper(regularizer, spring, friction);
		//joint->SetConeLimit(60.0f * ndDegreeToRad);
		//joint->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
		world->AddJoint(joint);
	}

	if (0)
	{
		// add a chain with viscous friction.
		const ndInt32 count = 6;
		matrix.m_posit.m_z -= 2.0f;
		matrix.m_posit.m_y = floor.m_y;
		ndBodyDynamic* array[count];
		for (ndInt32 i = 0; i < count; i++)
		{
			matrix.m_posit.m_y += diameter;
			ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
			//ndVector inertia(body->GetMassMatrix());
			//ndFloat32 maxI(dMax(dMax(inertia.m_x, inertia.m_z), inertia.m_z));
			//inertia.m_x = maxI;
			//inertia.m_y = maxI;
			//inertia.m_z = maxI;
			//body->SetMassMatrix(inertia);
			array[i] = body;
		}

		ndFloat32 friction = 10.0f;
		ndFloat32 regularizer = 0.1f;

		ndMatrix pinAlign(dRollMatrix(180.0f * ndDegreeToRad));
		for (ndInt32 i = 1; i < count; i++)
		{
			ndMatrix bodyMatrix0(array[i - 1]->GetMatrix());
			ndMatrix bodyMatrix1(array[i - 0]->GetMatrix());
			ndMatrix pinMatrix(pinAlign * bodyMatrix0);
			pinMatrix.m_posit = (bodyMatrix0.m_posit + bodyMatrix1.m_posit).Scale(0.5f);
			ndJointSpherical* const joint = new ndJointSpherical(pinMatrix, array[i - 1], array[i - 0]);
			joint->SetAsSpringDamper(regularizer, ndFloat32(0.0f), friction);
			joint->SetConeLimit(60.0f * ndDegreeToRad);
			joint->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
			world->AddJoint(joint);
		}

		ndMatrix bodyMatrix0(pinAlign * array[count - 1]->GetMatrix());
		bodyMatrix0.m_posit.m_y += diameter * 0.5f + diameter * 0.25f;
		ndBodyKinematic* const fixBody = world->GetSentinelBody();
		ndJointSpherical* const joint = new ndJointSpherical(bodyMatrix0, array[count - 1], fixBody);
		joint->SetAsSpringDamper(regularizer, ndFloat32(0.0f), friction);
		joint->SetConeLimit(60.0f * ndDegreeToRad);
		joint->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
		world->AddJoint(joint);
	}

	mesh->Release();
}

static void BuildRollingFriction(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndMatrix matrix(dRollMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	
	ndShapeInstance shape2(new ndShapeSphere(diameter * 0.5f));
	ndDemoMesh* const mesh2 = new ndDemoMesh("shape2", scene->GetShaderCache(), &shape2, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	matrix.m_posit.m_y += 5.0f;
	
	ndPhysicsWorld* const world = scene->GetWorld();
	ndVector posit(matrix.m_posit);
	for (ndInt32 i = 0; i < 8; i++)
	{
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape2, mesh2, mass);
		ndJointBilateralConstraint* const joint = new ndJointDryRollingFriction(body, world->GetSentinelBody(), 0.5f);
		world->AddJoint(joint);
		posit.m_y += diameter * 1.5f;
		matrix.m_posit = posit + ndVector(dGaussianRandom(0.01f), 0.0f, dGaussianRandom(0.01f), 0.0f);
	}

	mesh2->Release();
}

static void BuildSlider(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndShapeInstance shape(new ndShapeBox(diameter, diameter, diameter));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// spring damper slider with limits
		matrix.m_posit.m_y += 2.0f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointSlider* const joint = new ndJointSlider(dYawMatrix(90.0f * ndDegreeToRad) * matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 100.0f, 5.0f);
		joint->SetLimits(-1.0f, 1.0f);
		world->AddJoint(joint);
	}

	{
		// viscous damper slider with limits
		matrix.m_posit.m_y += 1.2f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointSlider* const joint = new ndJointSlider(dYawMatrix(90.0f * ndDegreeToRad) * matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 0.0f, 10.0f);
		joint->SetLimits(-1.0f, 1.0f);
		world->AddJoint(joint);
	}

	{
		class ndJointSliderOscillator : public ndJointSlider
		{
			public:
			ndJointSliderOscillator(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
				:ndJointSlider(pinAndPivotFrame, child, parent)
				,m_angle(0.0f)
			{

			}

			void JacobianDerivative(ndConstraintDescritor& desc)
			{
				m_angle += ndFmod(5.0f * desc.m_timestep, 2.0f * ndPi);
				ndFloat32 dist = 0.9f * ndSin(m_angle);
				SetOffsetPosit(dist);
				ndJointSlider::JacobianDerivative(desc);
			}

			ndFloat32 m_angle;
		};

		// proportional derivative slider oscillator with limits
		matrix.m_posit.m_y += 1.2f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointSlider* const joint = new ndJointSliderOscillator(dYawMatrix(90.0f * ndDegreeToRad) * matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 500.0f, 10.0f);
		joint->SetLimits(-1.0f, 1.0f);
		world->AddJoint(joint);
	}

	mesh->Release();
}

static void BuildHinge(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndShapeInstance shape(new ndShapeBox(diameter, diameter, diameter));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndMatrix matrix(dRollMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// spring damper and limits
		matrix.m_posit.m_y += 2.0f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointHinge* const joint = new ndJointHinge(matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 20.0f, 1.0f);
		joint->SetLimits(-3.0f, 3.0f);
		world->AddJoint(joint);
	}

	{
		// viscous friction and limits
		matrix.m_posit.m_y += 1.2f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointHinge* const joint = new ndJointHinge(matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 0.0f, 1.0f);
		joint->SetLimits(-10.0f, 15.0f);
		world->AddJoint(joint);
	}

	{
		class ndJointHingeOscillator : public ndJointHinge
		{
			public:
			ndJointHingeOscillator(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
				:ndJointHinge(pinAndPivotFrame, child, parent)
				,m_angle(0.0f)
			{
			}

			void JacobianDerivative(ndConstraintDescritor& desc)
			{
				m_angle += ndFmod(5.0f * desc.m_timestep, 2.0f * ndPi);
				ndFloat32 dist = 150.0f * ndDegreeToRad * ndSin(m_angle);
				SetOffsetAngle(dist);
				ndJointHinge::JacobianDerivative(desc);
			}

			ndFloat32 m_angle;
		};

		// proportional derivative hinge oscillator with limits
		matrix.m_posit.m_y += 1.2f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointHinge* const joint = new ndJointHingeOscillator(matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 1500.0f, 10.0f);
		world->AddJoint(joint);
	}

	{
		class ndJointHingeMotor : public ndJointHinge
		{
			public:
			ndJointHingeMotor(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
				:ndJointHinge(pinAndPivotFrame, child, parent)
				,m_speed(10.0f)
			{
			}

			void JacobianDerivative(ndConstraintDescritor& desc)
			{
				ndFloat32 angle = GetAngle();
				SetOffsetAngle(angle + m_speed * desc.m_timestep);
				ndJointHinge::JacobianDerivative(desc);
			}
			ndFloat32 m_speed;
		};

		// proportional derivative hinge motor with limits
		matrix.m_posit.m_y += 1.2f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointHinge* const joint = new ndJointHingeMotor(matrix, body, fixBody);
		joint->SetAsSpringDamper(0.1f, 1500.0f, 10.0f);
		world->AddJoint(joint);
	}


	mesh->Release();
}

static void BuildDoubleHinge(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndShapeInstance shape(new ndShapeCylinder(diameter, diameter, diameter * 0.5f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndMatrix matrix(dYawMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	matrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();

	{
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		body->SetOmega(ndVector(0.0f, 10.0f, 20.0f, 0.0f));
		
		ndJointDoubleHinge* const joint = new ndJointDoubleHinge(matrix, body, fixBody);
		world->AddJoint(joint);
	}

	{
		class ndJointDoubleHingeMotor : public ndJointDoubleHinge
		{
			public:
				ndJointDoubleHingeMotor(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
				:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
				,m_angle(0.0f)
				,m_speed(10.0f)
			{
			}

			void JacobianDerivative(ndConstraintDescritor& desc)
			{
				m_angle += ndFmod(5.0f * desc.m_timestep, 2.0f * ndPi);
				ndFloat32 dist = 150.0f * ndDegreeToRad * ndSin(m_angle);
				SetOffsetAngle0(dist);

				ndFloat32 angle = GetAngle1();
				SetOffsetAngle1(angle + m_speed * desc.m_timestep);

				ndJointDoubleHinge::JacobianDerivative(desc);
			}

			ndFloat32 m_angle;
			ndFloat32 m_speed;
		};

		// proportional derivative double hinge motor
		matrix.m_posit.m_z += 1.8f;
		ndBodyDynamic* const body = MakePrimitive(scene, matrix, shape, mesh, mass);
		ndJointDoubleHinge* const joint = new ndJointDoubleHingeMotor(matrix, body, fixBody);
		joint->SetAsSpringDamper0(0.1f, 1500.0f, 10.0f);
		joint->SetAsSpringDamper1(0.1f, 1500.0f, 10.0f);
		world->AddJoint(joint);
	}
	
	mesh->Release();
}

void BuildFixDistanceJoints(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndShapeInstance shape(new ndShapeSphere(0.25f));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "earthmap.tga", "earthmap.tga", "earthmap.tga");

	ndMatrix matrix(dGetIdentityMatrix());

	ndBodyDynamic* bodies[8];

	matrix.m_posit = origin + ndVector(0.0f, 4.0f, 0.0f, 0.0f);
	bodies[0] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(2.0f, 4.0f, 0.0f, 0.0f);
	bodies[1] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(2.0f, 4.0f, 2.0f, 0.0f);
	bodies[2] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(0.0f, 4.0f, 2.0f, 0.0f);
	bodies[3] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(0.0f, 6.0f, 0.0f, 0.0f);
	bodies[4] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(2.0f, 6.0f, 0.0f, 0.0f);
	bodies[5] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(2.0f, 6.0f, 2.0f, 0.0f);
	bodies[6] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	matrix.m_posit = origin + ndVector(0.0f, 6.0f, 2.0f, 0.0f);
	bodies[7] = MakePrimitive(scene, matrix, shape, mesh, 5.0f);

	ndWorld* world = scene->GetWorld();
	for (ndInt32 i = 0; i < 8; i++)
	{
		ndBodyDynamic* const body0 = bodies[i];
		for (ndInt32 j = i + 1; j < 8; j++)
		{
			ndBodyDynamic* const body1 = bodies[j];
			world->AddJoint(new ndJointFixDistance(body0->GetMatrix().m_posit, body1->GetMatrix().m_posit, body0, body1));
		}
	}

	mesh->Release();
}

static void BuildGear(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndShapeInstance shape(new ndShapeBox(diameter, diameter, diameter));
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");

	ndMatrix matrix(dRollMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndPhysicsWorld* const world = scene->GetWorld();

	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y;

	matrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	ndBodyDynamic* const body0 = MakePrimitive(scene, matrix, shape, mesh, mass);

	matrix.m_posit.m_y += diameter * 1.5f;
	ndBodyDynamic* const body1 = MakePrimitive(scene, matrix, shape, mesh, mass);

	world->AddJoint(new ndJointHinge(matrix, body0, fixBody));
	world->AddJoint(new ndJointHinge(matrix, body1, fixBody));

	ndVector pin(matrix.m_front);
	world->AddJoint(new ndJointGear(4.0f, pin, body0, pin, body1));

	mesh->Release();
}

static void AddPathFollow(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;

	// create a Bezier Spline path for AI car to drive
	ndShapeInstance box(new ndShapeBox(1.0f, 1.0f, 1.0f));
	ndBodyDynamic* const pathBody = MakePrimitive(scene, matrix, box, nullptr, 0.0f);
	ndDemoEntity* const rollerCosterPath = (ndDemoEntity*)pathBody->GetNotifyCallback()->GetUserData();

	ndBezierSpline spline;
	ndFloat64 knots[] = { 0.0f, 1.0f / 5.0f, 2.0f / 5.0f, 3.0f / 5.0f, 4.0f / 5.0f, 1.0f };

	ndBigVector control[] =
	{
		ndBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
		ndBigVector(150.0f - 100.0f, 10.0f, 150.0f - 250.0f, 1.0f),
		ndBigVector(175.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		ndBigVector(200.0f - 100.0f, 70.0f, 250.0f - 250.0f, 1.0f),
		ndBigVector(215.0f - 100.0f, 20.0f, 250.0f - 250.0f, 1.0f),
		ndBigVector(150.0f - 100.0f, 50.0f, 350.0f - 250.0f, 1.0f),
		ndBigVector(50.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		ndBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
	};

	spline.CreateFromKnotVectorAndControlPoints(3, sizeof(knots) / sizeof(knots[0]), knots, control);

	ndDemoSplinePathMesh* const mesh = new ndDemoSplinePathMesh(spline, scene->GetShaderCache(), 500);
	rollerCosterPath->SetMesh(mesh, dGetIdentityMatrix());

	mesh->SetVisible(true);
	//mesh->SetRenderResolution(500);
	mesh->Release();

	const ndInt32 count = 32;

	ndBigVector point0;
	ndVector positions[count + 1];
	ndFloat64 knot = spline.FindClosestKnot(point0, ndBigVector(ndVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f)), 4);
	positions[0] = point0;
	for (ndInt32 i = 0; i < count; i++) 
	{
		ndBigVector point1;
		ndBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale(1.0 / ndSqrt(tangent.DotProduct(tangent).GetScalar()));
		knot = spline.FindClosestKnot(point1, ndBigVector(point0 + tangent.Scale(2.0f)), 4);
		point0 = point1;
		positions[i + 1] = point1;
	}

	ndMatrix pathBodyMatrix (pathBody->GetMatrix());
	ndFloat32 attachmentOffset = 0.8f;

	ndShapeInstance shape(new ndShapeChamferCylinder(0.5f, 0.5f));
	ndDemoMeshIntance* const instanceMesh = new ndDemoMeshIntance("shape", scene->GetShaderCache(), &shape, "wood_0.tga", "wood_0.tga", "wood_0.tga");
	ndDemoInstanceEntity* const rootEntity = new ndDemoInstanceEntity(instanceMesh);
	scene->AddEntity(rootEntity);
	instanceMesh->Release();

	ndBodyDynamic* bodies[count];
	ndPhysicsWorld* const world = scene->GetWorld();
	for (ndInt32 i = 0; i < count; i++) 
	{
		ndVector location0(positions[i + 0].m_x, positions[i + 0].m_y, positions[i + 0].m_z, 0.0);
		ndVector location1(positions[i + 1].m_x, positions[i + 1].m_y, positions[i + 1].m_z, 0.0);
		
		location0 = pathBodyMatrix.TransformVector(location0);
		location1 = pathBodyMatrix.TransformVector(location1);
		
		ndVector dir(location1 - location0);
		dir.m_w = 0.0f;
		matrix.m_front = dir.Scale(1.0f / ndSqrt(dir.DotProduct(dir).GetScalar()));
		matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
		matrix.m_right = matrix.m_right.Scale(1.0f / ndSqrt(matrix.m_right.DotProduct(matrix.m_right).GetScalar()));
		matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
		matrix.m_posit = pathBodyMatrix.TransformVector(ndVector(positions[i].m_x, positions[i].m_y - attachmentOffset, positions[i].m_z, 1.0));
		ndMatrix matrix1(dYawMatrix(0.5f * ndPi) * matrix);
		
		ndBodyDynamic* const body = new ndBodyDynamic();
		ndDemoEntity* const entity = new ndDemoEntity(matrix1, rootEntity);
		
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix1);
		body->SetCollisionShape(shape);
		body->SetMassMatrix(1.0f, shape);
		
		world->AddBody(body);
		
		bodies[i] = body;
		matrix.m_posit = pathBodyMatrix.TransformVector(ndVector(positions[i].m_x, positions[i].m_y, positions[i].m_z, 1.0));
		world->AddJoint(new ndFollowSplinePath(matrix, body, pathBody));
		
		ndVector veloc(dir.Scale(20.0f));
		body->SetVelocity(veloc);
	}

	for (ndInt32 i = 1; i < count; i++) 
	{
		ndBodyDynamic* const box0 = bodies[i - 1];
		ndBodyDynamic* const box1 = bodies[i - 0];

		ndMatrix matrix0(box0->GetMatrix());
		ndMatrix matrix1(box1->GetMatrix());

		matrix0.m_posit.m_y += attachmentOffset;
		matrix1.m_posit.m_y += attachmentOffset;

		world->AddJoint(new ndJointFixDistance(matrix1.m_posit, matrix0.m_posit, box1, box0));
	}
}

void ndBasicJoints (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	BuildBallSocket(scene, ndVector(0.0f, 0.0f, -7.0f, 1.0f));
	//BuildHinge(scene, ndVector(0.0f, 0.0f, -2.0f, 1.0f), 10.0f, 1.0f);
	//BuildSlider(scene, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 100.0f, 0.75f);
	//BuildGear(scene, ndVector(0.0f, 0.0f, -4.0f, 1.0f), 100.0f, 0.75f);
	//BuildDoubleHinge(scene, ndVector(0.0f, 0.0f, 4.0f, 1.0f), 100.0f, 0.75f);
	//BuildFixDistanceJoints(scene, ndVector(-4.0f, 0.0f, -5.0f, 1.0f));
	//BuildRollingFriction(scene, ndVector(-4.0f, 0.0f, 0.0f, 1.0f), 10.0f, 0.5f);
	//AddPathFollow(scene, ndVector(40.0f, 0.0f, 0.0f, 1.0f));
	
	ndQuaternion rot;
	ndVector origin(-20.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
