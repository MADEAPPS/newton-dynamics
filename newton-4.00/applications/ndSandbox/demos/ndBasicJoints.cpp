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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

static ndSharedPtr<ndBody> MakePrimitive(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndShapeInstance& shape, ndSharedPtr<ndRenderPrimitive> mesh, ndFloat32 mass)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
	entity->SetPrimitive(mesh);

	ndSharedPtr<ndBody> body(new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	body->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}

static void BuildBallSocket(ndDemoEntityManager* const scene, const ndVector& origin)
{
	class ndJointSphericalMotor : public ndJointSpherical
	{
		public:
		D_CLASS_REFLECTION(ndJointSphericalMotor, ndJointSpherical)

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

		void JacobianDerivative(ndConstraintDescritor& desc) override
		{
			m_rollAngle = ndFmod(m_rollAngle + m_rollOmega * desc.m_timestep, 2.0f * ndPi);
			m_pitchAngle = ndFmod(m_pitchAngle + m_pitchOmega * desc.m_timestep, 2.0f * ndPi);

			const ndMatrix rotaion(ndPitchMatrix(m_pitchAngle) * ndRollMatrix(m_rollAngle));
			SetOffsetRotation(rotaion);
			ndJointSpherical::JacobianDerivative(desc);
		}

		ndFloat32 m_rollAngle;
		ndFloat32 m_pitchAngle;
		ndFloat32 m_rollOmega;
		ndFloat32 m_pitchOmega;
	};

	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndFloat32 mass = 1.0f;
	ndFloat32 diameter = 0.5f;
	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCapsule(diameter * 0.25f, diameter * 0.25f, diameter * 1.0f)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_capsule;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix refMatrix(ndRollMatrix(90.0f * ndDegreeToRad));
	refMatrix.m_posit = FindFloor(*world, origin, 200.0f);
	{
		// add a spherical motor.
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_y += diameter;
		ndSharedPtr<ndBody> body (MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndMatrix pinAlign(ndRollMatrix(180.0f * ndDegreeToRad));
		ndMatrix bodyMatrix0(pinAlign * body->GetMatrix());
		bodyMatrix0.m_posit.m_y += diameter * 0.5f + diameter * 0.25f;
		ndBodyKinematic* const fixBody = world->GetSentinelBody();
		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointSphericalMotor(bodyMatrix0, body->GetAsBodyDynamic(), fixBody));
		world->AddJoint(joint);
	}

	if (1)
	{
		// add flexible chain with spring damper.
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_z -= 2.0f;

		const ndInt32 count = 6;
		ndBodyDynamic* array[count];
		for (ndInt32 i = 0; i < count; ++i)
		{
			matrix.m_posit.m_y += diameter;
			ndSharedPtr<ndBody> body (MakePrimitive(scene, matrix, **shape, mesh, mass));
			array[i] = body->GetAsBodyDynamic();
		}
		
		ndFloat32 friction = 10.0f;
		ndFloat32 spring = 1500.0f;
		ndFloat32 regularizer = 0.01f;
		
		ndMatrix pinAlign(ndRollMatrix(180.0f * ndDegreeToRad));
		for (ndInt32 i = 1; i < count; ++i)
		{
			ndMatrix bodyMatrix0(array[i - 1]->GetMatrix());
			ndMatrix bodyMatrix1(array[i - 0]->GetMatrix());
			ndMatrix pinMatrix(pinAlign * bodyMatrix0);
			pinMatrix.m_posit = (bodyMatrix0.m_posit + bodyMatrix1.m_posit).Scale(0.5f);
			ndJointSpherical* const joint = new ndJointSpherical(pinMatrix, array[i - 1], array[i - 0]);
			joint->SetAsSpringDamper(regularizer, spring, friction);
			ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
			world->AddJoint(jointPtr);
		}
		
		ndMatrix bodyMatrix0(pinAlign * array[count - 1]->GetMatrix());
		bodyMatrix0.m_posit.m_y += diameter * 0.5f + diameter * 0.25f;
		ndBodyKinematic* const fixBody = world->GetSentinelBody();
		ndJointSpherical* const joint = new ndJointSpherical(bodyMatrix0, array[count - 1], fixBody);
		joint->SetAsSpringDamper(regularizer, spring, friction);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	if (1)
	{
		// add a chain with limits and viscous friction.
		// this is trick extact moementon form the articalation 
		// and it is a practical way to make long change of connected bodies stable.

		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_z -= 4.0f;

		const ndInt32 count = 6;
		ndBodyDynamic* array[count];
		for (ndInt32 i = 0; i < count; ++i)
		{
			matrix.m_posit.m_y += diameter;
			ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
			//ndVector inertia(body->GetMassMatrix());
			//ndFloat32 maxI(dMax(dMax(inertia.m_x, inertia.m_z), inertia.m_z));
			//inertia.m_x = maxI;
			//inertia.m_y = maxI;
			//inertia.m_z = maxI;
			//body->SetMassMatrix(inertia);
			array[i] = body->GetAsBodyDynamic();
		}
		
		ndFloat32 friction = 10.0f;
		ndFloat32 regularizer = 0.1f;
		
		ndMatrix pinAlign(ndRollMatrix(180.0f * ndDegreeToRad));
		// connect all links
		for (ndInt32 i = 1; i < count; ++i)
		{
			ndMatrix bodyMatrix0(array[i - 1]->GetMatrix());
			ndMatrix bodyMatrix1(array[i - 0]->GetMatrix());
			ndMatrix pinMatrix(pinAlign * bodyMatrix0);
			pinMatrix.m_posit = (bodyMatrix0.m_posit + bodyMatrix1.m_posit).Scale(0.5f);
			ndJointSpherical* const joint = new ndJointSpherical(pinMatrix, array[i - 1], array[i - 0]);
			joint->SetAsSpringDamper(regularizer, ndFloat32(0.0f), friction);
			joint->SetConeLimit(60.0f * ndDegreeToRad);
			joint->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
			ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
			world->AddJoint(jointPtr);
		}
		
		// connect root link to the world
		ndMatrix bodyMatrix0(pinAlign * array[count - 1]->GetMatrix());
		bodyMatrix0.m_posit.m_y += diameter * 0.5f + diameter * 0.25f;
		ndBodyKinematic* const fixBody = world->GetSentinelBody();
		ndJointSpherical* const joint = new ndJointSpherical(bodyMatrix0, array[count - 1], fixBody);
		joint->SetAsSpringDamper(regularizer, ndFloat32(0.0f), friction);
		joint->SetConeLimit(60.0f * ndDegreeToRad);
		joint->SetTwistLimits(-90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}
}

static void BuildHinge(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
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
			SetTargetAngle(dist);
			ndJointHinge::JacobianDerivative(desc);
		}

		ndFloat32 m_angle;
	};

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
			SetTargetAngle(angle + m_speed * desc.m_timestep);
			ndJointHinge::JacobianDerivative(desc);
		}
		ndFloat32 m_speed;
	};

	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(diameter, diameter, diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix refMatrix(ndRollMatrix(90.0f * ndDegreeToRad));
	refMatrix.m_posit = FindFloor(*world, origin, 200.0f);
	refMatrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// spring damper and limits
		ndMatrix matrix(refMatrix);
		ndSharedPtr<ndBody> body (MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointHinge* const joint = new ndJointHinge(matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 20.0f, 1.0f);
		joint->SetLimits(-3.0f, 3.0f);
		joint->SetLimitState(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	{
		// viscous friction and limits
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_y += 1.2f;
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointHinge* const joint = new ndJointHinge(matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 0.0f, 1.0f);
		joint->SetLimits(-10.0f, 15.0f);
		joint->SetLimitState(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	{
		// proportional derivative hinge oscillator with limits
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_y += 2.4f;
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointHinge* const joint = new ndJointHingeOscillator(matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 1500.0f, 10.0f);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	{
		// proportional derivative hinge motor with limits
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_y += 3.6f;
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointHinge* const joint = new ndJointHingeMotor(matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 1500.0f, 10.0f);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}
}

static void BuildSlider(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
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
			SetTargetPosit(dist);
			ndJointSlider::JacobianDerivative(desc);
		}

		ndFloat32 m_angle;
	};

	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(diameter, diameter, diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix refMatrix(ndGetIdentityMatrix());
	refMatrix.m_posit = FindFloor(*world, origin, 200.0f);
	refMatrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// spring damper slider with limits
		ndMatrix matrix(refMatrix);
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointSlider* const joint = new ndJointSlider(ndYawMatrix(90.0f * ndDegreeToRad) * matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 100.0f, 5.0f);
		joint->SetLimits(-1.0f, 1.0f);
		joint->SetLimitState(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	{
		// viscous damper slider with limits
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_y += 1.2f;
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointSlider* const joint = new ndJointSlider(ndYawMatrix(90.0f * ndDegreeToRad) * matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 0.0f, 10.0f);
		joint->SetLimits(-1.0f, 1.0f);
		joint->SetLimitState(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	{
		// proportional derivative slider oscillator with limits
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_y += 2.4f;
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointSlider* const joint = new ndJointSliderOscillator(ndYawMatrix(90.0f * ndDegreeToRad) * matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper(0.1f, 500.0f, 10.0f);
		joint->SetLimits(-1.0f, 1.0f);
		joint->SetLimitState(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}
}

static void BuildGear(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(diameter, diameter, diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix matrix(ndRollMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = FindFloor(*world, origin, 200.0f);
	matrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	ndSharedPtr<ndBody> body0 (MakePrimitive(scene, matrix, **shape, mesh, mass));

	matrix.m_posit.m_y += diameter * 1.5f;
	ndSharedPtr<ndBody> body1 (MakePrimitive(scene, matrix, **shape, mesh, mass));

	ndVector pin(matrix.m_front);
	ndSharedPtr<ndJointBilateralConstraint> joint0(new ndJointHinge(matrix, body0->GetAsBodyDynamic(), fixBody));
	ndSharedPtr<ndJointBilateralConstraint> joint1(new ndJointHinge(matrix, body1->GetAsBodyDynamic(), fixBody));
	ndSharedPtr<ndJointBilateralConstraint> joint2(new ndJointGear(4.0f, pin, body0->GetAsBodyDynamic(), pin, body1->GetAsBodyDynamic()));

	world->AddJoint(joint0);
	world->AddJoint(joint1);
	world->AddJoint(joint2);
}

static void BuildDoubleHinge(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	class ndJointDoubleHingeMotor : public ndJointDoubleHinge
	{
		public:
		ndJointDoubleHingeMotor(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
			:ndJointDoubleHinge(pinAndPivotFrame, child, parent)
			,m_angle(150.0f * ndDegreeToRad)
			,m_speed(2.5f)
		{
		}

		void JacobianDerivative(ndConstraintDescritor& desc)
		{
			// first axis is a angle motor, by setting target angle;
			ndFloat32 angle0 = GetAngle0();
			ndFloat32 deltaAngle = ndClamp (m_angle - angle0, -90.0f * ndDegreeToRad, 90.0f * ndDegreeToRad);
			SetOffsetAngle0(angle0 + deltaAngle * desc.m_timestep);

			// second axis an angular motor by integrating the angle 
			ndFloat32 angle1 = GetAngle1();
			SetOffsetAngle1(angle1 + m_speed * desc.m_timestep);

			ndJointDoubleHinge::JacobianDerivative(desc);
		}

		ndFloat32 m_angle;
		ndFloat32 m_speed;
	};

	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCylinder(diameter, diameter, diameter * 0.5f)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_cylindrical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix refMatrix(ndGetIdentityMatrix());
	refMatrix.m_posit = FindFloor(*world, origin, 200.0f);
	refMatrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// free rolling double hinge
		ndMatrix matrix(refMatrix);
		ndSharedPtr<ndBody> body (MakePrimitive(scene, matrix, **shape, mesh, mass));
		body->SetOmega(ndVector(20.0f, 10.0f, 0.0f, 0.0f));

		ndJointDoubleHinge* const joint = new ndJointDoubleHinge(matrix, body->GetAsBodyDynamic(), fixBody);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}

	{
		// proportional derivative double hinge motor
		ndMatrix matrix(refMatrix);
		matrix.m_posit.m_z += 1.8f;
		ndSharedPtr<ndBody> body (MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointDoubleHinge* const joint = new ndJointDoubleHingeMotor(matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamper0(0.1f, 1500.0f, 10.0f);
		joint->SetAsSpringDamper1(0.1f, 1500.0f, 10.0f);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}
}

static void BuildRoller(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeChamferCylinder(diameter * 0.5f, diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_cylindrical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix refMatrix(ndGetIdentityMatrix());
	refMatrix.m_posit = FindFloor(*world, origin, 200.0f);
	refMatrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// spring damper slider with limits, this coudl be use for a roller wheels
		ndMatrix matrix(refMatrix);
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointRoller* const joint = new ndJointRoller(ndPitchMatrix(90.0f * ndDegreeToRad) * matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamperPosit(0.1f, 100.0f, 5.0f);
		joint->SetLimitsPosit(-1.0f, 1.0f);
		joint->SetLimitStatePosit(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}
}

static void BuildCylindrical(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeChamferCylinder(diameter * 0.5f, diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_cylindrical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix refMatrix(ndGetIdentityMatrix());
	refMatrix.m_posit = FindFloor(*world, origin, 200.0f);
	refMatrix.m_posit.m_y += 2.0f;

	ndBodyKinematic* const fixBody = world->GetSentinelBody();
	{
		// spring damper slider with limits
		ndMatrix matrix(refMatrix);
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointCylinder* const joint = new ndJointCylinder(matrix, body->GetAsBodyDynamic(), fixBody);
		joint->SetAsSpringDamperPosit(0.1f, 100.0f, 5.0f);
		joint->SetLimitsPosit(-1.0f, 1.0f);
		joint->SetLimitStatePosit(true);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
	}
}

void BuildFixDistanceJoints(ndDemoEntityManager* const scene, const ndVector& posit)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeSphere(0.25f)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_spherical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("earthmap.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix matrix(ndGetIdentityMatrix());
	const ndVector origin(FindFloor(*world, posit, 200.0f));

	ndBodyDynamic* bodies[8];

	matrix.m_posit = origin + ndVector(0.0f, 4.0f, 0.0f, 0.0f);
	bodies[0] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(2.0f, 4.0f, 0.0f, 0.0f);
	bodies[1] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(2.0f, 4.0f, 2.0f, 0.0f);
	bodies[2] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(0.0f, 4.0f, 2.0f, 0.0f);
	bodies[3] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(0.0f, 6.0f, 0.0f, 0.0f);
	bodies[4] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(2.0f, 6.0f, 0.0f, 0.0f);
	bodies[5] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(2.0f, 6.0f, 2.0f, 0.0f);
	bodies[6] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	matrix.m_posit = origin + ndVector(0.0f, 6.0f, 2.0f, 0.0f);
	bodies[7] = MakePrimitive(scene, matrix, **shape, mesh, 5.0f)->GetAsBodyDynamic();

	for (ndInt32 i = 0; i < 8; ++i)
	{
		ndBodyDynamic* const body0 = bodies[i];
		for (ndInt32 j = i + 1; j < 8; ++j)
		{
			ndBodyDynamic* const body1 = bodies[j];
			ndJointFixDistance* const joint = new ndJointFixDistance(body0->GetMatrix().m_posit, body1->GetMatrix().m_posit, body0, body1);
			ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
			world->AddJoint(jointPtr);
		}
	}
}

static void BuildRollingFriction(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndMatrix matrix(ndRollMatrix(90.0f * ndDegreeToRad));
	matrix.m_posit = FindFloor(*world, origin, 200.0f);
	matrix.m_posit.m_y += 5.0f;

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeSphere(diameter * 0.5f)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_spherical;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_1.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndVector posit(matrix.m_posit);
	for (ndInt32 i = 0; i < 8; ++i)
	{
		ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));
		ndJointBilateralConstraint* const joint = new ndJointDryRollingFriction(body->GetAsBodyDynamic(), world->GetSentinelBody(), 0.5f);
		ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
		world->AddJoint(jointPtr);
		posit.m_y += diameter * 1.5f;
		matrix.m_posit = posit + ndVector(ndGaussianRandom(0.0f, 0.01f), 0.0f, ndGaussianRandom(0.0f, 0.01f), 0.0f);
	}
}

class ndRollerCoasterModelNotify: public ndModelNotify
{
	class ndJointFollowSplinePath : public ndJointFollowPath
	{
		public:
		D_CLASS_REFLECTION(ndJointFollowSplinePath, ndJointFollowPath)

		ndJointFollowSplinePath()
			:ndJointFollowPath()
		{
		}

		ndJointFollowSplinePath(
			const ndMatrix& pinAndPivotFrame, 
			ndBodyDynamic* const child, 
			ndBodyDynamic* const pathBody,
			const ndSharedPtr<ndBezierSpline>& spline)
			:ndJointFollowPath(pinAndPivotFrame, child, pathBody)
			,m_spline(spline)
		{
		}

		void GetPointAndTangentAtLocation(const ndVector& location, ndVector& positOut, ndVector& tangentOut) const override
		{
			const ndBodyKinematic* const splineBody = GetBody1();
			//const ndBezierSpline& spline = splineBody->m_spline;

			ndMatrix matrix(splineBody->GetMatrix());

			ndVector p(matrix.UntransformVector(location));
			ndBigVector point;
			ndFloat64 knot = m_spline->FindClosestKnot(point, p, 4);
			ndBigVector tangent(m_spline->CurveDerivative(knot));
			tangent = tangent.Scale(1.0 / ndSqrt(tangent.DotProduct(tangent).GetScalar()));
			positOut = matrix.TransformVector(point);
			tangentOut = tangent;
		}

		ndSharedPtr<ndBezierSpline> m_spline;
	};

	public:
	ndRollerCoasterModelNotify(ndDemoEntityManager* const scene, const ndVector& origin)
		:ndModelNotify()
		,m_spline(nullptr)
	{
		ndSharedPtr<ndRenderSceneNode> visualSpline (CreateSpline(scene, origin));
		BuildCoaster(scene, visualSpline, origin);
	}

	void BuildCoaster(ndDemoEntityManager* const scene, const ndSharedPtr<ndRenderSceneNode>& visualPath, const ndVector& origin)
	{
		ndPhysicsWorld* const world = scene->GetWorld();
		ndMatrix matrix(ndGetIdentityMatrix());
		matrix.m_posit = origin;

		// create a Bezier Spline path for AI car to drive
		ndShapeInstance box(new ndShapeBox(1.0f, 1.0f, 1.0f));
		ndSharedPtr<ndBody> pathBody(new ndBodyDynamic());
		pathBody->SetNotifyCallback(new ndDemoEntityNotify(scene, visualPath));
		pathBody->SetMatrix(matrix);
		pathBody->GetAsBodyKinematic()->SetCollisionShape(box);
		world->AddBody(pathBody);

		// create all the roller coater carts
		ndRender* const render = *scene->GetRenderer();
		ndMatrix pathBodyMatrix(pathBody->GetMatrix());

		ndFloat32 attachmentOffset = ndFloat32 (0.8f);
		ndSharedPtr<ndShapeInstance> shape(new ndShapeInstance(new ndShapeChamferCylinder(ndFloat32(0.5f), ndFloat32(0.5f))));

		ndRenderPrimitive::ndDescriptor descriptor(render);
		descriptor.m_collision = shape;
		descriptor.m_mapping = ndRenderPrimitive::m_cylindrical;
		//descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));
		descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));
		ndSharedPtr<ndRenderPrimitive> coasterCarPrimitive(new ndRenderPrimitive(descriptor));

		ndBigVector point0;
		ndFixSizeArray<ndVector, 64> positions;
		ndFloat64 knot = m_spline->FindClosestKnot(point0, ndBigVector(ndVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f)), 4);
		positions.PushBack(point0);
		for (ndInt32 i = 0; i < 32; ++i)
		{
			ndBigVector point1;
			ndBigVector tangent(m_spline->CurveDerivative(knot));
			tangent = tangent.Scale(1.0 / ndSqrt(tangent.DotProduct(tangent).GetScalar()));
			knot = m_spline->FindClosestKnot(point1, ndBigVector(point0 + tangent.Scale(2.0f)), 4);
			point0 = point1;
			positions.PushBack(point1);
		}

		ndFixSizeArray<ndBodyDynamic*, 64> bodies;
		for (ndInt32 i = 0; i < positions.GetCount() - 1; ++i)
		{
			ndVector location0(positions[i + 0].m_x, positions[i + 0].m_y, positions[i + 0].m_z, ndFloat32(0.0f));
			ndVector location1(positions[i + 1].m_x, positions[i + 1].m_y, positions[i + 1].m_z, ndFloat32(0.0f));
		
			location0 = pathBodyMatrix.TransformVector(location0);
			location1 = pathBodyMatrix.TransformVector(location1);
		
			ndVector dir(location1 - location0);
			dir.m_w = 0.0f;
			matrix.m_front = dir.Scale(1.0f / ndSqrt(dir.DotProduct(dir).GetScalar()));
			matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
			matrix.m_right = matrix.m_right.Scale(1.0f / ndSqrt(matrix.m_right.DotProduct(matrix.m_right).GetScalar()));
			matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
			matrix.m_posit = pathBodyMatrix.TransformVector(ndVector(positions[i].m_x, positions[i].m_y - attachmentOffset, positions[i].m_z, ndFloat32(1.0f)));
			ndMatrix matrix1(ndYawMatrix(0.5f * ndPi) * matrix);
		
			ndSharedPtr<ndBody> body(new ndBodyDynamic());
			ndSharedPtr<ndRenderSceneNode>coasterCar(new ndRenderSceneNode(matrix1));
			visualPath->AddChild(coasterCar);
			coasterCar->SetPrimitive(coasterCarPrimitive);
		
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, coasterCar));
			body->SetMatrix(matrix1);
			body->GetAsBodyDynamic()->SetCollisionShape(**shape);
			body->GetAsBodyDynamic()->SetMassMatrix(ndFloat32 (1.0f), **shape);
		
			world->AddBody(body);
		
			bodies.PushBack(body->GetAsBodyDynamic());
			matrix.m_posit = pathBodyMatrix.TransformVector(ndVector(positions[i].m_x, positions[i].m_y, positions[i].m_z, ndFloat32(1.0f)));
		
			ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointFollowSplinePath(matrix, body->GetAsBodyDynamic(), pathBody->GetAsBodyDynamic(), m_spline));
			world->AddJoint(joint);
		
			ndVector veloc(dir.Scale(20.0f));
			body->SetVelocity(veloc);
		}
		
		for (ndInt32 i = 1; i < bodies.GetCount(); ++i)
		{
			ndBodyDynamic* const box0 = bodies[i - 1];
			ndBodyDynamic* const box1 = bodies[i - 0];
		
			ndMatrix matrix0(box0->GetMatrix());
			ndMatrix matrix1(box1->GetMatrix());
		
			matrix0.m_posit.m_y += attachmentOffset;
			matrix1.m_posit.m_y += attachmentOffset;
		
			ndJointFixDistance* const joint = new ndJointFixDistance(matrix1.m_posit, matrix0.m_posit, box1, box0);
			ndSharedPtr<ndJointBilateralConstraint> jointPtr(joint);
			world->AddJoint(jointPtr);
		}
	}
	
	ndSharedPtr<ndRenderSceneNode> CreateSpline(ndDemoEntityManager* const scene, const ndVector& origin)
	{
		// create the splane path
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

		m_spline = ndSharedPtr<ndBezierSpline>(new ndBezierSpline);
		m_spline->CreateFromKnotVectorAndControlPoints(3, sizeof(knots) / sizeof(knots[0]), knots, control);

		ndSharedPtr<ndRenderPrimitiveSimpleMesh> wireFrame(new ndRenderPrimitiveSimpleMesh);

		// build the mesh wire frame line list
		ndFloat64 resolution = 500;
		ndFloat64 scale = 1.0f / resolution;
		wireFrame->m_type = ndRenderPrimitiveSimpleMesh::m_lines;

		ndVector color(ndFloat32(1.0f));
		ndBigVector p0(m_spline->CurvePoint(0));
		for (ndInt32 i = 1; i < resolution; ++i)
		{
			ndBigVector p1(m_spline->CurvePoint(i * scale));
			wireFrame->m_vertex.PushBack(ndVector(p0));
			wireFrame->m_color.PushBack(color);

			wireFrame->m_vertex.PushBack(ndVector(p1));
			wireFrame->m_color.PushBack(color);
			p0 = p1;
		}
		wireFrame->m_vertex.PushBack(ndVector(p0));
		wireFrame->m_color.PushBack(color);

		wireFrame->m_vertex.PushBack(wireFrame->m_vertex[0]);
		wireFrame->m_color.PushBack(color);

		ndRender* const render = *scene->GetRenderer();
		ndRenderPrimitive::ndDescriptor descriptor(render);
		descriptor.m_simpleMesh = wireFrame;

		ndSharedPtr<ndRenderPrimitive>mesh(new ndRenderPrimitive(descriptor));

		ndMatrix matrix(ndGetIdentityMatrix());
		matrix.m_posit = origin;
		matrix.m_posit.m_w = ndFloat32 (1.0f);
		ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(matrix));
		entity->SetPrimitive(mesh);

		scene->AddEntity(entity);
		return entity;
	}

	ndSharedPtr<ndBezierSpline> m_spline;
};

static void BuildPathFollow(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndModel> model(new ndModel());
	ndSharedPtr<ndModelNotify> rollerCoster(new ndRollerCoasterModelNotify(scene, origin));
	model->SetNotifyCallback(rollerCoster);
	world->AddModel(model);
}

static void BuildUpVector(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeCapsule(diameter, diameter, diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_capsule;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix matrix(ndRollMatrix(-90.0f * ndDegreeToRad));
	matrix.m_posit = FindFloor(*world, origin, 200.0f);
	matrix.m_posit.m_y += 2.0f;

	matrix.m_posit.m_y += diameter * 1.5f;
	ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));

	const ndVector pin(matrix.m_front);
	ndSharedPtr<ndJointBilateralConstraint> upVector(new ndJointUpVector(pin, body->GetAsBodyDynamic(), world->GetSentinelBody()));
	world->AddJoint(upVector);
}


static void BuildWheel(ndDemoEntityManager* const scene, const ndVector& origin, ndFloat32 mass, ndFloat32 diameter)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	ndRender* const render = *scene->GetRenderer();

	ndSharedPtr<ndShapeInstance>shape(new ndShapeInstance(new ndShapeBox(diameter * 2.0f, 0.5f * diameter, 1.5f * diameter)));
	ndRenderPrimitive::ndDescriptor descriptor(render);
	descriptor.m_collision = shape;
	descriptor.m_mapping = ndRenderPrimitive::m_box;
	descriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("wood_0.png")));

	ndSharedPtr<ndRenderPrimitive> mesh(new ndRenderPrimitive(descriptor));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = FindFloor(*world, origin, 200.0f);
	matrix.m_posit.m_y += diameter * 1.5f;
	ndSharedPtr<ndBody> body(MakePrimitive(scene, matrix, **shape, mesh, mass));

	// add two roller wheels
	{
		ndSharedPtr<ndShapeInstance>rollerShape(new ndShapeInstance(new ndShapeChamferCylinder(0.25f * diameter, 0.25f * diameter)));
		ndRenderPrimitive::ndDescriptor rollerDescriptor(render);
		rollerDescriptor.m_collision = rollerShape;
		rollerDescriptor.m_mapping = ndRenderPrimitive::m_capsule;
		rollerDescriptor.AddMaterial(render->GetTextureCache()->GetTexture(ndGetWorkingFileName("smilli.png")));
		ndSharedPtr<ndRenderPrimitive> rollerMesh(new ndRenderPrimitive(rollerDescriptor));

		{
			// add a roller
			ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * matrix);
			rollerMatrix.m_posit.m_x -= diameter * 0.9f;
			rollerMatrix.m_posit.m_z -= diameter * 0.8f;
			rollerMatrix.m_posit.m_y -= diameter * 0.25f;
			ndSharedPtr<ndBody> roller(MakePrimitive(scene, rollerMatrix, **rollerShape, rollerMesh, mass * 0.125f));

			const ndMatrix rollerPin(rollerMatrix);
			ndSharedPtr<ndJointBilateralConstraint> rollerAxle(new ndJointHinge(rollerPin, roller->GetAsBodyDynamic(), body->GetAsBodyDynamic()));
			world->AddJoint(rollerAxle);
		}

		{
			// add another roller
			ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * matrix);
			rollerMatrix.m_posit.m_x -= diameter * 0.9f;
			rollerMatrix.m_posit.m_z += diameter * 0.8f;
			rollerMatrix.m_posit.m_y -= diameter * 0.25f;
			ndSharedPtr<ndBody> roller(MakePrimitive(scene, rollerMatrix, **rollerShape, rollerMesh, mass * 0.125f));

			const ndMatrix rollerPin(rollerMatrix);
			ndSharedPtr<ndJointBilateralConstraint> rollerAxle(new ndJointHinge(rollerPin, roller->GetAsBodyDynamic(), body->GetAsBodyDynamic()));
			world->AddJoint(rollerAxle);
		}

		{
			// add a wheel
			ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * matrix);
			rollerMatrix.m_posit.m_x += diameter * 1.0f;
			rollerMatrix.m_posit.m_y -= diameter * 0.25f;
			ndSharedPtr<ndBody> roller1(MakePrimitive(scene, rollerMatrix, **rollerShape, rollerMesh, mass * 0.125f));

			ndWheelDescriptor desc;
			const ndMatrix rollerPin(rollerMatrix);
			ndSharedPtr<ndJointBilateralConstraint> wheelAxle(new ndJointWheel(rollerPin, roller1->GetAsBodyDynamic(), body->GetAsBodyDynamic(), desc));
			world->AddJoint(wheelAxle);
		}
	}
}

void ndBasicJoints (ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	//BuildBallSocket(scene, ndVector(0.0f, 0.0f, -7.0f, 1.0f));
	//BuildHinge(scene, ndVector(0.0f, 0.0f, -2.0f, 1.0f), 10.0f, 1.0f);
	//BuildSlider(scene, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 100.0f, 0.75f);
	//BuildGear(scene, ndVector(0.0f, 0.0f, -4.0f, 1.0f), 100.0f, 0.75f);
	//BuildDoubleHinge(scene, ndVector(0.0f, 0.0f, 4.0f, 1.0f), 100.0f, 0.75f);
	//BuildRoller(scene, ndVector(0.0f, 0.0f, 9.0f, 1.0f), 10.0f, 0.75f);
	//BuildCylindrical(scene, ndVector(0.0f, 0.0f, 12.0f, 1.0f), 10.0f, 0.75f);
	//BuildFixDistanceJoints(scene, ndVector( -4.0f, 0.0f, -5.0f, 1.0f));
	//BuildRollingFriction(scene, ndVector(-4.0f, 0.0f, 5.0f, 1.0f), 10.0f, 0.5f);
	//BuildPathFollow(scene, ndVector(40.0f, 0.0f, 0.0f, 1.0f));

	BuildWheel(scene, ndVector(0.0f, 0.0f, -4.0f, 1.0f), 100.0f, 0.75f);
	//BuildUpVector(scene, ndVector(0.0f, 0.0f, -4.0f, 1.0f), 100.0f, 0.75f);
	
	ndQuaternion rot;
	ndVector origin(-20.0f, 5.0f, 0.0f, 1.0f);

	scene->SetCameraMatrix(rot, origin);
}
