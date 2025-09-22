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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

class BackGroundVehicleController : public ndModel
{
	public:
	class ndNotify : public ndModelNotify
	{
		public:
		class ndWheelSpin
		{
			public:
			ndMatrix m_bindMatrix;
			ndRenderSceneNode* m_wheelNode;
			ndFloat32 m_angle;
			ndFloat32 m_invRadius;
		};

		ndNotify(ndDemoEntityManager* pScene, const ndSharedPtr<ndBody>& body)
			:ndModelNotify()
			,m_scene(pScene)
			,m_vehicleBody(body)
			,m_desiredSpeed(ndFloat32 (0.0f))
			,m_desiredAngularSpeedFactor(ndFloat32(0.0f))
		{
			ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetAsBodyKinematic()->GetNotifyCallback();
			ndSharedPtr<ndRenderSceneNode> vehicleMesh(notify->GetUserData());

			// iterate ove each animated mesh part, 
			// and save the nessery information to play the animation.
			// is this example we only spin the tires.
			const ndString tireId("tire");
			const ndVector rightDir(vehicleMesh->GetTransform().GetMatrix().m_right);
			const ndList<ndSharedPtr<ndRenderSceneNode>>& children = vehicleMesh->GetChilden();
			for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = children.GetFirst(); node; node = node->GetNext())
			{
				ndRenderSceneNode* const child = *node->GetInfo();
				if (child->m_name.Find(tireId) >= 0)
				{
					ndWheelSpin wheel;
					wheel.m_wheelNode = child;
					wheel.m_bindMatrix = child->GetTransform().GetMatrix();
					ndFloat32 dir = wheel.m_bindMatrix.m_front.DotProduct(rightDir).GetScalar();
					wheel.m_angle = ndFloat32 (0.0f);
					wheel.m_invRadius = (dir < 0.0f) ? ndFloat32 (1.0f/0.4f) : ndFloat32(-1.0f / 0.4f);
					m_wheelAnimation.PushBack(wheel);
				}
			}
		}

		// update pseudo physics every substep
		void Update(ndFloat32 timestep)
		{
			ndModelNotify::Update(timestep);
			if (IsOnGround())
			{
				ApplyTurningImpulse(timestep);
				ApplyLateralImpulse(timestep);
				ApplyLongitudinalImpulse(timestep);
			}
		}

		// update the body part stuff like animations of the wheels,
		// setting the follow camera, apply controls, etc;
		void PostTransformUpdate(ndFloat32 timestep)
		{
			ndModelNotify::PostTransformUpdate(timestep);

			// apply vehicle control 
			ApplyImpulControls();

			// apply tire animation and othe stuff if nessesary
			AnimateTires(timestep);
		}

		private:
		void ApplyImpulControls()
		{
			// apply forward controls
			m_desiredSpeed = ndFloat32(0.0f);
			if (m_scene->GetKeyState(ImGuiKey_UpArrow))
			{
				m_desiredSpeed = ndFloat32(3.0f);
			}
			else if (m_scene->GetKeyState(ImGuiKey_DownArrow))
			{
				m_desiredSpeed = ndFloat32(-1.5f);
			}

			// apply turning controls
			m_desiredAngularSpeedFactor = ndFloat32(0.0f);
			if (m_scene->GetKeyState(ImGuiKey_LeftArrow))
			{
				m_desiredAngularSpeedFactor = ndFloat32(0.5f);
			}
			else if (m_scene->GetKeyState(ImGuiKey_RightArrow))
			{
				m_desiredAngularSpeedFactor = ndFloat32(-0.5f);
			}
		}

		bool IsOnGround() const
		{
			const ndBodyKinematic* const vehicleBody = m_vehicleBody->GetAsBodyKinematic();
			ndInt32 wheelContacts = 0;
			ndBodyKinematic::ndContactMap::Iterator it(vehicleBody->GetContactMap());
			for (it.Begin(); it; it++)
			{
				const ndContact* const contact = it.GetNode()->GetInfo();
				if (contact->IsActive())
				{
					const ndContactPointList& contactPoints = contact->GetContactPoints();
					for (ndContactPointList::ndNode* node = contactPoints.GetFirst(); node; node = node->GetNext())
					{
						const ndContactMaterial& contactPoint = node->GetInfo();
						const ndShapeInstance* const instance = (contactPoint.m_body0 == vehicleBody) ? contactPoint.m_shapeInstance0 : contactPoint.m_shapeInstance1;
						const ndShape* const shape = instance->GetShape();
						const ndShapeChamferCylinder* const wheel = ((ndShape*)shape)->GetAsShapeChamferCylinder();
						if (wheel)
						{
							wheelContacts++;
							if (wheelContacts >= 3)
							{
								return true;
							}
						}
					}
				}
			}
			return false;
		}

		void AnimateTires(ndFloat32 timestep)
		{
			const ndBodyKinematic* const vehicleBody = m_vehicleBody->GetAsBodyKinematic();
			const ndMatrix& matrix = vehicleBody->GetMatrix();
			const ndVector veloc(vehicleBody->GetVelocity());
			ndFloat32 speed = veloc.DotProduct(matrix.m_front).GetScalar();
			ndFloat32 step = speed * timestep;
			for (ndInt32 i = 0; i < m_wheelAnimation.GetCount(); ++i)
			{
				ndFloat32 angleAngle = step * m_wheelAnimation[i].m_invRadius;
				ndFloat32 angle = m_wheelAnimation[i].m_angle + angleAngle;
				if (angle > ndPi * ndFloat32(2.0f))
				{
					angle -= ndPi * ndFloat32(2.0f);
				}
				else if (angle < -ndPi * ndFloat32(2.0f))
				{
					angle += ndPi * ndFloat32(2.0f);
				}
				m_wheelAnimation[i].m_angle = angle;

				const ndMatrix wheelMatrix(ndPitchMatrix(angle) * m_wheelAnimation[i].m_bindMatrix);
				m_wheelAnimation[i].m_wheelNode->SetTransform(wheelMatrix, wheelMatrix.m_posit);
			}
		}

		void ApplyLongitudinalImpulse(ndFloat32 timestep)
		{
			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();

			const ndMatrix matrix(vehicleBody->GetMatrix());
			const ndVector velocity(m_vehicleBody->GetVelocity());
			ndFloat32 speed = matrix.m_front.DotProduct(velocity).GetScalar();
			ndFloat32 speedError = m_desiredSpeed - speed;
			ndFloat32 impulseMag = ndFloat32(0.4f) * speedError * vehicleBody->GetMassMatrix().m_w;
			vehicleBody->ApplyImpulsePair(matrix.m_front.Scale (impulseMag), ndVector::m_zero, timestep); 
		}

		void ApplyLateralImpulse(ndFloat32 timestep)
		{
			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();

			const ndMatrix matrix(vehicleBody->GetMatrix());
			const ndVector forward (matrix.m_front);
			const ndVector upDir (matrix.m_up);

			// Get some info about our current velocity
			const ndVector velocity = m_vehicleBody->GetVelocity();

			// Work out lateral force to stop sliding (all in world space)
			const ndVector sidewayVelocity = velocity - forward.Scale(velocity.DotProduct(forward).GetScalar()) - upDir.Scale(velocity.DotProduct(upDir).GetScalar());
			const ndVector velocityUp = velocity - forward.Scale(velocity.DotProduct(forward).GetScalar()) - sidewayVelocity;
			const ndVector sidewaysMomentum = sidewayVelocity.Scale(vehicleBody->GetMassMatrix().m_w);
			const ndVector verticalMomentum = velocityUp.Scale(vehicleBody->GetMassMatrix().m_w);

			// 0.8 = momentum canceling factor
			vehicleBody->ApplyImpulsePair(ndVector::m_negOne * sidewaysMomentum * 0.8f, ndVector(0.0f), timestep);
		}

		void ApplyTurningImpulse(ndFloat32 timestep)
		{
			if (m_desiredSpeed == 0.0f)
			{
				return;
			}

			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();
			const ndMatrix matrix(vehicleBody->GetMatrix());

			const ndVector upDir(matrix.m_up);

			ndFloat32 turningSign = ndSign(m_desiredSpeed);
			ndFloat32 omega = upDir.DotProduct(m_vehicleBody->GetOmega()).GetScalar();
			ndFloat32 deltaOmega = turningSign * m_desiredAngularSpeedFactor - omega;
			ndFloat32 angularImpulse = ndFloat32(0.3f) * deltaOmega * vehicleBody->GetMassMatrix().m_y;
			vehicleBody->ApplyImpulsePair(ndVector::m_zero, upDir.Scale(angularImpulse), timestep);
		}

		ndDemoEntityManager* m_scene;
		ndSharedPtr<ndBody> m_vehicleBody;
		ndFloat32 m_desiredSpeed;
		ndFloat32 m_desiredAngularSpeedFactor;
		ndFixSizeArray<ndWheelSpin, 8> m_wheelAnimation;
	};

	BackGroundVehicleController(ndDemoEntityManager* pScene, const ndSharedPtr<ndBody>& body)
		:ndModel()
	{
		SetNotifyCallback(ndSharedPtr<ndModelNotify>(new ndNotify(pScene, body)));
	}
};

static ndSharedPtr<ndBody> CreateAiPropVehicle(ndDemoEntityManager* const scene)
{
	ndMeshLoader loader;
	ndSharedPtr<ndRenderSceneNode> vehicleMesh(loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("vmw.fbx")));
	ndMatrix matrix (ndGetIdentityMatrix());
	ndVector floor(FindFloor(*scene->GetWorld(), ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	floor.m_y += ndFloat32 (1.0f);
	matrix.m_posit = floor;

	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* renderChildren = vehicleMesh->GetChilden().GetFirst();

	// build a compound collision shape with chassis and tires
	ndSharedPtr<ndShapeInstance> compoundShapeInstance(new ndShapeInstance(new ndShapeCompound()));
	ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
	compoundShape->BeginAddRemove();
		
		//add chassis shape
		ndSharedPtr<ndShapeInstance> chassisShape (loader.m_mesh->CreateCollision());
		compoundShape->AddCollision(*chassisShape);

		// add all tires
		ndString tirename("tire");
		const ndList<ndSharedPtr<ndMesh>>& children = loader.m_mesh->GetChildren();
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* node = children.GetFirst(); node; node = node->GetNext())
		{
			ndMesh* const child = *node->GetInfo();
			if (child->GetName().Find(tirename) >= 0)
			{
				// we can use a convex hull for simplicity, 
				// but a tire capsule is much better choice because it generates 
				// only one contact
				ndSharedPtr<ndShapeInstance> tireShape(child->CreateCollision());
				tireShape->SetLocalMatrix(tireShape->GetLocalMatrix() * child->m_matrix);
				compoundShape->AddCollision(*tireShape);
			}
			renderChildren = renderChildren->GetNext();
		}
	compoundShape->EndAddRemove();


	ndWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> vehicleBody(new ndBodyDynamic());
	vehicleBody->SetNotifyCallback(new ndDemoEntityNotify(scene, vehicleMesh));
	vehicleBody->SetMatrix(matrix);
	vehicleBody->GetAsBodyDynamic()->SetCollisionShape(**compoundShapeInstance);
	vehicleBody->GetAsBodyDynamic()->SetMassMatrix(1000.0f, **chassisShape);

	ndSharedPtr<ndModel> controller(new BackGroundVehicleController(scene, vehicleBody));

	world->AddBody(vehicleBody);
	world->AddModel(controller);
	scene->AddEntity(vehicleMesh);

	return vehicleBody;
}

void ndBasicModel(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));
	//ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));
	
	ndSharedPtr<ndBody> vehicleBody(CreateAiPropVehicle(scene));

	//ndVector origin(vehicleBody->GetPosition());
	//ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
	//origin.m_x += 10.0f;
	//scene->SetCameraMatrix(rot, origin);

	ndVector floor(FindFloor(*scene->GetWorld(), ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	ndQuaternion rot(ndYawMatrix(0.0f * ndDegreeToRad));

	floor.m_x -= 10.0f;
	floor.m_y += 2.0f;
	scene->SetCameraMatrix(rot, floor);
}
