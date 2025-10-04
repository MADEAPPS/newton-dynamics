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
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"

// 16 mps approximatly 60 kmp
#define ND_MAX_PROP_VEHICLE_SPEED		ndFloat32(12.0f)
#define ND_PROP_VEHICLE_CAMERA_DISTANCE	ndFloat32(-7.0f)

class ndBackGroundVehicleController : public ndModelNotify
{
	public:

	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "implements a basic simple background prp");
			scene->Print(color, "c key to change player");
			scene->Print(color, "a key for turning left");
			scene->Print(color, "d key for turning right");
			scene->Print(color, "w key for moving walking forward");
			scene->Print(color, "s key for going walking backward");
			scene->Print(color, "left click on dynamics body for picking the body");
		}
	};

	class ndWheelSpin
	{
		public:
		ndMatrix m_bindMatrix;
		ndRenderSceneNode* m_wheelNode;
		ndFloat32 m_angle;
		ndFloat32 m_invRadius;
	};

	ndBackGroundVehicleController(
		ndDemoEntityManager* const scene, const ndSharedPtr<ndBody>& body)
		:ndModelNotify()
		,m_scene(scene)
		,m_vehicleBody(body)
		,m_cameraNode(nullptr)
		,m_desiredSpeed(ndFloat32(0.0f))
		,m_desiredAngularSpeedFactor(ndFloat32(0.0f))
	{
		ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetAsBodyKinematic()->GetNotifyCallback();
		ndSharedPtr<ndRenderSceneNode> vehicleMesh(notify->GetUserData());

		// iterate ove each animated mesh part, 
		// and save the nessery information to play the animation.
		// is this example we only spin the tires.
		const ndString tireId("tire");
		const ndVector rightDir(vehicleMesh->GetTransform().GetMatrix().m_right);
		const ndList<ndSharedPtr<ndRenderSceneNode>>& children = vehicleMesh->GetChildren();
		for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = children.GetFirst(); node; node = node->GetNext())
		{
			ndRenderSceneNode* const child = *node->GetInfo();
			if (child->m_name.Find(tireId) >= 0)
			{
				ndWheelSpin wheel;
				wheel.m_wheelNode = child;
				wheel.m_bindMatrix = child->GetTransform().GetMatrix();
				ndFloat32 dir = wheel.m_bindMatrix.m_front.DotProduct(rightDir).GetScalar();
				wheel.m_angle = ndFloat32(0.0f);
				wheel.m_invRadius = (dir < 0.0f) ? ndFloat32(1.0f / 0.4f) : ndFloat32(-1.0f / 0.4f);
				m_wheelAnimation.PushBack(wheel);
			}
		}

		// attach a follow camera to the vehicle prop
		const ndVector cameraPivot(0.0f, 2.0f, 0.0f, 0.0f);
		ndRender* const renderer = *m_scene->GetRenderer();
		m_cameraNode = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, ND_PROP_VEHICLE_CAMERA_DISTANCE));
		vehicleMesh->AddChild(m_cameraNode);
	}

	ndSharedPtr<ndRenderSceneNode> GetCamera()
	{
		return m_cameraNode;
	}

	static ndSharedPtr<ndModelNotify> CreateAiVehicleProp(ndDemoEntityManager* const scene, const ndVector& location,  const ndMeshLoader& loader)
	{
		ndSharedPtr<ndRenderSceneNode> vehicleMesh(loader.m_renderMesh->Clone());
		ndMatrix matrix(ndGetIdentityMatrix());
		matrix.m_posit = location;
		matrix.m_posit.m_y += ndFloat32(1.0f);
		matrix.m_posit.m_w = ndFloat32 (1.0f);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);

		ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* renderChildren = vehicleMesh->GetChildren().GetFirst();

		// build a compound collision shape with chassis and tires
		ndSharedPtr<ndShapeInstance> compoundShapeInstance(new ndShapeInstance(new ndShapeCompound()));
		ndShapeCompound* const compoundShape = compoundShapeInstance->GetShape()->GetAsShapeCompound();
		compoundShape->BeginAddRemove();

		//add chassis shape
		ndSharedPtr<ndShapeInstance> chassisShape(loader.m_mesh->CreateCollision());
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

		ndSharedPtr<ndModel> model(new ndModel());
		ndSharedPtr<ndModelNotify> controller(new ndBackGroundVehicleController(scene, vehicleBody));
		model->SetNotifyCallback(controller);

		world->AddBody(vehicleBody);
		scene->AddEntity(vehicleMesh);
		world->AddModel(model);
		return controller;
	}

	private:
	// update pseudo physics every substep
	void Update(ndFloat32 timestep) override
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
		ApplyImpulseControls();

		// apply tire animation and othe stuff if nessesary
		AnimateTires(timestep);
	}

	void ApplyImpulseControls()
	{
		// apply forward controls
		m_desiredSpeed = ndFloat32(0.0f);
		if (m_scene->GetKeyState(ImGuiKey_W))
		{
			m_desiredSpeed = ND_MAX_PROP_VEHICLE_SPEED;
		}
		else if (m_scene->GetKeyState(ImGuiKey_S))
		{
			m_desiredSpeed = -0.5f * ND_MAX_PROP_VEHICLE_SPEED;
		}

		// apply turning controls
		m_desiredAngularSpeedFactor = ndFloat32(0.0f);
		if (m_scene->GetKeyState(ImGuiKey_A))
		{
			m_desiredAngularSpeedFactor = ndFloat32(0.5f);
		}
		else if (m_scene->GetKeyState(ImGuiKey_D))
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
			ndFloat32 angleStep = step * m_wheelAnimation[i].m_invRadius;
			m_wheelAnimation[i].m_angle = ndAnglesAdd(m_wheelAnimation[i].m_angle, angleStep);

			const ndMatrix wheelMatrix(ndPitchMatrix(m_wheelAnimation[i].m_angle) * m_wheelAnimation[i].m_bindMatrix);
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
		ndFloat32 impulseMag = ndFloat32(0.05f) * speedError * vehicleBody->GetMassMatrix().m_w;
		vehicleBody->ApplyImpulsePair(matrix.m_front.Scale(impulseMag), ndVector::m_zero, timestep);
	}

	void ApplyLateralImpulse(ndFloat32 timestep)
	{
		ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();

		const ndMatrix matrix(vehicleBody->GetMatrix());
		const ndVector forward(matrix.m_front);
		const ndVector upDir(matrix.m_up);

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

	public:
	ndDemoEntityManager* m_scene;
	ndSharedPtr<ndBody> m_vehicleBody;
	ndSharedPtr<ndRenderSceneNode> m_cameraNode;
	ndFloat32 m_desiredSpeed;
	ndFloat32 m_desiredAngularSpeedFactor;
	ndFixSizeArray<ndWheelSpin, 8> m_wheelAnimation;
};

void ndBasicModel(ndDemoEntityManager* const scene)
{
	//ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));
	ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));

	// add a help menu
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndBackGroundVehicleController::ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	ndMeshLoader vmwLoaderRedPaint;
	vmwLoaderRedPaint.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("vmwRed.fbx"));
	ndSharedPtr<ndModelNotify> controller(ndBackGroundVehicleController::CreateAiVehicleProp(scene, ndVector::m_wOne, vmwLoaderRedPaint));

	// set this player as the active camera
	ndBackGroundVehicleController* const playerController = (ndBackGroundVehicleController*)*controller;
	ndRender* const renderer = *scene->GetRenderer();
	renderer->SetCamera(playerController->GetCamera());

#if 1
	{
		// add an array of vehicles 
		ndMeshLoader vmwLoaderGreenPaint;
		vmwLoaderGreenPaint.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("vmwGreen.fbx"));

		const ndInt32 count = 5;
		ndFloat32 spacing = ndFloat32(10.0f);

		ndFloat32 x0 = spacing;
		ndFloat32 z0 = -spacing * ndFloat32(count / 2);
		for (ndInt32 i = 0; i < count; ++i)
		{
			for (ndInt32 j = 0; j < count; ++j)
			{
				ndFloat32 z = z0 + ndFloat32(i) * spacing;
				ndFloat32 x = x0 + ndFloat32(j) * spacing;
				x += (ndRand() - ndFloat32(0.5f)) * spacing * ndFloat32(0.25f);
				z += (ndRand() - ndFloat32(0.5f)) * spacing * ndFloat32(0.25f);

				ndVector position(x, ndFloat32(0.0f), z, ndFloat32(1.0f));

				if (ndRandInt() & 1)
				{
					ndBackGroundVehicleController::CreateAiVehicleProp(scene, position, vmwLoaderRedPaint);
				}
				else
				{
					ndBackGroundVehicleController::CreateAiVehicleProp(scene, position, vmwLoaderGreenPaint);
				}
			}
		}
	}
#endif
}
