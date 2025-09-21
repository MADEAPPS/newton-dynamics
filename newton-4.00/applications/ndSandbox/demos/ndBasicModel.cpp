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

#if 0
class BackgroundLowLodCVehicleMaterial : public ndApplicationMaterial
{
	public:
	BackgroundLowLodCVehicleMaterial()
		:ndApplicationMaterial()
	{
	}

	BackgroundLowLodCVehicleMaterial(const BackgroundLowLodCVehicleMaterial& src)
		:ndApplicationMaterial(src)
	{
	}

	ndApplicationMaterial* Clone() const
	{
		return new BackgroundLowLodCVehicleMaterial(*this);
	}

	virtual void OnContactCallback(const ndContact* const joint, ndFloat32) const
	{
		if (joint->IsActive())
		{
			const ndContactPointList& contactPoints = joint->GetContactPoints();
			for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
			{
				// do some logic here
				//ndContactPoint& contactPoint = contactPointsNode->GetInfo();
			}
		}
	}
};

static ndShapeInstance CreateCompoundCollision()
{
	ndMatrix mParentInv = ndGetIdentityMatrix();
	mParentInv.m_posit = ndVector(ndFloat32(0.0), ndFloat32(-1.0), ndFloat32(0.0), ndFloat32(1.0));
	ndMatrix mChildWorld = ndGetIdentityMatrix();
	mChildWorld.m_front = ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0));
	mChildWorld.m_up = ndVector(ndFloat32(1.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0));
	mChildWorld.m_right = ndVector(ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0), ndFloat32(0.0));
	mChildWorld.m_posit = ndVector(ndFloat32(0.0), ndFloat32(1.158), ndFloat32(-0.508), ndFloat32(1.0));
	ndMatrix mChildLocal = mChildWorld * mParentInv;

#if 0
	ndShapeInstance shapeInstance(new ndShapeCompound());

	shapeInstance.GetShape()->GetAsShapeCompound()->BeginAddRemove();
	ndShapeInstance childShapeInstance(new ndShapeConvexHull(336 / 3, sizeof(ndFloat32) * 3, 0.0, &convexHullPoints[0]));

	// set material ID
	ndShapeMaterial childMaterial(childShapeInstance.GetMaterial());
	childMaterial.m_userId = ndApplicationMaterial::m_aiCar;
	childShapeInstance.SetMaterial(childMaterial);

	childShapeInstance.SetLocalMatrix(mChildLocal);
	shapeInstance.GetShape()->GetAsShapeCompound()->AddCollision(&childShapeInstance);
	shapeInstance.GetShape()->GetAsShapeCompound()->EndAddRemove();
#else
	ndShapeInstance shapeInstance(new ndShapeConvexHull(336 / 3, sizeof(ndFloat32) * 3, 0.0, &convexHullPoints[0]));
	shapeInstance.SetLocalMatrix(mChildLocal);
#endif

	// set material ID
	ndShapeMaterial material(shapeInstance.GetMaterial());
	material.m_userId = ndDemoContactCallback::m_aiCar;
	shapeInstance.SetMaterial(material);
	return shapeInstance;
}
#endif

class BackGroundVehicleController : public ndModel
{
	public:
	class ndNotify : public ndModelNotify
	{
		public:
		ndNotify(ndDemoEntityManager* pScene, const ndSharedPtr<ndBody>& body)
			:ndModelNotify()
			,m_scene(pScene)
			,m_vehicleBody(body)
			,m_desiredSpeed(3.0f) // 15 km/h
		{
		}

		void Update(ndFloat32 timestep)
		{
			if (IsOnGround())
			{
				ApplyTurningTorque();
				ApplyLateralForces(timestep);
				ApplyLongitudinalImpulse(timestep);
			}

			// Get the camera to follow the vehicle
			//ndVector origin(m_vehicleBody->GetPosition());
			//ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
			//origin.m_x += 10.0f;
			//m_scene->SetCameraMatrix(rot, origin);
		}

		private:
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

		void ApplyLongitudinalImpulse(ndFloat32 timestep)
		{
			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();

			const ndMatrix matrix(vehicleBody->GetMatrix());
			const ndVector velocity(m_vehicleBody->GetVelocity());
			ndFloat32 speed = matrix.m_front.DotProduct(velocity).GetScalar();
			ndFloat32 speedError = m_desiredSpeed - speed;
			ndFloat32 impulseMag = 0.4f * speedError * vehicleBody->GetMassMatrix().m_w;
			vehicleBody->ApplyImpulsePair(matrix.m_front.Scale (impulseMag), ndVector::m_zero, timestep); 
		}

		void ApplyLateralForces(ndFloat32 timestep)
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

		void ApplyTurningTorque()
		{
			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();
			const ndMatrix matrix(vehicleBody->GetMatrix());
			const ndVector forward(matrix.m_front);
			const ndVector upDir(matrix.m_up);

			ndVector velocity = m_vehicleBody->GetVelocity();
			ndFloat32 speed2 = velocity.DotProduct(velocity).GetScalar();

			// Work out steering forces
			ndFloat32 steering = 0.0f;               // Keep the vehicle traveling straight
			ndFloat32 maxTorque = 50000.0f;
			ndFloat32 desiredAngularSpeedFactor = 1.0f;
			ndFloat32 desiredAngularSpeed = desiredAngularSpeedFactor * speed2 * steering / (ndPi);   // Steering is a value from -pi to pi

			//if (m_currentSpeed < 0)
			//{
			//	// Flip desired angular speed for reversing
			//	desiredAngularSpeed *= -1;
			//}

			ndFloat32 angularSpeed = upDir.DotProduct(m_vehicleBody->GetOmega()).GetScalar();  // Component of the angular velocity about the up vector
			ndFloat32 angularSpeedDifference = desiredAngularSpeed - angularSpeed;
			angularSpeedDifference = ndClamp(angularSpeedDifference, ndFloat32(-0.3), ndFloat32(0.3));
			angularSpeedDifference = angularSpeedDifference / ndFloat32(0.3);  // Normalise to between -1 and 1;
			ndFloat32 torque = angularSpeedDifference * maxTorque;
			const ndVector turnTorque (upDir.Scale(torque));                     // Vehicle space torque
			vehicleBody->SetTorque(vehicleBody->GetTorque() + turnTorque);
		}

		ndDemoEntityManager* m_scene;
		ndSharedPtr<ndBody> m_vehicleBody;
		ndFloat32 m_desiredSpeed;
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
	//ndShapeHeightfield* const heighfield = mapBody->GetAsBodyKinematic()->GetCollisionShape().GetShape()->GetAsShapeHeightfield();
	//ndMatrix heighfieldLocation(ndGetIdentityMatrix());
	//heighfieldLocation.m_posit.m_x = -0.5f * ndFloat32(heighfield->GetWith()) * heighfield->GetWithScale();
	//heighfieldLocation.m_posit.m_z = -0.5f * ndFloat32(heighfield->GetHeight()) * heighfield->GetHeightScale();
	//mapBody->GetAsBodyKinematic()->SetMatrixUpdateScene(heighfieldLocation);
	//ndShapeMaterial mapMaterial(mapBody->GetAsBodyKinematic()->GetCollisionShape().GetMaterial());
	//mapMaterial.m_userId = ndDemoContactCallback::m_aiTerrain;
	//mapBody->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(mapMaterial);
	
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
