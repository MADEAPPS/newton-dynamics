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
			,m_desiredSpeed(4.1667f)      // 4.1667f m/s = 15 km/h (9.3 mph) 33.3333 m/s = 120km/h (74.6mph)
			,m_currentSpeed(0.0f)
			,m_dSpeedProportional(4000.0f)
			,m_integral(0.0f)
			,m_integralGain(3000.0f)
			,m_derivativeGain(600.0f)
			,m_previousError(0.0f)
			,m_combinedMaximumForce()
		{
			// Mass * Maximum acceleration of 6m/s/s
			m_combinedMaximumForce = body->GetAsBodyDynamic()->GetMassMatrix().m_w * ndFloat32(6.0);
		}

		void Update(ndFloat32 timestep)
		{
			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();

			ndMatrix matrix(ndCalculateMatrix(m_vehicleBody->GetRotation(), ndVector::m_wOne));
			ndVector forward = matrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0)));
			ndVector velocity = m_vehicleBody->GetVelocity();

			m_currentSpeed = forward.DotProduct(velocity).GetScalar();
			ndFloat32 speedDifference = m_desiredSpeed - m_currentSpeed;
			ndFloat32 longForceMag (UpdatePIDForDriveForces(speedDifference, timestep));

			longForceMag = ndClamp(longForceMag, -m_combinedMaximumForce, m_combinedMaximumForce);
			ndVector engineForce(ndFloat32(0.0), ndFloat32(0.0), longForceMag, ndFloat32(0.0));
			ndVector localOffset(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0));
			ndVector globalSpaceOffset = matrix.TransformVector(localOffset);      // Offset in local space
			ndVector globalSpaceForce = matrix.TransformVector(engineForce);      // Force in local space

			ndVector force(vehicleBody->GetForce() + globalSpaceForce);
			ndVector torque(vehicleBody->GetTorque() + (globalSpaceOffset.CrossProduct(globalSpaceForce)));

			vehicleBody->SetForce(force);
			vehicleBody->SetTorque(torque);

			ApplyLateralForces(timestep);

			// Get the camera to follow the vehicle
			//ndVector origin(m_vehicleBody->GetPosition());
			//ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
			//origin.m_x += 10.0f;
			//m_scene->SetCameraMatrix(rot, origin);
		}

		private:
		ndFloat32 UpdatePIDForDriveForces(ndFloat32 error, ndFloat32 dTimestep)
		{
			ndFloat32 proportional = m_dSpeedProportional * error;
			m_integral = m_integral + m_integralGain * error * dTimestep;

			// Make sure our integral remains inside reasonable bounds relative to the maximum force we are supposed to use
			m_integral = ndClamp(m_integral, -m_combinedMaximumForce, m_combinedMaximumForce);

			// Reset integral so that we don't overshoot stopping, and don't move slightly when our speed should be zero.
			if (abs(m_desiredSpeed) < 0.01 && abs(m_currentSpeed) < 0.01)
			{
				m_integral = 0;
			}

			ndFloat32 derivative = m_derivativeGain * (error - m_previousError) / dTimestep;
			ndFloat32 output = m_integral + derivative + proportional;
			m_previousError = error;
			return output;
		}

		void ApplyLateralForces(ndFloat32 dTimestep)
		{
			ndBodyDynamic* const vehicleBody = m_vehicleBody->GetAsBodyDynamic();

			ndMatrix matrix(ndCalculateMatrix(vehicleBody->GetRotation()));
			ndVector forward = matrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0)));
			forward.m_w = ndFloat32(0.0);

			ndVector upDir (matrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0), ndFloat32(0.0))));
			upDir.m_w = ndFloat32(0.0);

			// Get some info about our current velocity
			ndVector velocity = m_vehicleBody->GetVelocity();
			ndFloat32 speed2 = velocity.DotProduct(velocity).GetScalar();
			// Work out lateral force to stop sliding (all in world space)
			ndVector sidewayVelocity = velocity - forward.Scale(velocity.DotProduct(forward).GetScalar()) - upDir.Scale(velocity.DotProduct(upDir).GetScalar());
			ndVector velocityUp = velocity - forward.Scale(velocity.DotProduct(forward).GetScalar()) - sidewayVelocity;
			ndVector sidewaysMomentum = sidewayVelocity.Scale(vehicleBody->GetMassMatrix().m_w);
			ndVector verticalMomentum = velocityUp.Scale(vehicleBody->GetMassMatrix().m_w);

			vehicleBody->ApplyImpulsePair(ndVector::m_negOne * sidewaysMomentum * 0.8f, ndVector(0.0f), dTimestep);      // 0.8 = momentum canceling factor
			vehicleBody->ApplyImpulsePair(ndVector::m_negOne * verticalMomentum * 0.0f, ndVector(0.0f), dTimestep);

			// Work out steering forces
			ndFloat32 steering = 0.0;               // Keep the vehicle traveling straight
			ndFloat32 maxTorque = 50000;
			ndFloat32 desiredAngularSpeedFactor = 1.0;
			ndFloat32 desiredAngularSpeed = desiredAngularSpeedFactor * speed2 * steering / (ndPi);   // Steering is a value from -pi to pi

			if (m_currentSpeed < 0)
			{
				// Flip desired angular speed for reversing
				desiredAngularSpeed *= -1;
			}

			ndFloat32 angularSpeed = upDir.DotProduct(m_vehicleBody->GetOmega()).GetScalar();  // Component of the angular velocity about the up vector
			ndFloat32 angularSpeedDifference = desiredAngularSpeed - angularSpeed;
			angularSpeedDifference = ndClamp(angularSpeedDifference, ndFloat32(-0.3), ndFloat32(0.3));
			angularSpeedDifference = angularSpeedDifference / ndFloat32(0.3);  // Normalise to between -1 and 1;
			ndFloat32 torque = angularSpeedDifference * maxTorque;
			ndVector yAxisMoment = ndVector(ndFloat32(0.0), torque, ndFloat32(0.0), ndFloat32(0.0));                        // Vehicle space torque
			ndVector worldMoment = matrix.TransformVector(yAxisMoment);      // Get the torque to world space
			vehicleBody->SetTorque(vehicleBody->GetTorque() + worldMoment);
		}

		ndDemoEntityManager* m_scene;
		ndSharedPtr<ndBody> m_vehicleBody;
		ndFloat32 m_desiredSpeed;
		ndFloat32 m_currentSpeed;
		ndFloat32 m_dSpeedProportional;
		ndFloat32 m_integral;
		ndFloat32 m_integralGain;
		ndFloat32 m_derivativeGain;
		ndFloat32 m_previousError;
		ndFloat32 m_combinedMaximumForce;
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

	ndSharedPtr<ndShapeInstance> collision(loader.m_mesh->CreateCollisionConvex());

	ndWorld* const world = scene->GetWorld();

	ndSharedPtr<ndBody> vehicleBody(new ndBodyDynamic());
	vehicleBody->SetNotifyCallback(new ndDemoEntityNotify(scene, vehicleMesh));
	vehicleBody->SetMatrix(matrix);
	vehicleBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
	vehicleBody->GetAsBodyDynamic()->SetMassMatrix(1000.0f, **collision);

	ndSharedPtr<ndModel> controller(new BackGroundVehicleController(scene, vehicleBody));

	world->AddBody(vehicleBody);
	scene->AddEntity(vehicleMesh);
	world->AddModel(controller);

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
