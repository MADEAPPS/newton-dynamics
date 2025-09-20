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


class BackGroundVehicleController : public ndModel
{
	public:
	class ndNotify : public ndModelNotify
	{
		public:
		ndNotify(ndDemoEntityManager* pScene, const ndSharedPtr<ndBody>& body)
			:ndModelNotify()
			,m_pScene(pScene)
			,m_pAiBody(body)
			,m_dDesiredSpeed(4.1667f)      // 4.1667f m/s = 15 km/h (9.3 mph) 33.3333 m/s = 120km/h (74.6mph)
			,m_dCurrentSpeed(0.0f)
			,m_dSpeedProportional(4000.0f)
			,m_dIntegral(0.0f)
			,m_dIntegralGain(3000.0f)
			,m_dDerivativeGain(600.0f)
			,m_dPreviousError(0.0f)
			,m_dCombinedMaximumForce()
		{
			// Mass * Maximum acceleration of 6m/s/s
			m_dCombinedMaximumForce = body->GetAsBodyDynamic()->GetMassMatrix().m_w * ndFloat32(6.0);
		}

		void Update(ndFloat32 timestep)
		{
			ndBodyDynamic* const pAiBody = m_pAiBody->GetAsBodyDynamic();
			ndMatrix mMatrix(ndCalculateMatrix(m_pAiBody->GetRotation(), ndVector::m_wOne));
			ndVector vForward = mMatrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0)));
			ndVector vVelocity = m_pAiBody->GetVelocity();
			m_dCurrentSpeed = (vForward.DotProduct(vVelocity)).GetScalar();
			ndFloat32 dSpeedDifference = m_dDesiredSpeed - m_dCurrentSpeed;
			ndFloat32 dForce = _UpdatePIDForDriveForces(dSpeedDifference, timestep);
			dForce = ndClamp(dForce, -m_dCombinedMaximumForce, m_dCombinedMaximumForce);
			ndVector vForce(ndFloat32(0.0), ndFloat32(0.0), dForce, ndFloat32(0.0));
			ndVector vOffset(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0));
			ndVector vOffsetLS = mMatrix.TransformVector(vOffset);      // Offset in local space
			ndVector vForceLS = mMatrix.TransformVector(vForce);      // Force in local space

			ndVector force(pAiBody->GetForce() + vForceLS);
			ndVector torque(pAiBody->GetTorque() + (vOffsetLS.CrossProduct(vForceLS)));
			pAiBody->SetForce(force);
			pAiBody->SetTorque(torque);

			_ApplyLateralForces(timestep);
			
			// Get the camera to follow the vehicle
			ndVector origin(m_pAiBody->GetPosition());
			ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
			origin.m_x += 10.0f;
			m_pScene->SetCameraMatrix(rot, origin);
		}

		private:
		ndFloat32 _UpdatePIDForDriveForces(ndFloat32 dError, ndFloat32 dTimestep)
		{
			ndFloat32 dProportional = m_dSpeedProportional * dError;
			m_dIntegral = m_dIntegral + m_dIntegralGain * dError * dTimestep;
			
			// Make sure our integral remains inside reasonable bounds relative to the maximum force we are supposed to use
			m_dIntegral = ndClamp(m_dIntegral, -m_dCombinedMaximumForce, m_dCombinedMaximumForce);
			
			// Reset integral so that we don't overshoot stopping, and don't move slightly when our speed should be zero.
			if (abs(m_dDesiredSpeed) < 0.01 && abs(m_dCurrentSpeed) < 0.01)
				m_dIntegral = 0;
			
			ndFloat32 dDerivative = m_dDerivativeGain * (dError - m_dPreviousError) / dTimestep;
			ndFloat32 dOutput = m_dIntegral + dDerivative + dProportional;
			m_dPreviousError = dError;
			return dOutput;
		}

		void _ApplyLateralForces(ndFloat32 dTimestep)
		{
			ndBodyDynamic* const pAiBody = m_pAiBody->GetAsBodyDynamic();
			ndMatrix mMatrix(ndCalculateMatrix(pAiBody->GetRotation()));
			ndVector vForward = mMatrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0)));
			vForward.m_w = ndFloat32(0.0);
			ndVector vUp = mMatrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0), ndFloat32(0.0)));
			vUp.m_w = ndFloat32(0.0);
			// Get some info about our current velocity
			ndVector vVelocity = m_pAiBody->GetVelocity();
			ndFloat32 dVelocity = vVelocity.DotProduct(vVelocity).GetScalar();
			// Work out lateral force to stop sliding (all in world space)
			ndVector vVelocitySideways = vVelocity - vForward.Scale(vVelocity.DotProduct(vForward).GetScalar()) - vUp.Scale(vVelocity.DotProduct(vUp).GetScalar());
			ndVector vVelocityUp = vVelocity - vForward.Scale(vVelocity.DotProduct(vForward).GetScalar()) - vVelocitySideways;
			ndVector vSidewaysMomentum = vVelocitySideways.Scale(pAiBody->GetMassMatrix().m_w);
			ndVector vVerticalMomentum = vVelocityUp.Scale(pAiBody->GetMassMatrix().m_w);
			pAiBody->ApplyImpulsePair(ndVector::m_negOne * vSidewaysMomentum * 0.8f, ndVector(0.0f), dTimestep);      // 0.8 = momentum canceling factor
			pAiBody->ApplyImpulsePair(ndVector::m_negOne * vVerticalMomentum * 0.0f, ndVector(0.0f), dTimestep);
		
			// Work out steering forces
			ndFloat32 dSteering = 0.0;               // Keep the vehicle traveling straight
			ndFloat32 dMaxTorque = 50000;
			ndFloat32 dDesiredAngularSpeedFactor = 1.0;
			ndFloat32 dDesiredAngularSpeed = dDesiredAngularSpeedFactor * dVelocity * dSteering / (ndPi);   // Steering is a value from -pi to pi
		
			if (m_dCurrentSpeed < 0)   // Flip desired angular speed for reversing
				dDesiredAngularSpeed *= -1;
		
			ndFloat32 dAngularSpeed = vUp.DotProduct(m_pAiBody->GetOmega()).GetScalar();  // Component of the angular velocity about the up vector
			ndFloat32 dAngularSpeedDifference = dDesiredAngularSpeed - dAngularSpeed;
			dAngularSpeedDifference = ndClamp(dAngularSpeedDifference, ndFloat32(-0.3), ndFloat32(0.3));
			dAngularSpeedDifference = dAngularSpeedDifference / ndFloat32(0.3);  // Normalise to between -1 and 1;
			ndFloat32 dTorque = dAngularSpeedDifference * dMaxTorque;
			ndVector vYAxisMoment = ndVector(ndFloat32(0.0), dTorque, ndFloat32(0.0), ndFloat32(0.0));                        // Vehicle space torque
			ndVector vWorldMoment = mMatrix.TransformVector(vYAxisMoment);      // Get the torque to world space
			pAiBody->SetTorque(pAiBody->GetTorque() + vWorldMoment);
		}

		ndDemoEntityManager* m_pScene;
		ndSharedPtr<ndBody> m_pAiBody;
		ndFloat32 m_dDesiredSpeed;
		ndFloat32 m_dCurrentSpeed;
		ndFloat32 m_dSpeedProportional;
		ndFloat32 m_dIntegral;
		ndFloat32 m_dIntegralGain;
		ndFloat32 m_dDerivativeGain;
		ndFloat32 m_dPreviousError;
		ndFloat32 m_dCombinedMaximumForce;
	};

	BackGroundVehicleController(ndDemoEntityManager* pScene, const ndSharedPtr<ndBody>& body)
		:ndModel()
	{
		SetNotifyCallback(ndSharedPtr<ndModelNotify>(new ndNotify(pScene, body)));
	}
};


static ndShapeInstance CreateCompoundCollision()
{
	ndFloat32 convexHullPoints[336] = { 
		2.64974f,  0.903322f,  -0.766362f,  2.64974f,  0.903322f,  -0.0842047f,  0.677557f,  0.903322f,  0.662591f,  -0.596817f,  0.903322f,  0.662591f,  -2.77764f,  0.903321f,  0.0424131f,
		-2.77764f,  0.903321f,  -0.766362f,  -1.56248f,  0.903322f,  -0.766362f,  -0.784835f,  0.903322f,  -0.766362f,  1.46171f,  0.903322f,  -0.766362f,  2.22016f,  0.903322f,  -0.766362f,
		-2.77764f,  -0.918637f,  -0.766362f,  -2.77764f,  -0.918637f,  0.0424131f,  -0.596817f,  -0.918637f,  0.662591f,  0.677557f,  -0.918636f,  0.662591f,  2.64974f,  -0.918636f,  -0.0842047f,
		-1.56248f,  -0.918637f,  -0.766362f,  -0.784834f,  -0.918637f,  -0.766362f,  1.46171f,  -0.918636f,  -0.766362f,  2.22016f,  -0.918636f,  -0.766362f,  2.64974f,  -0.918636f,  -0.766362f,
		2.22016f,  -0.654416f,  -0.766362f,  2.22016f,  -0.918636f,  -0.766362f,  2.64974f,  -0.918636f,  -0.766362f,  2.22016f,  0.661676f,  -0.766362f,  2.22016f,  0.903322f,  -0.766362f,
		2.64974f,  0.903322f,  -0.766362f,  -2.77764f,  0.903321f,  -0.766362f,  -2.77764f,  0.903321f,  0.0424131f,  -2.77764f,  -0.918637f,  0.0424131f,  -2.77764f,  -0.918637f,  -0.766362f,
		-2.77764f,  0.903321f,  0.0424131f,  -0.596817f,  0.903322f,  0.662591f,  -0.596817f,  -0.918637f,  0.662591f,  -2.77764f,  -0.918637f,  0.0424131f,  2.64974f,  0.903322f,  -0.0842047f,
		2.64974f,  0.903322f,  -0.766362f,  2.64974f,  -0.918636f,  -0.766362f,  2.64974f,  -0.918636f,  -0.0842047f,  0.677557f,  0.903322f,  0.662591f,  2.64974f,  0.903322f,  -0.0842047f,
		2.64974f,  -0.918636f,  -0.0842047f,  0.677557f,  -0.918636f,  0.662591f,  -2.77764f,  -0.918637f,  -0.766362f,  -1.56248f,  -0.918637f,  -0.766362f,  -1.56248f,  -0.654416f,  -0.766362f,
		-2.77764f,  0.903321f,  -0.766362f,  -1.56248f,  0.661676f,  -0.766362f,  -1.56248f,  0.903322f,  -0.766362f,  -0.784834f,  -0.654416f,  -0.766362f,  -0.784835f,  0.661676f,  -0.766362f,
		-0.784834f,  -0.918637f,  -0.766362f,  1.46171f,  -0.918636f,  -0.766362f,    1.46171f,  -0.654416f,  -0.766362f,  1.46171f,  0.661676f,  -0.766362f,  -0.784835f,  0.903322f,  -0.766362f,
		1.46171f,  0.903322f,  -0.766362f,  -0.784834f,  -0.654416f,  -0.766362f,  -1.56248f,  -0.654416f,  -0.766362f,  -1.17366f,  -0.654417f,  -1.15761f,  -1.56248f,  -0.654416f,  -0.766362f,
		-1.56248f,  -0.918637f,  -0.766362f,  -1.17366f,  -0.918637f,  -1.15761f,  -1.17366f,  -0.654417f,  -1.15761f,  -1.56248f,  -0.918637f,  -0.766362f,  -0.784834f,  -0.918637f,  -0.766362f,
		-1.17366f,  -0.918637f,  -1.15761f,  -0.784834f,  -0.918637f,  -0.766362f,  -0.784834f,  -0.654416f,  -0.766362f,  -1.17366f,  -0.654417f,  -1.15761f,  -1.17366f,  -0.918637f,  -1.15761f,
		-1.56248f,  0.661676f,  -0.766362f,  -0.784835f,  0.661676f,  -0.766362f,  -1.17366f,  0.661676f,  -1.15761f,  -0.784835f,  0.661676f,  -0.766362f,  -0.784835f,  0.903322f,  -0.766362f,
		-1.17366f,  0.903322f,  -1.15761f,  -1.17366f,  0.661676f,  -1.15761f,  -0.784835f,  0.903322f,  -0.766362f,  -1.56248f,  0.903322f,  -0.766362f,  -1.17366f,  0.903322f,  -1.15761f,
		-1.56248f,  0.903322f,  -0.766362f,  -1.56248f,  0.661676f,  -0.766362f,  -1.17366f,  0.661676f,  -1.15761f,  -1.17366f,  0.903322f,  -1.15761f,  2.22016f,  -0.654416f,  -0.766362f,
		1.46171f,  -0.654416f,  -0.766362f,  1.84093f,  -0.654416f,  -1.15761f,  1.46171f,  -0.654416f,  -0.766362f,  1.46171f,  -0.918636f,  -0.766362f,  1.84093f,  -0.918636f,  -1.15761f,
		1.84093f,  -0.654416f,  -1.15761f,  1.46171f,  -0.918636f,  -0.766362f,  2.22016f,  -0.918636f,  -0.766362f,  1.84093f,  -0.918636f,  -1.15761f,  2.22016f,  -0.918636f,  -0.766362f,
		2.22016f,  -0.654416f,  -0.766362f,  1.84093f,  -0.654416f,  -1.15761f,  1.84093f,  -0.918636f,  -1.15761f,  1.46171f,  0.661676f,  -0.766362f,  2.22016f,  0.661676f,  -0.766362f,
		1.84093f,  0.661676f,  -1.15761f,  2.22016f,  0.661676f,  -0.766362f,  2.22016f,  0.903322f,  -0.766362f,  1.84093f,  0.903322f,  -1.15761f,  1.84093f,  0.661676f,  -1.15761f,
		2.22016f,  0.903322f,  -0.766362f,  1.46171f,  0.903322f,  -0.766362f,  1.84093f,  0.903322f,  -1.15761f,  1.46171f,  0.903322f,  -0.766362f,  1.46171f,  0.661676f,  -0.766362f,
		1.84093f,  0.661676f,  -1.15761f,  1.84093f,  0.903322f,  -1.15761f };

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

static ndSharedPtr<ndBody> AddAiVehicle(ndDemoEntityManager* const scene)
{
	//ndShapeInstance shapeInstance = CreateCompoundCollision();
	//ndDemoMeshIntance* const aiGeometry = new ndDemoMeshIntance("AiVehicle", scene->GetShaderCache(), &shapeInstance, "earthmap.png", "earthmap.png", "earthmap.png");
	//ndSharedPtr<ndDemoEntity> aiEntity(new ndDemoInstanceEntity(aiGeometry));
	//scene->AddEntity(aiEntity);
	//ndMatrix mBodyMatrix = ndGetIdentityMatrix();
	//mBodyMatrix.m_posit = ndVector(0.0f, 5.0f, 0.0f, 1.0f);
	////mBodyMatrix.m_posit = ndVector(0.0f, 1.2f, 0.0f, 1.0f);
	 
	ndMeshLoader loader;
	ndSharedPtr<ndRenderSceneNode> vehicleMesh(loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("vmw.fbx")));
	ndMatrix matrix (ndGetIdentityMatrix());
	matrix.m_posit = ndVector(0.0f, 5.0f, 0.0f, 1.0f);

	ndSharedPtr<ndShapeInstance> collision(loader.m_mesh->CreateCollisionConvex());
	//ndSharedPtr<ndBody> aiVehicleBody (AddRigidBody(scene, matrix, **collision, vehicle, 1000.0f));

	ndWorld* const world = scene->GetWorld();

	ndSharedPtr<ndBody> vehicleBody(new ndBodyDynamic());
	vehicleBody->SetNotifyCallback(new ndDemoEntityNotify(scene, vehicleMesh));
	vehicleBody->SetMatrix(matrix);
	vehicleBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
	vehicleBody->GetAsBodyDynamic()->SetMassMatrix(1000.0f, **collision);

	world->AddBody(vehicleBody);
	scene->AddEntity(vehicleMesh);
	//world->AddModel(controller);

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
	
	ndSharedPtr<ndBody> vehuicleBody(AddAiVehicle(scene));

	//ndVector origin(pAiBody->GetPosition());
	//ndQuaternion rot(ndYawMatrix(180.0f * ndDegreeToRad));
	//origin.m_x += 10.0f;
	//scene->SetCameraMatrix(rot, origin);

	ndVector floor(FindFloor(*scene->GetWorld(), ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	ndQuaternion rot(ndYawMatrix(0.0f * ndDegreeToRad));

	floor.m_x -= 10.0f;
	floor.m_y += 2.0f;
	scene->SetCameraMatrix(rot, floor);
}
