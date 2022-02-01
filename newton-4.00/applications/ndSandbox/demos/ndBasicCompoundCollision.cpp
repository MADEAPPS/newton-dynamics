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
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndHeightFieldPrimitive.h"
#include "ndMakeProceduralStaticMap.h"

#if 0
static ndBodyDynamic* AddRigidBody(
	ndDemoEntityManager* const scene,
	const ndMatrix& matrix, const ndShapeInstance& shape,
	ndDemoInstanceEntity* const rootEntity, ndFloat32 mass)
{
	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, rootEntity);

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);

	ndWorld* const world = scene->GetWorld();
	world->AddBody(body);
	return body;
}

static void AddToCompoundShape(const ndMatrix& mLocalMatrix, ndShapeInstance& parentShape, ndShapeInstance& childInstance)
{
	auto pCompoundShape = parentShape.GetShape()->GetAsShapeCompound();
	childInstance.SetLocalMatrix(mLocalMatrix);
	pCompoundShape->AddCollision(&childInstance);
}

void CreateBoxCompoundShape(ndShapeInstance& parentInstance)
{
	ndShapeInstance wall1(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall2(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance wall3(new ndShapeBox(2.0f, 2.0f, 0.1f));
	ndShapeInstance wall4(new ndShapeBox(0.1f, 2.0f, 2.0f));
	ndShapeInstance floor(new ndShapeBox(2.0f, 0.1f, 2.0f));
	ndMatrix mWall1Local = dGetIdentityMatrix();
	mWall1Local.m_posit = ndVector(0.0f, 0.0f, -1.0f, 1.0f);
	ndMatrix mWall2Local = dGetIdentityMatrix();
	mWall2Local.m_posit = ndVector(1.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix mWall3Local = dGetIdentityMatrix();
	mWall3Local.m_posit = ndVector(0.0f, 0.0f, 1.0f, 1.0f);
	ndMatrix mWall4Local = dGetIdentityMatrix();
	mWall4Local.m_posit = ndVector(-1.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix mFloorLocal = dGetIdentityMatrix();
	mFloorLocal = dYawMatrix(3.14f / 4.0f);//45 degree
	mFloorLocal.m_posit = ndVector(0.0f, -1.0f, 0.0f, 1.0f);

	auto pCompoundShape = parentInstance.GetShape()->GetAsShapeCompound();
	pCompoundShape->BeginAddRemove();
	AddToCompoundShape(mWall1Local, parentInstance, wall1);
	AddToCompoundShape(mWall2Local, parentInstance, wall2);
	AddToCompoundShape(mWall3Local, parentInstance, wall3);
	AddToCompoundShape(mWall4Local, parentInstance, wall4);
	AddToCompoundShape(mFloorLocal, parentInstance, floor);
	pCompoundShape->EndAddRemove();
}

void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene)
{
	ndMatrix heighfieldLocation(dGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	// build a floor
	//BuildPlayArena(scene);
	//BuildFlatPlane(scene, true);
	//BuildFloorBox(scene, dGetIdentityMatrix());
	//BuildCompoundScene(scene, dGetIdentityMatrix());
	//BuildGridPlane(scene, 120, 4.0f, 0.0f);
	//BuildHeightFieldTerrain(scene, heighfieldLocation);
	BuildProceduralMap(scene, 120, 4.0f, 0.0f);

	ndShapeInstance compoundShapeInstance(new ndShapeCompound());
	CreateBoxCompoundShape(compoundShapeInstance);
	compoundShapeInstance.SetScale(ndVector(2.0f, 1.0f, 1.0f, 0.0f));

	ndDemoMeshIntance* const compGeometry = new ndDemoMeshIntance("compoundShape", scene->GetShaderCache(), &compoundShapeInstance, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoInstanceEntity* const compEntity = new ndDemoInstanceEntity(compGeometry);
	scene->AddEntity(compEntity);
	ndMatrix mBodyMatrix = dGetIdentityMatrix();
	mBodyMatrix.m_posit = ndVector(-2.0f, 5.0f, -5.0f, 1.0f);
	AddRigidBody(scene, mBodyMatrix, compoundShapeInstance, compEntity, 10.0);

	ndShapeInstance originShape(new ndShapeSphere(0.5));
	ndDemoMeshIntance* const origGeometry = new ndDemoMeshIntance("origShape", scene->GetShaderCache(), &originShape, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoInstanceEntity* const origEntity = new ndDemoInstanceEntity(origGeometry);
	scene->AddEntity(origEntity);
	ndMatrix mOrigMatrix = dGetIdentityMatrix();
	mOrigMatrix.m_posit.m_y += 2.0f;
	AddRigidBody(scene, mOrigMatrix, originShape, origEntity, 5.0);

	ndVector origin(ndVector::m_zero);
	ndQuaternion rot(dYawMatrix(45.0f * ndDegreeToRad));
	origin.m_x -= 3.0f;
	origin.m_y += 5.0f;

	origin.m_x -= 15.0f;
	origin.m_z += 15.0f;

	scene->SetCameraMatrix(rot, origin);
}

#else

class CAiController : public ndModel
{
public:
	CAiController(ndDemoEntityManager* pScene, ndBodyDynamic* pBody)
		: m_pScene(pScene)
		, m_pAiBody(pBody)
		, m_dDesiredSpeed(4.1667f)      // 15 kilometers per hour (9.3 miles per hour)
		, m_dCurrentSpeed(0.0f)
		, m_dSpeedProportional(3000.0f)
		, m_dIntegral(0.0f)
		, m_dIntegralGain(3000.0f)
		, m_dDerivativeGain(600.0f)
		, m_dPreviousError(0.0f)
		, m_dCombinedMaximumForce()
	{
		m_dDesiredSpeed *= 2;
		m_dCombinedMaximumForce = pBody->GetMassMatrix().m_w * ndFloat32(6.0);      // Mass * Maximum acceleration of 6m/s/s
	}

protected:
	virtual void Update(ndWorld* const, ndFloat32 timestep) override
	{
		if (m_pAiBody->GetPosition().m_y < 1.5f)
		{
			ndMatrix mMatrix(m_pAiBody->GetRotation(), ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0)));
			ndVector vForward = mMatrix.TransformVector(ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0)));
			ndVector vVelocity = m_pAiBody->GetVelocity();
			m_dCurrentSpeed = (vForward.DotProduct(vVelocity)).GetScalar();
			ndFloat32 dSpeedDifference = m_dDesiredSpeed - m_dCurrentSpeed;
			ndFloat32 dForce = _UpdatePIDForDriveForces(dSpeedDifference, timestep);
			dForce = dClamp(dForce, -m_dCombinedMaximumForce, m_dCombinedMaximumForce);
			ndVector vForce(ndFloat32(0.0), ndFloat32(0.0), dForce, ndFloat32(0.0));
			ndVector vOffset(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0));
			ndVector vOffsetLS = mMatrix.TransformVector(vOffset);      // Offset in local space
			ndVector vForceLS = mMatrix.TransformVector(vForce);      // Force in local space
			m_pAiBody->SetForce(m_pAiBody->GetForce() + vForceLS);
			m_pAiBody->SetTorque(m_pAiBody->GetTorque() + (vOffsetLS.CrossProduct(vForceLS)));
			_ApplyLateralForces(timestep);
		}

		// Get the camera to follow the vehicle
		ndVector origin(m_pAiBody->GetPosition());
		ndQuaternion rot(dYawMatrix(180.0f * ndDegreeToRad));
		origin.m_x += 10.0f;
		m_pScene->SetCameraMatrix(rot, origin);
	}

private:
	ndFloat32 _UpdatePIDForDriveForces(ndFloat32 dError, ndFloat32 dTimestep)
	{
		ndFloat32 dProportional = m_dSpeedProportional * dError;
		m_dIntegral = m_dIntegral + m_dIntegralGain * dError * dTimestep;

		// Make sure our integral remains inside reasonable bounds relative to the maximum force we are supposed to use
		m_dIntegral = dClamp(m_dIntegral, -m_dCombinedMaximumForce, m_dCombinedMaximumForce);

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
		ndMatrix mMatrix(m_pAiBody->GetRotation(), ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0)));
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
		ndVector vSidewaysMomentum = vVelocitySideways.Scale(m_pAiBody->GetMassMatrix().m_w);
		ndVector vVerticalMomentum = vVelocityUp.Scale(m_pAiBody->GetMassMatrix().m_w);
		m_pAiBody->ApplyImpulsePair(ndVector::m_negOne * vSidewaysMomentum * 0.8f, ndVector(0.0f), dTimestep);      // 0.8 = momentum cancelling factor
		m_pAiBody->ApplyImpulsePair(ndVector::m_negOne * vVerticalMomentum * 0.0f, ndVector(0.0f), dTimestep);

		// Work out steering forces
		ndFloat32 dSteering = 0.0;               // Keep the vehicle travelling straight
		ndFloat32 dMaxTorque = 50000;
		ndFloat32 dDesiredAngularSpeedFactor = 1.0;
		ndFloat32 dDesiredAngularSpeed = dDesiredAngularSpeedFactor * dVelocity * dSteering / (ndPi);   // Steering is a value from -pi to pi

		if (m_dCurrentSpeed < 0)   // Flip desired angular speed for reversing
			dDesiredAngularSpeed *= -1;

		ndFloat32 dAngularSpeed = vUp.DotProduct(m_pAiBody->GetOmega()).GetScalar();            // Component of the angular velocity about the up vector
		ndFloat32 dAngularSpeedDifference = dDesiredAngularSpeed - dAngularSpeed;
		dAngularSpeedDifference = dClamp(dAngularSpeedDifference, ndFloat32(-0.3), ndFloat32(0.3));
		dAngularSpeedDifference = dAngularSpeedDifference / ndFloat32(0.3);               // Normalise to between -1 and 1;
		ndFloat32 dTorque = dAngularSpeedDifference * dMaxTorque;
		ndVector vYAxisMoment = ndVector(ndFloat32(0.0), dTorque, ndFloat32(0.0), ndFloat32(0.0));                        // Vehicle space torque
		ndVector vWorldMoment = mMatrix.TransformVector(vYAxisMoment);               // Get the torque to world space
		m_pAiBody->SetTorque(m_pAiBody->GetTorque() + vWorldMoment);
	}

	ndDemoEntityManager* m_pScene;
	ndBodyDynamic* m_pAiBody;
	ndFloat32 m_dDesiredSpeed;
	ndFloat32 m_dCurrentSpeed;
	ndFloat32 m_dSpeedProportional;
	ndFloat32 m_dIntegral;
	ndFloat32 m_dIntegralGain;
	ndFloat32 m_dDerivativeGain;
	ndFloat32 m_dPreviousError;
	ndFloat32 m_dCombinedMaximumForce;
};

static ndBodyDynamic* AddRigidBody(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndShapeInstance& shape, ndDemoInstanceEntity* const rootEntity, ndFloat32 mass)
{
	ndBodyDynamic* const pBody = new ndBodyDynamic();
	ndDemoEntity* const pEntity = new ndDemoEntity(matrix, rootEntity);
	pBody->SetNotifyCallback(new ndDemoEntityNotify(scene, pEntity));
	pBody->SetMatrix(matrix);
	pBody->SetCollisionShape(shape);
	pBody->SetMassMatrix(mass, shape);
	CAiController* pController = new CAiController(scene, pBody);


	ndWorld* const world = scene->GetWorld();
	world->AddModel(pController);
	world->AddBody(pBody);
	return pBody;
}

ndShapeInstance CreateCompondCollision()
{
	ndFloat32 convexHullPoints[336] = { 2.64974f,  0.903322f,  -0.766362f,  2.64974f,  0.903322f,  -0.0842047f,  0.677557f,  0.903322f,  0.662591f,  -0.596817f,  0.903322f,  0.662591f,  -2.77764f,  0.903321f,  0.0424131f,
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

	ndMatrix mParentInv = dGetIdentityMatrix();
	mParentInv.m_posit = ndVector(ndFloat32(0.0), ndFloat32(-1.0), ndFloat32(0.0), ndFloat32(1.0));
	ndMatrix mChildWorld = dGetIdentityMatrix();
	mChildWorld.m_front = ndVector(ndFloat32(0.0), ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0));
	mChildWorld.m_up = ndVector(ndFloat32(1.0), ndFloat32(0.0), ndFloat32(0.0), ndFloat32(0.0));
	mChildWorld.m_right = ndVector(ndFloat32(0.0), ndFloat32(1.0), ndFloat32(0.0), ndFloat32(0.0));
	mChildWorld.m_posit = ndVector(ndFloat32(0.0), ndFloat32(1.158), ndFloat32(-0.508), ndFloat32(1.0));
	ndMatrix mChildLocal = mChildWorld * mParentInv;

#if 1
	ndShapeInstance shapeInstance(new ndShapeCompound());
	shapeInstance.GetShape()->GetAsShapeCompound()->BeginAddRemove();
	ndShapeInstance childShapeInstance(new ndShapeConvexHull(336 / 3, sizeof(ndFloat32) * 3, 0.0, &convexHullPoints[0]));
	childShapeInstance.SetLocalMatrix(mChildLocal);
	shapeInstance.GetShape()->GetAsShapeCompound()->AddCollision(&childShapeInstance);
	shapeInstance.GetShape()->GetAsShapeCompound()->EndAddRemove();
#else
	ndShapeInstance shapeInstance(new ndShapeConvexHull(336 / 3, sizeof(ndFloat32) * 3, 0.0, &convexHullPoints[0]));
	shapeInstance.SetLocalMatrix(mChildLocal);
#endif
	return shapeInstance;
}

ndShapeInstance CreateBoxCollision()
{
	return ndShapeInstance(new ndShapeBox(1.0, 0.5, 2.0));
}

static void AddAiVehicle(ndDemoEntityManager* const scene)
{
	//ndShapeInstance shapeInstance = CreateBoxCollision();
	ndShapeInstance shapeInstance = CreateCompondCollision();

	ndDemoMeshIntance* const aiGeometry = new ndDemoMeshIntance("AiVehicle", scene->GetShaderCache(), &shapeInstance, "earthmap.tga", "earthmap.tga", "earthmap.tga");
	ndDemoInstanceEntity* const aiEntity = new ndDemoInstanceEntity(aiGeometry);
	scene->AddEntity(aiEntity);
	ndMatrix mBodyMatrix = dGetIdentityMatrix();
	mBodyMatrix.m_posit = ndVector(0.0f, 5.0f, 0.0f, 1.0f);
	//mBodyMatrix.m_posit = ndVector(0.0f, 1.2f, 0.0f, 1.0f);
	AddRigidBody(scene, mBodyMatrix, shapeInstance, aiEntity, 1000.0);
}

//void ndBasicAiDemo(ndDemoEntityManager* const scene)
void ndBasicCompoundShapeDemo(ndDemoEntityManager* const scene)
{
	BuildProceduralMap(scene, 100, 2.0f, 0.0f);
	AddAiVehicle(scene);
}
#endif