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
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoFlybyCameraNode.h"

class FrictionMaterial : public ndApplicationMaterial
{
	public:
	FrictionMaterial()
		:ndApplicationMaterial()
		,m_soundSpeed(10.0f)
	{
	}

	FrictionMaterial(const FrictionMaterial& src)
		:ndApplicationMaterial(src)
		,m_soundSpeed(10.0f)
	{
	}

	ndApplicationMaterial* Clone() const
	{
		return new FrictionMaterial(*this);
	}

	virtual void OnContactCallback(const ndContact* const joint, ndFloat32) const
	{
		const ndContactPointList& contactPoints = joint->GetContactPoints();
		ndFloat32 maxSpeed = 0.0f;
		ndBodyKinematic* const body = joint->GetBody0();
		for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
		{
			ndContactMaterial& contactPoint = contactPointsNode->GetInfo();
			ndFloat32 friction = contactPoint.m_shapeInstance0->m_shapeMaterial.m_userParam[ndDemoContactCallback::m_friction].m_floatData;
			contactPoint.m_material.m_staticFriction0 = friction;
			contactPoint.m_material.m_staticFriction1 = friction;
			contactPoint.m_material.m_dynamicFriction0 = friction;
			contactPoint.m_material.m_dynamicFriction1 = friction;
			ndVector contactVeloc(body->GetVelocityAtPoint(contactPoint.m_point));
			ndFloat32 speed = contactVeloc.DotProduct(contactPoint.m_normal).GetScalar();
			if (speed < maxSpeed)
			{
				maxSpeed = speed;
			}
		}

		if (-maxSpeed > m_soundSpeed)
		{
			// play sound here;
		}
	}

	ndFloat32 m_soundSpeed;
};

static void BuildFrictionRamp(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();
	
	FrictionMaterial material;
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_frictionTest, ndDemoContactCallback::m_default);
	
	ndMatrix matrix(ndPitchMatrix(30.0f * ndDegreeToRad));
	matrix.m_posit.m_y = 5.0f;
	ndSharedPtr<ndBody> ramp (AddBox(scene, matrix, ndFloat32 (0.0f), ndFloat32(30.0f), ndFloat32(0.25f), ndFloat32(30.0f), "wood_3.png"));
	
	ndVector boxSize(1.0f, 0.5f, 1.5f, 0.0f);
	matrix.m_posit.m_z -= 10.0f;
	matrix.m_posit.m_x -= 12.0f;
	ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + boxSize.m_y;
	
	for (ndInt32 i = 0; i < 10; ++i)
	{
		ndSharedPtr<ndBody> boxBody(AddBox(scene, matrix, ndFloat32(5.0f), boxSize.m_x, boxSize.m_y, boxSize.m_z, "wood_0.png"));

		matrix.m_posit.m_x += 2.5f;
	
		// assign material id to the collision shape.
		ndShapeInstance& instanceShape = boxBody->GetAsBodyKinematic()->GetCollisionShape();
		instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_frictionTest;
	
		// save fiction coefficient
		ndFloat32 frictionValue = ndFloat32(i) / 15.0f;
		instanceShape.m_shapeMaterial.m_userParam[ndDemoContactCallback::m_friction].m_floatData = frictionValue;
	}
}

void ndBasicFriction (ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> floor (BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	//ndSharedPtr<ndBody> BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	BuildFrictionRamp(scene);

	ndMatrix camMatrix(ndRollMatrix(10.0f * ndDegreeToRad) *  ndYawMatrix(20.0f * ndDegreeToRad));
	ndQuaternion rot(camMatrix);
	ndVector origin(-40.0f, 10.0f, 15.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
