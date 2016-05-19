#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"

#include "OpenGlUtil.h"


class MyTriggerManager: public CustomTriggerManager
{
	public:
	class TriggerCallback
	{
		public:
		TriggerCallback(CustomTriggerController* const controller)
			:m_controller(controller)
		{
		}

		virtual ~TriggerCallback()
		{
		}

		virtual void OnEnter(NewtonBody* const visitor)
		{
		}

		virtual void OnInside(NewtonBody* const visitor)
		{
		}

		virtual void OnExit(NewtonBody* const visitor)
		{
		}

		CustomTriggerController* m_controller;
	};

	class BuoyancyForce: public TriggerCallback
	{
		public:
		BuoyancyForce(CustomTriggerController* const controller)
			:TriggerCallback (controller)
			,m_plane(0.0f)
			,m_waterToSolidVolumeRatio(0.9f)
		{
			// get the fluid plane for the upper face of the trigger volume
			//NewtonBody* const body = m_controller->GetBody();
			m_plane = dVector (0.0f, 1.0f, 0.0f, 1.5f);
		}

		void OnInside(NewtonBody* const visitor)
		{
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			
			NewtonBodyGetMass(visitor, &mass, &Ixx, &Iyy, &Izz);
			if (mass > 0.0f) {
				dMatrix matrix;
				dVector cog(0.0f);
				dVector accelPerUnitMass(0.0f);
				dVector torquePerUnitMass(0.0f);
				const dVector gravity (0.0f, DEMO_GRAVITY, 0.0f, 0.0f);

				NewtonBodyGetMatrix (visitor, &matrix[0][0]);
				NewtonBodyGetCentreOfMass(visitor, &cog[0]);
				cog = matrix.TransformVector (cog);
				NewtonCollision* const collision = NewtonBodyGetCollision(visitor);

				
				dFloat shapeVolume = NewtonConvexCollisionCalculateVolume (collision);
				dFloat fluidDentity = 1.0f / (m_waterToSolidVolumeRatio * shapeVolume);
				dFloat viscosity = 0.995f;

				NewtonConvexCollisionCalculateBuoyancyAcceleration (collision, &matrix[0][0], &cog[0], &gravity[0], &m_plane[0], fluidDentity, viscosity, &accelPerUnitMass[0], &torquePerUnitMass[0]);

				dVector force (accelPerUnitMass.Scale (mass));
				dVector torque (torquePerUnitMass.Scale (mass));

				dVector omega(0.0f); 
				NewtonBodyGetOmega(visitor, &omega[0]);
				omega = omega.Scale (viscosity);
				NewtonBodySetOmega(visitor, &omega[0]);

				NewtonBodyAddForce (visitor, &force[0]);
				NewtonBodyAddTorque (visitor, &torque[0]);
			}
		}

		dVector m_plane;
		dFloat m_waterToSolidVolumeRatio;
	};



	MyTriggerManager(NewtonWorld* const world)
		:CustomTriggerManager(world)
	{
	}

	void CreateBuoyancyTrigger (const dMatrix& matrix, NewtonCollision* const convexShape)
	{
		CustomTriggerController* const controller = CreateTrigger (matrix, convexShape, NULL);
		BuoyancyForce* const buoyancyForce = new BuoyancyForce (controller);
		controller->SetUserData (buoyancyForce);
	}

	void DestroyController (CustomTriggerController* const controller)
	{
		TriggerCallback* const userData = (TriggerCallback*) controller->GetUserData();
		delete userData;
		CustomTriggerManager::DestroyController (controller);
	}
	

	virtual void EventCallback (const CustomTriggerController* const me, TriggerEventType event, NewtonBody* const visitor) const
	{
		TriggerCallback* const callback = (TriggerCallback*) me->GetUserData();
		switch (event) 
		{
			case m_enterTrigger:
			{
				callback->OnEnter(visitor);
				break;
			}

			case m_exitTrigger:
			{
				callback->OnExit(visitor);
				break;
			}

			case m_inTrigger:
			{
				callback->OnInside(visitor);
				break;
			}
		}
	}
};



void AlchimedesBuoyancy(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();


	// load the mesh 
	CreateLevelMesh (scene, "swimmingPool.ngd", true);


	// add a trigger Manager to the world
	MyTriggerManager* const triggerManager = new MyTriggerManager(scene->GetNewton());

	dMatrix triggerLocation (dGetIdentityMatrix());
	triggerLocation.m_posit.m_x =  17.0f;
	triggerLocation.m_posit.m_y = -3.5f;

	NewtonCollision* const poolBox = NewtonCreateBox (scene->GetNewton(), 30.0f, 6.0f, 20.0f, 0, NULL);  
	triggerManager->CreateBuoyancyTrigger (triggerLocation, poolBox);
	NewtonDestroyCollision (poolBox);

	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
	dMatrix offsetMatrix (dGetIdentityMatrix());

	// place camera into position
	dMatrix camMatrix (dGetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-20.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());

/*
	//test buoyancy on scaled collisions
	dVector plane (0.0f, 1.0f, 0.0f, 0.0f);
	dMatrix L1 (dPitchMatrix(30.0f * 3.141692f / 180.0f) * dYawMatrix(0.0f * 3.141692f / 180.0f) * dRollMatrix(0.0f * 3.141692f / 180.0f));
	NewtonCollision* xxx0 = NewtonCreateCompoundCollision(scene->GetNewton(), 0);
	NewtonCompoundCollisionBeginAddRemove(xxx0);
	NewtonCollision* xxxx0 = NewtonCreateBox(scene->GetNewton(), 1.0f, 2.0f, 1.0f, 0, &L1[0][0]);
	NewtonCompoundCollisionAddSubCollision(xxx0, xxxx0);
	NewtonCompoundCollisionEndAddRemove(xxx0);

	NewtonCollision* xxx1 = NewtonCreateCompoundCollision(scene->GetNewton(), 0);
	NewtonCollision* xxxx1 = NewtonCreateBox(scene->GetNewton(), 1.0f, 1.0f, 1.0f, 0, &L1[0][0]);
	NewtonCompoundCollisionAddSubCollision(xxx1, xxxx1);
	NewtonCompoundCollisionEndAddRemove(xxx1);
	NewtonCollisionSetScale(xxx1, 1.0f, 2.0f, 1.0f);

	//dMatrix m (dPitchMatrix(45.0f * 3.141692f / 180.0f) * dYawMatrix(40.0f * 3.141692f / 180.0f) * dRollMatrix(70.0f * 3.141692f / 180.0f));
	dMatrix m (dPitchMatrix(0.0f * 3.141692f / 180.0f) * dYawMatrix(0.0f * 3.141692f / 180.0f) * dRollMatrix(0.0f * 3.141692f / 180.0f));

	dVector gravity (0.0f, 0.0f, -9.8f, 0.0f);
	dVector cog0 (0.0f, 0.0f, 0.0f, 0.0f);
	dVector accelPerUnitMass0;
	dVector torquePerUnitMass0;
	NewtonConvexCollisionCalculateBuoyancyAcceleration (xxx0, &m[0][0], &cog0[0], &gravity[0], &plane[0], 1.0f, 0.1f, &accelPerUnitMass0[0], &torquePerUnitMass0[0]);

	dVector cog1 (0.0f, 0.0f, 0.0f, 0.0f);
	dVector accelPerUnitMass1;
	dVector torquePerUnitMass1;
	NewtonConvexCollisionCalculateBuoyancyAcceleration (xxx1, &m[0][0], &cog1[0], &gravity[0], &plane[0], 1.0f, 0.1f, &accelPerUnitMass1[0], &torquePerUnitMass1[0]);
*/

	int count = 5;
	dVector size (1.0f, 0.25f, 0.5f);
	dVector location (10.0f, 0.0f, 0.0f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());

//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

/*
for (NewtonBody* bodyPtr = NewtonWorldGetFirstBody(scene->GetNewton()); bodyPtr; bodyPtr = NewtonWorldGetNextBody(scene->GetNewton(), bodyPtr)) {
	NewtonCollision* collision = NewtonBodyGetCollision(bodyPtr);
	if (NewtonCollisionGetType(collision) == SERIALIZE_ID_COMPOUND) {
		NewtonCollisionSetScale (collision, 0.5f, 0.5f, 0.5f);
	}
}
*/

//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}