#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"

#include "../toolBox/OpenGlUtil.h"


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
			
			NewtonBodyGetMassMatrix(visitor, &mass, &Ixx, &Iyy, &Izz);
			if (mass > 0.0f) {
				dMatrix matrix;
				dVector cog;
				dVector accelPerUnitMass;
				dVector torquePerUnitMass;
				const dVector gravity (0.0f, DEMO_GRAVITY, 0.0f, 0.0f);

				NewtonBodyGetMatrix (visitor, &matrix[0][0]);
				NewtonBodyGetCentreOfMass(visitor, &cog[0]);
				cog = matrix.TransformVector (cog);
				NewtonCollision* const collision = NewtonBodyGetCollision(visitor);

				
				dFloat shapeVolume = NewtonConvexCollisionCalculateVolume (collision);
				dFloat fluidDentity = 1.0f / (m_waterToSolidVolumeRatio * shapeVolume);

				NewtonConvexCollisionCalculateBuoyancyAcceleration (collision, &matrix[0][0], &cog[0], &gravity[0], &m_plane[0], fluidDentity, 0.1f, &accelPerUnitMass[0], &torquePerUnitMass[0]);

				dVector force (accelPerUnitMass.Scale (mass));
				dVector torque (torquePerUnitMass.Scale (mass));

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
	CreateLevelMesh (scene, "swimmingPool.ngd", false);


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

	int count = 5;
	dVector size (1.0f, 0.25f, 0.5f);
	dVector location (10.0f, 0.0f, 0.0f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());

	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}