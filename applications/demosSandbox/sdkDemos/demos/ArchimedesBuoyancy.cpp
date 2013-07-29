#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "CustomTriggerManager.h"
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

		virtual void OnEnter()
		{
		}

		virtual void OnInside()
		{
		}

		virtual void OnExit()
		{
		}

		CustomTriggerController* m_controller;
	};

	class BuoyancyForce: public TriggerCallback
	{
		public:
		BuoyancyForce(CustomTriggerController* const controller)
			:TriggerCallback (controller)
		{
		}

		void OnInside()
		{
		}
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
				callback->OnEnter();
				break;
			}

			case m_exitTrigger:
			{
				callback->OnExit();
				break;
			}

			case m_inTrigger:
			{
				callback->OnInside();
				break;
			}
		}
	}
};



void AlchemedesBuoyancy(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();


	// load the mesh 
	CreateLevelMesh (scene, "swimmingPool.ngd", false);


	// add a triget Managet to teh workld
	MyTriggerManager* const triggerManager = new MyTriggerManager(scene->GetNewton());

	dMatrix triggerLocation (GetIdentityMatrix());
	triggerLocation.m_posit.m_x =  17.0f;
	triggerLocation.m_posit.m_y = -3.5f;

	NewtonCollision* const poolBox = NewtonCreateBox (scene->GetNewton(), 30.0f, 6.0f, 20.0f, 0, NULL);  
	triggerManager->CreateBuoyancyTrigger (triggerLocation, poolBox);
	NewtonDestroyCollision (poolBox);


	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
//	NewtonWorld* const world = scene->GetNewton();
	dMatrix offsetMatrix (GetIdentityMatrix());

	// place camera into position
	dMatrix camMatrix (GetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-20.0f, 10.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);


	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());

	int count = 1;
	dVector size (1.0f, 0.25f, 0.5f);
	dVector location (.0f, 0.0f, 0.0f, 0.0f);
	dMatrix shapeOffsetMatrix (GetIdentityMatrix());

//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}