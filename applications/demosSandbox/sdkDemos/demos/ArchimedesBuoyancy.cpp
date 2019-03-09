#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"

class BuoyancyTriggerManager: public dCustomTriggerManager
{
	public:
	class TriggerCallback
	{
		public:
		TriggerCallback(dCustomTriggerController* const controller)
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

		dCustomTriggerController* m_controller;
	};

	class BuoyancyForce: public TriggerCallback
	{
		public:
		BuoyancyForce(dCustomTriggerController* const controller)
			:TriggerCallback (controller)
			,m_plane(0.0f)
		{
			// get the fluid plane for the upper face of the trigger volume
			NewtonBody* const body = m_controller->GetBody();
			NewtonWorld* const world = NewtonBodyGetWorld(body);

			dMatrix matrix;
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, 20.0f, matrix.m_posit.m_z, 0.0f), 40.0f));
			m_plane = dVector (0.0f, 1.0f, 0.0f, -floor.m_y);
		}

		void OnEnter(NewtonBody* const visitor)
		{
			// make some random density, and store on the collision shape for more interesting effect. 
			dFloat density = 1.1f + dGaussianRandom (0.4f);
			NewtonCollision* const collision = NewtonBodyGetCollision(visitor);

			NewtonCollisionMaterial collisionMaterial;
			NewtonCollisionGetMaterial (collision, &collisionMaterial);
			collisionMaterial.m_userParam[0]= density;
			NewtonCollisionSetMaterial (collision, &collisionMaterial);
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
				dVector cenyterOfPreasure(0.0f);

				NewtonBodyGetMatrix (visitor, &matrix[0][0]);
				NewtonCollision* const collision = NewtonBodyGetCollision(visitor);
				
				// calculate the volume and center of mass of the shape under the water surface 
				dFloat volume = NewtonConvexCollisionCalculateBuoyancyVolume (collision, &matrix[0][0], &m_plane[0], &cenyterOfPreasure[0]);
				if (volume > 0.0f) {
					// if some part of the shape si under water, calculate the buoyancy force base on 
					// Archimedes's buoyancy principle, which is the buoyancy force is equal to the 
					// weight of the fluid displaced by the volume under water. 
					dVector cog(0.0f);
					const dFloat viscousDrag = 0.997f;
					//const dFloat solidDentityFactor = 1.35f;

					// Get the body density form the collision material.
					NewtonCollisionMaterial collisionMaterial;
					NewtonCollisionGetMaterial(collision, &collisionMaterial);
					const dFloat solidDentityFactor = collisionMaterial.m_userParam[0];

					// calculate the ratio of volumes an use it calculate a density equivalent
					dFloat shapeVolume = NewtonConvexCollisionCalculateVolume (collision);
					dFloat density = mass * solidDentityFactor /shapeVolume;

					dFloat displacedMass = density * volume;
					NewtonBodyGetCentreOfMass(visitor, &cog[0]);
					cenyterOfPreasure -= matrix.TransformVector(cog);

					// now with the mass and center of mass of the volume under water, calculate buoyancy force and torque
					dVector force (dFloat(0.0f), dFloat(-DEMO_GRAVITY * displacedMass), dFloat(0.0f), dFloat(0.0f));
					dVector torque (cenyterOfPreasure.CrossProduct(force));

					NewtonBodyAddForce (visitor, &force[0]);
					NewtonBodyAddTorque (visitor, &torque[0]);

					// apply a fake viscous drag to damp the under water motion 
					dVector omega(0.0f);
					dVector veloc(0.0f);
					NewtonBodyGetOmega(visitor, &omega[0]);
					NewtonBodyGetVelocity(visitor, &veloc[0]);
					omega = omega.Scale(viscousDrag);
					veloc = veloc.Scale(viscousDrag);
					NewtonBodySetOmega(visitor, &omega[0]);
					NewtonBodySetVelocity(visitor, &veloc[0]);
				}
			}
		}

		dVector m_plane;
	};

	BuoyancyTriggerManager(NewtonWorld* const world)
		:dCustomTriggerManager(world)
	{
	}

	~BuoyancyTriggerManager()
	{
	}

	void CreateBuoyancyTrigger (const dMatrix& matrix, NewtonCollision* const convexShape)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// create a large summing pool, using a trigger volume 
		dCustomTriggerController* const trigger = CreateTrigger (matrix, convexShape, NULL);
		NewtonBody* const triggerBody = trigger->GetBody();

		NewtonBodySetTransformCallback(triggerBody, DemoEntity::TransformCallback);
		DemoMesh* const geometry = new DemoMesh("pool", scene->GetShaderCache(), convexShape, "", "", "");
		DemoEntity* const entity = new DemoEntity(matrix, NULL);
		scene->Append(entity);
		entity->SetMesh(geometry, dGetIdentityMatrix());
		NewtonBodySetUserData(triggerBody, entity);

		for (DemoMesh::dListNode* ptr = geometry->GetFirst(); ptr; ptr = ptr->GetNext()) {
			DemoSubMesh* const subMesh = &ptr->GetInfo();
			subMesh->SetOpacity(0.25f);
			subMesh->m_diffuse.m_x = 0.0f;
			subMesh->m_diffuse.m_y = 0.0f;
			subMesh->m_diffuse.m_z = 1.0f;
			subMesh->m_diffuse.m_z = 1.0f;
		}
		geometry->OptimizeForRender();

		geometry->Release();

		BuoyancyForce* const buoyancyForce = new BuoyancyForce (trigger);
		trigger->SetUserData (buoyancyForce);
	}
	
	void DestroyTrigger (dCustomTriggerController* const trigger)
	{
		TriggerCallback* const userData = (TriggerCallback*) trigger->GetUserData();
		delete userData;
		dCustomTriggerManager::DestroyTrigger (trigger);
	}
	
	virtual void EventCallback (const dCustomTriggerController* const me, dTriggerEventType event, NewtonBody* const visitor) const
	{
		// each trigger has it own callback for some effect 
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
	BuoyancyTriggerManager* const triggerManager = new BuoyancyTriggerManager(scene->GetNewton());

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

	int count = 6;
	dVector size (1.0f, 0.5f, 0.5f);
	dVector location (10.0f, 0.0f, 0.0f, 0.0f);
	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());

	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
}