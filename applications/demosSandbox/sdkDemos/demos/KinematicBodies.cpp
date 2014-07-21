/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "DemoMesh.h"
#include "NewtonDemos.h"
#include "PhysicsUtils.h"


class PhantomPlacement: public DemoEntity
{
	public:
	PhantomPlacement (DemoEntityManager* const scene)
		:DemoEntity (GetIdentityMatrix(), NULL)
	{
		NewtonWorld* const world = scene->GetNewton();

		dMatrix matrix (GetIdentityMatrix());
		NewtonCollision* const shape = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, NULL);
		m_phantom = NewtonCreateKinematicBody(world, shape, &matrix[0][0]);

		DemoMesh* const geometry = new DemoMesh("primitive", shape, NULL, NULL, NULL);
		SetMesh(geometry, GetIdentityMatrix());
		geometry->Release();

		
		NewtonBodySetUserData (m_phantom, this);
		NewtonBodySetMassProperties (m_phantom, 10.0f, shape);
		NewtonBodySetTransformCallback (m_phantom, DemoEntity::TransformCallback);

		NewtonDestroyCollision(shape);
	}

	virtual void Render(dFloat timeStep) const
	{
		glEnable (GL_BLEND);
		glDisable(GL_TEXTURE_2D);
		glDisable (GL_LIGHTING);
		//glDisable(GL_CULL_FACE);

		glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//glBlendFunc (GL_ONE, GL_ONE);
		glColor4f(1.0f, 0.0f, 1.0f, 0.25f);

		DemoEntity::Render(timeStep);
		
		//glEnable(GL_CULL_FACE);
		glEnable (GL_LIGHTING);
		glEnable(GL_TEXTURE_2D);
		glDisable (GL_BLEND);
	}

	NewtonBody* m_phantom;
};

class dKinematicPlacement: public CustomControllerBase
{
	public:
	void PreUpdate(dFloat timestep, int threadIndex)
	{
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
	}
};

class dKinematicPlacementManager: public CustomControllerManager<dKinematicPlacement> 
{
	public:
	dKinematicPlacementManager(DemoEntityManager* const scene)
		:CustomControllerManager<dKinematicPlacement>(scene->GetNewton(), "dKinematicPlacementManager")
		,m_phantomEntity(NULL)
		,m_helpKey (true)
		,m_selectShape (true)
	{
		scene->Set2DDisplayRenderFunction (RenderHelp, this);

		m_phantomEntity = new PhantomPlacement (scene);
		scene->Append (m_phantomEntity);
	}

	static void RenderHelp (DemoEntityManager* const scene, void* const context, int lineNumber)
	{
		dKinematicPlacementManager* const me = (dKinematicPlacementManager*) context;
		me->RenderHelp (scene, lineNumber);
	}

	void RenderHelp (DemoEntityManager* const scene, int lineNumber)
	{
		if (m_helpKey.GetPushButtonState()) {
			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			lineNumber = scene->Print (color, 10, lineNumber + 20, "Right click and drag the location where you want to place a dynamic body");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "space      : change casting shape");
			lineNumber = scene->Print (color, 10, lineNumber + 20, "h          : hide help");
		}
	}

	virtual void Debug () const 
	{
	};

    static dFloat RayCastFilter (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
    {
        dKinematicPlacementManager* const me = (dKinematicPlacementManager*) userData;
        if (me->m_phantomEntity->m_phantom != body) {
            if (intersetParam < me->m_hitPoint) {
                me->m_hitPoint = intersetParam;
            }
            return intersetParam;
        }
        return 1.0f;
    }

    virtual void PostUpdate (dFloat timestep)
    {
    }

/*
    static int aabbCollisionCallback (const NewtonBody * const otherBody, void * const userData)
    {   
	    NewtonBody* const myBody = (NewtonBody*)userData;
	    dAssert (myBody);

	    // not interested in self collision
	    if ( myBody == otherBody ) {
		    return 1;
	    }

	    NewtonWorld* const world = NewtonBodyGetWorld(myBody);
	    NewtonCollision* const collisionA = NewtonBodyGetCollision(myBody);
	    NewtonCollision* const collisionB = NewtonBodyGetCollision(otherBody);
	
	    dMatrix poseA;
	    NewtonBodyGetMatrix(myBody, &poseA[0][0]);

	    dMatrix poseB;
	    NewtonBodyGetMatrix(otherBody,&poseB[0][0]);

	    if( NewtonCollisionIntersectionTest(world, collisionA, &poseA[0][0], collisionB, &poseB[0][0],0) )
	    {
		    // ignore contact with no penetration
		    const int maxSize = 2;
		    dFloat contacts[maxSize * 3];
		    dFloat normals[maxSize * 3];
		    dFloat penetrations[maxSize];
		    dLong attrbA[maxSize * 3];
		    dLong attrbB[maxSize * 3];
		    int contactCount = NewtonCollisionCollide(world, maxSize,
			    collisionA, &poseA[0][0], 
			    collisionB, &poseB[0][0],
			    &contacts[0],
			    &normals[0],
			    &penetrations[0],
			    &attrbA[0],
			    &attrbB[0],
			    0);
		    if(contactCount) {
    //			dAssert (0);
			    contactCount*=1;
			    //entity->bodyDesc.collisionInfo.collisionList.push_back( (PhysicsBody*)(otherEntity) );

		    }
	    }
	    return 1;
    }


    static bool testForCollision (NewtonBody * const pBody)
    {
	    // this body has NewtonCollisionSetCollisonMode set to 0
	    dVector min;
	    dVector max;
	    NewtonBodyGetAABB (pBody, &min.m_x, &max.m_x);
	    NewtonWorld* const world = NewtonBodyGetWorld(pBody);
	    NewtonWorldForEachBodyInAABBDo(world, &min.m_x, &max.m_x, aabbCollisionCallback, pBody);
	    return true;
    }
*/

	virtual void PreUpdate (dFloat timestep)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);
		NewtonDemos* const mainWindow = scene->GetRootWindow();
		DemoCamera* const camera = scene->GetCamera();

		m_helpKey.UpdatePushButton (mainWindow, 'H');
		if (m_selectShape.UpdateTriggerButton (mainWindow, ' ')) {
//			m_stupidLevel->ChangeCastingShape();
		}

		bool buttonState = mainWindow->GetMouseKeyState(1);
		if (buttonState) {
			int mouseX;
			int mouseY;

			//buttonState = false;
			mainWindow->GetMousePosition (mouseX, mouseY);

			float x = dFloat (mouseX);
			float y = dFloat (mouseY);
			dVector p0 (camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));

//            dMousePickClass rayCast;
            m_hitPoint = 1.2f;
            NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastFilter, this, NULL, 0);
            if (m_hitPoint < 1.0f) {
                dMatrix matrix (GetIdentityMatrix());
                matrix.m_posit = p0 + (p1 - p0).Scale (m_hitPoint);
                NewtonBodySetMatrix(m_phantomEntity->m_phantom, &matrix[0][0]);
            }


/*
			// do the convex cast here 
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = p0;

			dFloat hitParam;
			NewtonWorldConvexCastReturnInfo info[16];
			NewtonCollision* const shape = m_stupidLevel->GetCurrentShape();
			int count = NewtonWorldConvexCast (world, &matrix[0][0], &p1[0], shape, &hitParam, NULL, NULL, &info[0], 4, 0);		
			if (count) {
				matrix.m_posit += (p1 - matrix.m_posit).Scale (hitParam);
				m_stupidLevel->SetCastEntityMatrix (scene, matrix);
			}
			m_stupidLevel->SetCastingLine (p0, p1);
*/
		}
	}

	PhantomPlacement* m_phantomEntity;
	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_selectShape;
    dFloat m_hitPoint;
};



void KinematicPlacement (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", 1);


	// create a system for object placement
	new dKinematicPlacementManager (scene);

	// add some bodies 
//	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (0,0,0,0);
	location.m_x += 0.0f;
	location.m_z += 0.0f;
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);

//	int count = 1;
//	dMatrix shapeOffsetMatrix (GetIdentityMatrix());
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _TAPERED_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);


	// place camera into position
	dQuaternion rot;
	dVector origin (-100.0f, 40.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);



//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");

}

