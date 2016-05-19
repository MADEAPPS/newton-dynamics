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

#define D_PLACEMENT_PENETRATION   0.005f

class PhantomPlacement: public DemoEntity
{
	public:
	PhantomPlacement (DemoEntityManager* const scene)
		:DemoEntity (dGetIdentityMatrix(), NULL)
	{
		NewtonWorld* const world = scene->GetNewton();

		dMatrix matrix (dGetIdentityMatrix());
		NewtonCollision* const shape = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, NULL);
		m_phantom = NewtonCreateKinematicBody(world, shape, &matrix[0][0]);

		m_solideMesh = new DemoMesh("primitive", shape, "smilli.tga", "smilli.tga", "smilli.tga");
		m_redMesh = CreatePhantomMesh (shape, dVector (1.0f, 0.0f, 0.0f, 0.5f)); 
        m_blueMesh = CreatePhantomMesh (shape, dVector (0.0f, 0.5f, 0.0f, 0.5f)); 
		SetMesh (m_redMesh, dGetIdentityMatrix());
		
		NewtonBodySetUserData (m_phantom, this);
		NewtonBodySetMassProperties (m_phantom, 10.0f, shape);
		NewtonBodySetTransformCallback (m_phantom, DemoEntity::TransformCallback);

		NewtonDestroyCollision(shape);
	}

	~PhantomPlacement()
	{
        m_redMesh->Release();
        m_blueMesh->Release();
		m_solideMesh->Release();
	}
	
    DemoMesh* CreatePhantomMesh (NewtonCollision* const shape, const dVector& color) 
    {
        DemoMesh* const mesh = new DemoMesh("primitive", shape, "smilli.tga", "smilli.tga", "smilli.tga", 0.5f);
        DemoSubMesh& subMesh = mesh->GetFirst()->GetInfo();
        subMesh.m_specular = color;
        subMesh.m_diffuse = color;
        subMesh.m_ambient = color;
        mesh->OptimizeForRender();
        return mesh;
    }

    void SetPhantomMesh (bool redOrBlue)
    {
        redOrBlue ? SetMesh(m_redMesh, dGetIdentityMatrix()) : SetMesh(m_blueMesh, dGetIdentityMatrix());
    }

	NewtonBody* m_phantom;
	DemoMesh* m_solideMesh;
    DemoMesh* m_redMesh;
    DemoMesh* m_blueMesh;
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



class dKinematicPlacementManager: public CustomControllerManager<dKinematicPlacement>, public dComplentaritySolver 
{
	public:

	dKinematicPlacementManager(DemoEntityManager* const scene)
		:CustomControllerManager<dKinematicPlacement>(scene->GetNewton(), "dKinematicPlacementManager")
		,m_castDir (0.0f, -1.0f, 0.0f, 0.0f)
		,m_phantomEntity(NULL)
		,m_helpKey (true)
		,m_selectShape (true)
		,m_placeInstance (false)
		,m_hitParam(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
        ,m_isInPenetration(false)
	{
		scene->Set2DDisplayRenderFunction (RenderHelp, this);
		m_phantomEntity = new PhantomPlacement (scene);
		scene->Append (m_phantomEntity);
		m_contactJoint.Init (&m_body, &m_static);
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
			lineNumber = scene->Print (color, 10, lineNumber + 20, "Left click while Right click down place a dynamics body if the location is free of intersection");
			//lineNumber = scene->Print (color, 10, lineNumber + 20, "space      : change casting shape");
			//lineNumber = scene->Print (color, 10, lineNumber + 20, "h          : hide help");
		}
	}

	virtual void Debug () const 
	{
	}


	bool CanBeRayCasted(const NewtonBody* const body) const 
	{
		if (m_phantomEntity->m_phantom != body) {
			for (dList<NewtonBody*>::dListNode* node = m_selectionToIgnore.GetFirst(); node; node = node->GetNext()) {
				if (node->GetInfo() == body) {
					return false;
				}
			}
			return true;
		}
		return false;
	}

    static dFloat RayCastFilter (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
    {
		dKinematicPlacementManager* const me = (dKinematicPlacementManager*) userData;
		if (intersetParam < me->m_hitParam) {
			me->m_hitParam = intersetParam;
		}
		return intersetParam;
    }

	static unsigned RayPrefilterCallback (const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
	{
		dKinematicPlacementManager* const me = (dKinematicPlacementManager*) userData;
		return me->CanBeRayCasted (body) ? 1 : 0;
	}

    static int RotationCollisionCallback (const NewtonBody * const otherBody, void * const userData)
    {   
        dKinematicPlacementManager* const me = (dKinematicPlacementManager*) userData;

        NewtonBody* const myBody = (NewtonBody*)me->m_phantomEntity->m_phantom;
        dAssert (myBody);

        // not interested in self collision
        if (myBody == otherBody) {
            return 1;
        }

        if (!me->CanBeRayCasted (otherBody)) {
            return 1;
        }

        NewtonWorld* const world = NewtonBodyGetWorld(myBody);
        NewtonCollision* const collisionA = NewtonBodyGetCollision(myBody);
        NewtonCollision* const collisionB = NewtonBodyGetCollision(otherBody);

        dMatrix poseA;
        NewtonBodyGetMatrix(myBody, &poseA[0][0]);

        dMatrix poseB;
        NewtonBodyGetMatrix(otherBody,&poseB[0][0]);

        const int maxSize = 4;
        dFloat points[maxSize][3];
        dFloat normals[maxSize][3];
        dFloat penetrations[maxSize];
        dLong attrbA[maxSize];
        dLong attrbB[maxSize];


        if( NewtonCollisionIntersectionTest(world, collisionA, &poseA[0][0], collisionB, &poseB[0][0],0) ) {
            int contactCount = NewtonCollisionCollide(world, maxSize, collisionA, &poseA[0][0], collisionB, &poseB[0][0], &points[0][0], &normals[0][0], &penetrations[0], &attrbA[0], &attrbB[0], 0);
            if (contactCount && ((me->m_contactCount + contactCount) <= int (sizeof (me->m_contacts) / sizeof (me->m_contacts[0])))) {
                for(int i = 0; i < contactCount; i ++) {
                    me->m_isInPenetration |= (penetrations[i] > 1.0e-3f);
                    me->m_contacts[me->m_contactCount].m_point = dVector (points[i][0], points[i][1], points[i][2], 1.0f);
                    me->m_contacts[me->m_contactCount].m_normal = dVector (normals[i][0], normals[i][1], normals[i][2], 0.0f);
                    me->m_contactCount ++;
                }
            }
        }
        return me->m_isInPenetration ? 0 : 1;
    }


    static int TranslationCollisionCallback (const NewtonBody * const otherBody, void * const userData)
    {   
		dKinematicPlacementManager* const me = (dKinematicPlacementManager*) userData;

	    NewtonBody* const myBody = (NewtonBody*)me->m_phantomEntity->m_phantom;
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

		const int maxSize = 4;
		dFloat contacts[maxSize][3];
		dFloat normals[maxSize][3];
		dFloat penetrations[maxSize];
		dLong attrbA[maxSize];
		dLong attrbB[maxSize];

		if( NewtonCollisionIntersectionTest(world, collisionA, &poseA[0][0], collisionB, &poseB[0][0],0) ) {
			int contactCount = NewtonCollisionCollide(world, maxSize, collisionA, &poseA[0][0], collisionB, &poseB[0][0], &contacts[0][0], &normals[0][0], &penetrations[0], &attrbA[0], &attrbB[0], 0);
			if (contactCount) {
				for(int i = 0; i < contactCount; i ++) {
					me->m_isInPenetration |= (penetrations[i] > (D_PLACEMENT_PENETRATION * 2.0f));
				}
			}
		}
	    return me->m_isInPenetration ? 0 : 1;
    }

    bool TestForCollision ()
    {
		dMatrix matrix;
	    dVector minP(0.0f);
	    dVector maxP(0.0f);
		
		//NewtonBodyGetAABB (m_phantomEntity->m_phantom, &minP.m_x, &maxP.m_x);

		NewtonBodyGetMatrix (m_phantomEntity->m_phantom, &matrix[0][0]);
		CalculateAABB (NewtonBodyGetCollision(m_phantomEntity->m_phantom), matrix, minP, maxP);

	    NewtonWorld* const world = NewtonBodyGetWorld(m_phantomEntity->m_phantom);

		m_isInPenetration = false;
	    NewtonWorldForEachBodyInAABBDo(world, &minP.m_x, &maxP.m_x, TranslationCollisionCallback, this);
		return m_isInPenetration;
    }


	bool CalculateTranslationMatrix (dMatrix& matrix) const
	{
		matrix.m_posit -= m_castDir.Scale (3.0f);
		matrix.m_posit.m_w = 1.0f;
		dVector dest (matrix.m_posit + m_castDir.Scale (6.0f));

		dFloat hitParam;
		NewtonWorldConvexCastReturnInfo info[16];
		NewtonCollision* const shape = NewtonBodyGetCollision(m_phantomEntity->m_phantom);

		NewtonWorld* const world = GetWorld();
		int count = NewtonWorldConvexCast (world, &matrix[0][0], &dest[0], shape, &hitParam, (void*)this, RayPrefilterCallback, &info[0], 4, 0);		
		if (count) {
			matrix.m_posit += (dest - matrix.m_posit).Scale (hitParam);
            matrix.m_posit += m_castDir.Scale (D_PLACEMENT_PENETRATION);
			return true;
		}

		return false;
	}

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

			dFloat x = dFloat (mouseX);
			dFloat y = dFloat (mouseY);
			dVector p0 (camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));

            m_hitParam = 1.2f;
            NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastFilter, this, RayPrefilterCallback, 0);
            if (m_hitParam < 1.0f) {
				dMatrix matrix1 (dPitchMatrix(m_pitch) * dYawMatrix(m_yaw) * dRollMatrix(m_roll));
				matrix1.m_posit = p0 + (p1 - p0).Scale (m_hitParam);
				matrix1.m_posit.m_w = 1.0f;
				if (CalculateTranslationMatrix (matrix1)) {
					NewtonBodySetMatrix(m_phantomEntity->m_phantom, &matrix1[0][0]);
					if (!TestForCollision ()) {
                        CalculateRotationMatrix (1000.0f);
                        m_phantomEntity->SetPhantomMesh (false);
//dTrace (("%d %d\n", mouseX, mouseY));

						if (m_placeInstance.UpdateTriggerJoystick (mainWindow, mainWindow->GetMouseKeyState(0))) {
							dMatrix matrix;
							NewtonBodyGetMatrix(m_phantomEntity->m_phantom, &matrix[0][0]);
							NewtonCollision* const collision = NewtonBodyGetCollision(m_phantomEntity->m_phantom);
							NewtonBody* const body = CreateSimpleSolid (scene, m_phantomEntity->m_solideMesh, 10.0f, matrix, collision, NewtonMaterialGetDefaultGroupID(world));
							m_selectionToIgnore.Append(body);
						} 
					} else {
                        m_phantomEntity->SetPhantomMesh (true);
                    }
				}
            }
		} else {
			m_selectionToIgnore.RemoveAll();
		}
	}

	virtual void PostUpdate (dFloat timestep)
	{
	}



	void CalculateRotationMatrix (dFloat power)
	{
		dMatrix matrix;
		NewtonWorld* const world = NewtonBodyGetWorld (m_phantomEntity->m_phantom);
		NewtonCollision* const collision = NewtonBodyGetCollision (m_phantomEntity->m_phantom);
		NewtonBodyGetMatrix (m_phantomEntity->m_phantom, &matrix[0][0]);

//dMatrix xxxxx (matrix);
		bool isUnstable = true;
//int xxx = 0;
		for (int i = 0; (i < 16) && isUnstable; i ++) {
            dVector minP(0.0f);
            dVector maxP(0.0f);
//xxx ++;
			isUnstable = false;
			if (CalculateTranslationMatrix (matrix)) {
                NewtonBodySetMatrix(m_phantomEntity->m_phantom, &matrix[0][0]);
                CalculateAABB (collision, matrix, minP, maxP);
                m_isInPenetration = false;
                m_contactCount = 0;
                NewtonWorldForEachBodyInAABBDo(world, &minP.m_x, &maxP.m_x, RotationCollisionCallback, this);
//dAssert (m_contactCount);
                if (m_contactCount) {
				    dMatrix localMatrix;
				    NewtonCollisionGetMatrix (collision, &localMatrix[0][0]);

				    m_body.SetMatrix(matrix);
				    m_body.SetLocalMatrix(localMatrix);

				    dVector com(0.0f);
				    dVector inertia(0.0f);
				    NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &com[0]);	

				    dFloat mass = 1.0f;
				    m_body.SetMass(mass);
				    m_body.SetInertia (mass * inertia[0], mass * inertia[1], mass * inertia[2]);
				    m_body.UpdateInertia();

				    m_body.SetForce (m_castDir.Scale (power * mass));
				    m_body.SetVeloc (dVector (0.0f, 0.0f, 0.0f, 0.0f));
				    m_body.SetOmega (dVector (0.0f, 0.0f, 0.0f, 0.0f));
				    m_body.SetTorque (dVector (0.0f, 0.0f, 0.0f, 0.0f));

				    dFloat timeStep = 0.005f;
				    dBodyState* bodyArray[2];
				    dBilateralJoint* jointArray[1];
				    dJacobianColum jacobianColumn[32];
				    dJacobianPair jacobianPairArray[32];

				    bodyArray[0] = &m_body;
				    bodyArray[1] = &m_static;
				    jointArray[0] = &m_contactJoint;
				    m_contactJoint.SetContacts (m_contactCount, m_contacts, 0.05f);
				    BuildJacobianMatrix (1, jointArray, timeStep, jacobianPairArray, jacobianColumn, sizeof (jacobianPairArray)/ sizeof (jacobianPairArray[0]));
				    CalculateReactionsForces (2, bodyArray, 1, jointArray, timeStep, jacobianPairArray, jacobianColumn);

				    dFloat vMag2 = m_body.GetVelocity() % m_body.GetVelocity();
				    dFloat wMag2 = m_body.GetOmega() % m_body.GetOmega();
				    if ((vMag2 > 1.0e-6f) || (wMag2 > 1.0e-6f)) {
					    isUnstable = true;
					    m_body.IntegrateVelocity (timeStep);
					    matrix = m_body.GetMatrix();
					    NewtonBodySetMatrix (m_phantomEntity->m_phantom, &matrix[0][0]);
				    }
			    }
            }
		}

//if (xxx == 1){
//NewtonBodySetMatrix (m_phantomEntity->m_phantom, &xxxxx[0][0]);
//}
	}

	dVector m_castDir;
	PhantomPlacement* m_phantomEntity;
	dList<NewtonBody*> m_selectionToIgnore;
	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_selectShape;
	DemoEntityManager::ButtonKey m_placeInstance;
    dFloat m_hitParam;
	dFloat m_pitch;
	dFloat m_yaw;
	dFloat m_roll;
    bool m_isInPenetration;

	dBodyState m_body;
	dBodyState m_static;
	dFrictionLessContactJoint m_contactJoint;
    int m_contactCount;
    dContact m_contacts[D_MAX_PLACEMENT_CONTACTS];
};


static NewtonBody* AddStaticMesh(DemoEntityManager* const scene)
{
	char fileName[2048];

	NewtonWorld* const world = scene->GetNewton();
	GetWorkingFileName ("ramp.off", fileName);
	NewtonMesh* const ntMesh = NewtonMeshLoadOFF(world, fileName);

	dMatrix matrix (dGetIdentityMatrix());
	DemoMesh* mesh = new DemoMesh(ntMesh);
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();

	scene->Append(entity);

	NewtonBody* const body = CreateLevelMeshBody (world, entity, true);
	NewtonMeshDestroy (ntMesh);

	return body;
}


void KinematicPlacement (DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();

	// load the scene from a ngd file format
	CreateLevelMesh (scene, "flatPlane.ngd", 1);

	AddStaticMesh (scene);

	// create a system for object placement
	new dKinematicPlacementManager (scene);

	// add some bodies 
//	int defaultMaterialID = NewtonMaterialGetDefaultGroupID (scene->GetNewton());
	dVector location (0,0,0,0);
	location.m_x += 0.0f;
	location.m_z += 0.0f;
	dVector size (0.5f, 0.5f, 0.75f, 0.0f);
/*
dMatrix matrix (dGetIdentityMatrix());
dFloat pts[] = { 0,0,0, 0,0,0.01, 0.01,0,0.01,  0.01,0,0, 0,0.03,0, 0,0.03,0.01, 0.01, 0.03,0, 0.01,0.03,0.01 };
NewtonCollision* col = NewtonCreateConvexHull(scene->GetNewton(), 8, pts, 12, 0.0, 0, NULL);
NewtonBody* body = NewtonCreateDynamicBody(scene->GetNewton(), col, &matrix[0][0]);
NewtonBodySetMassProperties(body, 1.0, col);
*/
//	int count = 1;
//	dMatrix shapeOffsetMatrix (GetIdentityMatrix());
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _SPHERE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CAPSULE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CONE_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _CHAMFER_CYLINDER_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _REGULAR_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _RANDOM_CONVEX_HULL_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
//	AddPrimitiveArray(scene, 10.0f, location, size, count, count, 5.0f, _COMPOUND_CONVEX_CRUZ_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);


	// place camera into position
	dQuaternion rot;
	dVector origin (-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);



//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");

}

