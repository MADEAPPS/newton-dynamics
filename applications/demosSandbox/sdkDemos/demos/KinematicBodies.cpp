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
		:DemoEntity (dGetIdentityMatrix(), NULL)
	{
		NewtonWorld* const world = scene->GetNewton();

		dMatrix matrix (dGetIdentityMatrix());
		NewtonCollision* const shape = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, NULL);
		m_phantom = NewtonCreateKinematicBody(world, shape, &matrix[0][0]);

		m_solideMesh = new DemoMesh("primitive", shape, "smilli.tga", "smilli.tga", "smilli.tga");
		DemoMesh* const geometry = new DemoMesh("primitive", shape, "smilli.tga", "smilli.tga", "smilli.tga", 0.5f);
		DemoSubMesh& subMesh = geometry->GetFirst()->GetInfo();
		subMesh.m_specular.m_z = 0.0f;
		subMesh.m_diffuse.m_z = 0.0f;
		subMesh.m_specular.m_z = 0.0f;
		geometry->OptimizeForRender();

		SetMesh(geometry, dGetIdentityMatrix());
		geometry->Release();

		
		NewtonBodySetUserData (m_phantom, this);
		NewtonBodySetMassProperties (m_phantom, 10.0f, shape);
		NewtonBodySetTransformCallback (m_phantom, DemoEntity::TransformCallback);

		NewtonDestroyCollision(shape);
	}

	~PhantomPlacement()
	{
		m_solideMesh->Release();
	}
	
	NewtonBody* m_phantom;
	DemoMesh* m_solideMesh;
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

class dKinematicPlacementManager: public CustomControllerManager<dKinematicPlacement>, public dComplemtaritySolver 
{
	public:
	class dContactJoint: public dBilateralJoint
	{
		public: 
		dContactJoint()
			:dBilateralJoint()
		{
		}

		void UpdateSolverForces (const dJacobianPair* const jacobians) const
		{
		}

		void SetContacts (int count, const dFloat* const points, const dFloat* const normals)
		{
			m_count = dMin (count, int (sizeof (m_points) / sizeof (m_points[0])));
			for (int i = 0; i < count; i ++) {
				m_points[i] = dVector (points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2], 1.0f);
				m_normals[i] = dVector (normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2], 1.0f);
			}
		}

		void JacobianDerivative (dParamInfo* const constraintParams)
		{
		
			for (int i = 0; i < m_count; i ++) {
				dPointDerivativeParam pointData;
				InitPointParam (pointData, m_points[i]);
				CalculatePointDerivative (constraintParams, m_normals[i], pointData);

				dVector velocError (pointData.m_veloc1 - pointData.m_veloc0);

				dFloat restitution = 0.0f;
				dFloat relVelocErr = velocError % m_normals[i];
				dFloat penetration = 0.0f;
				dFloat penetrationStiffness = 0.0f;
				dFloat penetrationVeloc = penetration * penetrationStiffness;

				if (relVelocErr > 1.0e-3f) {
					relVelocErr *= (restitution + dFloat (1.0f));
				}
								
				constraintParams->m_jointLowFriction[i] = dFloat (0.0f);
				constraintParams->m_jointAccel[i] = dMax (dFloat (-4.0f), relVelocErr + penetrationVeloc) * constraintParams->m_timestepInv;
			}
		}

		dVector m_points[8];
		dVector m_normals[8];
		int m_count;
	};


	dKinematicPlacementManager(DemoEntityManager* const scene)
		:CustomControllerManager<dKinematicPlacement>(scene->GetNewton(), "dKinematicPlacementManager")
		,m_castDir (0.0f, -1.0f, 0.0f, 0.0f)
		,m_phantomEntity(NULL)
		,m_helpKey (true)
		,m_selectShape (true)
		,m_placeInstance (false)
		,m_hitParam(0.0f)
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
		//return (me->m_phantomEntity->m_phantom != body) ? 1 : 0;
		return me->CanBeRayCasted (body) ? 1 : 0;
	}

    static int aabbCollisionCallback (const NewtonBody * const otherBody, void * const userData)
    {   
		dKinematicPlacementManager* const me = (dKinematicPlacementManager*) userData;

	    NewtonBody* const myBody = (NewtonBody*)me->m_phantomEntity->m_phantom;
	    dAssert (myBody);

	    // not interested in self collision
	    if ( myBody == otherBody ) {
		    return 1;
	    }

		if (!me->CanBeRayCasted(otherBody)) {
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

		int contactCount = 0;
		if( NewtonCollisionIntersectionTest(world, collisionA, &poseA[0][0], collisionB, &poseB[0][0],0) ) {
			contactCount = NewtonCollisionCollide(world, maxSize, collisionA, &poseA[0][0], collisionB, &poseB[0][0], &contacts[0][0], &normals[0][0], &penetrations[0], &attrbA[0], &attrbB[0], 0);
			for(int i = 0; i < contactCount; i ++) {
				me->m_isInPenetration |= (penetrations[i] > 1.0e-3f);
			}
		}

		if (!me->m_isInPenetration && contactCount) {
			bool wasAjusted = true;
			for (int i = 0; (i < 8) & wasAjusted; i ++) {
				dMatrix savedMatrix (poseA);
				wasAjusted = me->AdjustRotationMatrix (poseA, collisionA, contactCount, &contacts[0][0], &normals[0][0]);
				if (wasAjusted) {
					contactCount = NewtonCollisionCollide(world, maxSize, collisionA, &poseA[0][0], collisionB, &poseB[0][0], &contacts[0][0], &normals[0][0], &penetrations[0], &attrbA[0], &attrbB[0], 0);
					for(int i = 0; i < contactCount; i ++) {
						wasAjusted &= (penetrations[i] < 1.0e-3f);
					}
					if (!wasAjusted) {
						poseA = savedMatrix;
					}
				}
			}
		}
	    return me->m_isInPenetration ? 0 : 1;
    }

    bool testForCollision ()
    {
	    dVector minP;
	    dVector maxP;
		dMatrix matrix;
		//NewtonBodyGetAABB (m_phantomEntity->m_phantom, &minP.m_x, &maxP.m_x);

		NewtonBodyGetMatrix (m_phantomEntity->m_phantom, &matrix[0][0]);
		CalculateAABB (NewtonBodyGetCollision(m_phantomEntity->m_phantom), matrix, minP, maxP);

	    NewtonWorld* const world = NewtonBodyGetWorld(m_phantomEntity->m_phantom);

		m_isInPenetration = false;
	    NewtonWorldForEachBodyInAABBDo(world, &minP.m_x, &maxP.m_x, aabbCollisionCallback, this);

		return false;
    }


	bool SetPlacementMatrix (const dVector& posit) const
	{
		dMatrix matrix (dGetIdentityMatrix());

		matrix.m_posit = posit - m_castDir.Scale (3.0f);
		//matrix.m_posit.m_y += 3.0f;
		matrix.m_posit.m_w = 1.0f;

		dVector dest (matrix.m_posit + m_castDir.Scale (6.0f));
		//dest.m_y -= 6.0f;

		dFloat hitParam;
		NewtonWorldConvexCastReturnInfo info[16];
		NewtonCollision* const shape = NewtonBodyGetCollision(m_phantomEntity->m_phantom);

		NewtonWorld* const world = GetWorld();
		int count = NewtonWorldConvexCast (world, &matrix[0][0], &dest[0], shape, &hitParam, (void*)this, RayPrefilterCallback, &info[0], 4, 0);		
		if (count) {
			matrix.m_posit += (dest - matrix.m_posit).Scale (hitParam);
			NewtonBodySetMatrix(m_phantomEntity->m_phantom, &matrix[0][0]);
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

			float x = dFloat (mouseX);
			float y = dFloat (mouseY);
			dVector p0 (camera->ScreenToWorld(dVector (x, y, 0.0f, 0.0f)));
			dVector p1 (camera->ScreenToWorld(dVector (x, y, 1.0f, 0.0f)));

            m_hitParam = 1.2f;
            NewtonWorldRayCast(world, &p0[0], &p1[0], RayCastFilter, this, RayPrefilterCallback, 0);
            if (m_hitParam < 1.0f) {
				if (SetPlacementMatrix (p0 + (p1 - p0).Scale (m_hitParam))) {
					if (!testForCollision ()) {
						if (m_placeInstance.UpdateTriggerJoystick (mainWindow, mainWindow->GetMouseKeyState(0))) {
							//dTrace (("xxx\n"));
							dMatrix matrix;
							NewtonBodyGetMatrix(m_phantomEntity->m_phantom, &matrix[0][0]);
							NewtonCollision* const collision = NewtonBodyGetCollision(m_phantomEntity->m_phantom);
							NewtonBody* const body = CreateSimpleSolid (scene, m_phantomEntity->m_solideMesh, 10.0f, matrix, collision, NewtonMaterialGetDefaultGroupID(world));
							m_selectionToIgnore.Append(body);
						} 
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

	bool AdjustRotationMatrix (dMatrix& matrix, const NewtonCollision* const collision, int contactCount, const float* const contacts, const float* const normals)
	{
		dMatrix localMatrix;
		NewtonCollisionGetMatrix (collision, &localMatrix[0][0]);

		m_body.SetMatrix(matrix);
		m_body.SetLocalMatrix(localMatrix);

		dVector com;
		dVector inertia;
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &com[0]);	

		dFloat mass = 1.0f;
		m_body.SetMass(mass);
		m_body.SetInertia (mass * inertia[0], mass * inertia[1], mass * inertia[2]);
		m_body.UpdateInertia();

		m_body.SetForce (m_castDir.Scale (30.0f * mass));
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
		m_contactJoint.SetContacts (contactCount, contacts, normals);
		BuildJacobianMatrix (1, jointArray, timeStep, jacobianPairArray, jacobianColumn, sizeof (jacobianPairArray)/ sizeof (jacobianPairArray[0]));
		CalculateReactionsForces (2, bodyArray, 1, jointArray, timeStep, jacobianPairArray, jacobianColumn);

		bool ret = false;
		dFloat vMag2 = m_body.GetVelocity() % m_body.GetVelocity();
		dFloat wMag2 = m_body.GetOmega() % m_body.GetOmega();
		if ((vMag2 > 1.0e-6f) || (wMag2 > 1.0e-6f)) {
			ret = true;
			m_body.IntegrateVelocity (timeStep);
		}
		return ret;
	}

	dVector m_castDir;
	PhantomPlacement* m_phantomEntity;
	dList<NewtonBody*> m_selectionToIgnore;
	DemoEntityManager::ButtonKey m_helpKey;
	DemoEntityManager::ButtonKey m_selectShape;
	DemoEntityManager::ButtonKey m_placeInstance;
    dFloat m_hitParam;
	bool m_isInPenetration;

	dBodyState m_body;
	dBodyState m_static;
	dContactJoint m_contactJoint;
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
	dVector origin (-10.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);



//	ExportScene (scene->GetNewton(), "../../../media/test1.ngd");

}

