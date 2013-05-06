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
#include "RenderPrimitive.h"
#include "../OGLMesh.h"
#include "../MainFrame.h"
#include "../SceneManager.h"
#include "../PhysicsUtils.h"
#include "../toolBox/MousePick.h"
#include "../toolBox/OpenGlUtil.h"
#include "../toolBox/DebugDisplay.h"
#include "../toolBox/LevelPrimitive.h"
#include "../toolBox/PlaneCollision.h"
#include "../toolBox/HeightFieldPrimitive.h"
#include "../toolBox/UserHeightFieldCollision.h"


#include "Custom6DOF.h"
#include "CustomGear.h"
#include "CustomHinge.h"
#include "CustomPulley.h"
#include "CustomSlider.h"
#include "CustomWormGear.h"
#include "CustomUniversal.h"
#include "CustomCorkScrew.h"
#include "CustomBallAndSocket.h"
#include "CustomSlidingContact.h"
#include "JointLibrary.h"


static void SetDemoCallbacks (NewtonFrame& system)
{
	system.m_control = Keyboard;
	system.m_autoSleep = AutoSleep;
	system.m_showIslands = SetShowIslands;
	system.m_showContacts = SetShowContacts; 
	system.m_setMeshCollision = SetShowMeshCollision;
}

static LevelPrimitive* LoadLevelAndSceneRoot (NewtonFrame& system, const char* levelName, int optimized)
{
_ASSERTE (0);
return NULL;
/*

	NewtonWorld* world;
	NewtonBody* floorBody;
	LevelPrimitive* level;



	world = system.m_world;
	// /////////////////////////////////////////////////////////////////////
	//
	// create the sky box,
	OGLModel* sky = new SkyBox ();
	system.AddModel___ (sky);
	sky->Release();


	// Load a level geometry
	level = new LevelPrimitive (levelName, world, optimized);
	system.AddModel___(level);
	level->Release();
	floorBody = level->m_level;

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (floorBody, PhysicsBodyDestructor);

	// get the default material ID
	int defaultID;
	defaultID = NewtonMaterialGetDefaultGroupID (world);

	// set default material properties
	NewtonMaterialSetDefaultSoftness (world, defaultID, defaultID, 0.05f);
	NewtonMaterialSetDefaultElasticity (world, defaultID, defaultID, 0.4f);
	NewtonMaterialSetDefaultCollidable (world, defaultID, defaultID, 1);
	NewtonMaterialSetDefaultFriction (world, defaultID, defaultID, 1.0f, 0.5f);
	NewtonMaterialSetCollisionCallback (world, defaultID, defaultID, NULL, NULL, GenericContactProcess); 

//	NewtonMaterialSetSurfaceThickness(world, materialID, materialID, 0.1f);
//	NewtonMaterialSetSurfaceThickness(world, defaultID, defaultID, 0.0f);

	// set the island update callback
	NewtonSetIslandUpdateEvent (world, PhysicsIslandUpdate);

	// save th3 callback
	SetDemoCallbacks (system);

	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), dVector (-40.0f, 10.0f, 0.0f));

	return level;
*/
}







#define SPRING_CONST		40.0f
#define DAMPER_CONST		 4.0f
#define MAX_COMPRESION_DIST  0.1f
#define MIN_EXPANSION_DIST  -0.38f

#define REMOVE_REDUNDAT_CONTACT	

class FrictionTrank: public RenderPrimitive
{
	class FrictionTractionControl: public NewtonCustomJoint  
	{
		public:
		FrictionTractionControl (const NewtonBody* tankBody, const NewtonBody* leftTrack, const NewtonBody* rightTrack)
			:NewtonCustomJoint(1, tankBody, NULL)
		{
			m_veloc = 0.0f;
			m_turnVeloc = 0.0f;

			// hack the tank to move in circles
			m_veloc = 5.0f;
			m_turnVeloc = 1.0f;

			m_leftTrack = leftTrack;
			m_rightTrack = rightTrack;
		}

		void ApplyTracktionForce (dFloat timestep, const NewtonBody* track)
		{
			dVector veloc;
			dVector omega;
			dMatrix matrix;

			NewtonBodyGetOmega(m_body0, &omega[0]);
			NewtonBodyGetVelocity(m_body0, &veloc[0]);
			NewtonBodyGetMatrix (m_body0, &matrix[0][0]);
			

			// itetate over the contact list and condition each contact direction anc contact acclerations
			for (NewtonJoint* contactJoint = NewtonBodyGetFirstContactJoint (track); contactJoint; contactJoint = NewtonBodyGetNextContactJoint (track, contactJoint)) {
				_ASSERTE ((NewtonJointGetBody0 (contactJoint) == track) || (NewtonJointGetBody1 (contactJoint) == track));

				#ifdef REMOVE_REDUNDAT_CONTACT	
				int contactCount;
				contactCount = NewtonContactJointGetContactCount(contactJoint);
				if (contactCount > 2) {
					// project the contact to the bounday of the conve hull o fteh trhread foot ptint 
					dFloat maxDist;
					dFloat minDist;
					void* minContact;
					void* maxContact;
					
					dMatrix matrix;
			
					minContact = NULL;
				    maxContact = NULL;
					NewtonBodyGetMatrix (track, &matrix[0][0]);

					maxDist = -1.0e10f;
					minDist = -1.0e10f;
					//find the best two contacts and remove all others
					for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
						dFloat dist;
						dVector point;
						dVector normal;
						NewtonMaterial* material;

					    material = NewtonContactGetMaterial (contact);
						NewtonMaterialGetContactPositionAndNormal(material, &point[0], &normal[0]);
						
						dist = matrix.m_front % point;
						if (dist > maxDist) {
							maxDist = dist;
							maxContact = contact;
						} 
						if (-dist > minDist) {
							minDist = -dist;
							minContact = contact;
						}
						
					}

					// now delete all reduntact contacts
					void* nextContact;
					NewtonWorld* world;

					world = NewtonBodyGetWorld (track);
					NewtonWorldCriticalSectionLock(world);
					for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = nextContact) {
						nextContact = NewtonContactJointGetNextContact (contactJoint, contact);
						if (!((contact == minContact) || (contact == maxContact))) {
							NewtonContactJointRemoveContact (contactJoint, contact);
						}
					}
					NewtonWorldCriticalSectionUnlock(world);
				}

				#endif

			
				for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
					dFloat speed;
					dFloat accel;
					dVector point;
					dVector normal;
					dVector dir0;
					dVector dir1;
					NewtonMaterial* material;

				    material = NewtonContactGetMaterial (contact);
					NewtonMaterialContactRotateTangentDirections (material, &matrix.m_front[0]);
					NewtonMaterialGetContactPositionAndNormal(material, &point[0], &normal[0]);
					NewtonMaterialGetContactTangentDirections (material, &dir0[0], &dir1[0]);


					dVector posit (point - matrix.m_posit);
					veloc += omega * posit;
					speed = veloc % dir0;
				//	accel = m_accel - 0.1f * speed + (((posit % m_matrix.m_right) > 0.0f) ? m_turnAccel : - m_turnAccel);
					accel = m_veloc + (((posit % matrix.m_right) > 0.0f) ? m_turnVeloc : - m_turnVeloc);

					accel = (accel - speed) * 0.5f / timestep;

			//		NewtonMaterialSetContactStaticFrictionCoef (material, 1.0f, 0);
			//		NewtonMaterialSetContactKineticFrictionCoef (material, 1.0f, 0);
					NewtonMaterialSetContactFrictionCoef (material, 1.0f, 1.0f, 0);

			//		NewtonMaterialSetContactStaticFrictionCoef (material, 0.5f, 1);
			//		NewtonMaterialSetContactKineticFrictionCoef (material, 0.5f, 1);
					NewtonMaterialSetContactFrictionCoef (material, 0.5f, 0.5f, 1);
					
					NewtonMaterialSetContactTangentAcceleration (material, accel, 0);
				}

				// for debug purpose show the contact
				ShowJointContacts (contactJoint);
			}
		}

		void SubmitConstraints (dFloat timestep, int threadIndex)
		{
			// apply the friction due to contact
			ApplyTracktionForce (timestep, m_leftTrack);
			ApplyTracktionForce (timestep, m_rightTrack);
		}

		dFloat m_veloc;
		dFloat m_turnVeloc;
		const NewtonBody* m_leftTrack;
		const NewtonBody* m_rightTrack;
	};

	class FrictionTrankThreaSuspention: public CustomSlidingContact
	{
		public:
		FrictionTrankThreaSuspention(const dMatrix& pinsAndPivoFrame, const NewtonBody* child, const NewtonBody* parent)
			: CustomSlidingContact (pinsAndPivoFrame, child, parent)
		{
			dMatrix childMatrix;
			dMatrix parentMatrix;
			dVector maxPointFront;
			dVector minPointFront;

			NewtonCollision* collision;
			collision = NewtonBodyGetCollision(child);

			NewtonBodyGetMatrix(child, &childMatrix[0][0]);
			NewtonBodyGetMatrix(parent, &parentMatrix[0][0]);

			// find the the extreme front and rear point, this is used to calculate the position of the suspension points
			dVector frontDir (childMatrix.UnrotateVector(parentMatrix.m_front));
			NewtonCollisionSupportVertex(collision, &frontDir[0], &maxPointFront[0]);

			dVector rearDir (frontDir.Scale (-1.0f));
			NewtonCollisionSupportVertex(collision, &rearDir[0], &minPointFront[0]);


			// calculate the front suspension points
			dVector frontHardPoint (childMatrix.m_posit + childMatrix.RotateVector(frontDir.Scale (maxPointFront % frontDir)));
			m_frontHarpointOnParent = parentMatrix.UntransformVector(frontHardPoint);
			m_frontHarpointOnThread = childMatrix.UntransformVector(frontHardPoint);

			// calculate the front rear suspension points
			dVector rearHardPoint (childMatrix.m_posit + childMatrix.RotateVector(rearDir.Scale (minPointFront % rearDir)));
			m_rearHarpointOnParent = parentMatrix.UntransformVector(rearHardPoint);
			m_rearHarpointOnThread = childMatrix.UntransformVector(rearHardPoint);

		
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass0;
			dFloat mass1;
			NewtonBodyGetMassMatrix(child, &mass0, &Ixx, &Iyy, &Izz);
			NewtonBodyGetMassMatrix(parent, &mass1, &Ixx, &Iyy, &Izz);
			m_massScale = (mass0 * mass1) / (mass0 + mass1);
		}

		~FrictionTrankThreaSuspention()
		{	
			//do nothig at thsi time
		}

		void ApplySuspesionForce (
			dFloat timestep,
			const NewtonBody* thread, const dVector& threadPointLocal, const dMatrix& threadMatrix, const dVector& threadCOM, const dVector& threadVeloc, const dVector& threadOmega,
			const NewtonBody* parent, const dVector& parentPointLocal, const dMatrix& parentMatrix, const dVector& parentCOM, const dVector& parentVeloc, const dVector& parentOmega)
		{
			dFloat dist;
			dFloat speed;
			dFloat forceMag;

			// calculate separation and speed of hard points
			dVector threadPoint (threadMatrix.TransformVector(threadPointLocal));
			dVector parentPoint (parentMatrix.TransformVector(parentPointLocal));
			dist = (parentPoint - threadPoint) % parentMatrix.m_up;
			speed = ((parentVeloc + parentOmega * (parentPoint - parentCOM) -
					  threadVeloc - threadOmega * (threadPoint - threadCOM)) % parentMatrix.m_up);

			if (dist > MAX_COMPRESION_DIST) {
				NewtonUserJointAddLinearRow (m_joint, &threadPoint[0], &threadPoint[0], &parentMatrix.m_up[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			} else if (dist < MIN_EXPANSION_DIST) {
				// submit a contact constraint to prevent the body 
				NewtonUserJointAddLinearRow (m_joint, &threadPoint[0], &threadPoint[0], &parentMatrix.m_up[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} 

			// apply the spring force
			forceMag = NewtonCalculateSpringDamperAcceleration (timestep, SPRING_CONST, dist, DAMPER_CONST, speed) * m_massScale;


			dVector forceParent (parentMatrix.m_up.Scale (forceMag));
			dVector torqueParent ((parentPoint - parentCOM) * forceParent);
			NewtonBodyAddForce(m_body1, &forceParent[0]);
			NewtonBodyAddTorque(m_body1, &torqueParent[0]);
			
		
			dVector forceThread (forceParent.Scale (-1.0f));
			dVector torqueThread ((threadPoint - threadCOM) * forceThread);
			NewtonBodyAddForce(m_body0, &forceThread[0]);
			NewtonBodyAddTorque(m_body0, &torqueThread[0]);
		}

		void SubmitConstraints (dFloat timestep, int threadIndex)
		{
			// calculate suspension bumpers and forces
			dMatrix threadMatrix;
			dMatrix parentMatrix;

			dVector threadCOM;
			dVector parentCOM;
			dVector threadVeloc;
			dVector parentVeloc;
			dVector threadOmega;
			dVector parentOmega;


			// get the physics body state;
			NewtonBodyGetOmega(m_body0, &threadOmega[0]);
			NewtonBodyGetOmega(m_body1, &parentOmega[0]);

			NewtonBodyGetVelocity(m_body0, &threadVeloc[0]);
			NewtonBodyGetVelocity(m_body1, &parentVeloc[0]);

			NewtonBodyGetCentreOfMass(m_body0, &threadCOM[0]);
			NewtonBodyGetCentreOfMass(m_body1, &parentCOM[0]);

			NewtonBodyGetMatrix(m_body0, &threadMatrix[0][0]);
			NewtonBodyGetMatrix(m_body1, &parentMatrix[0][0]);

			threadCOM = threadMatrix.TransformVector(threadCOM);
			parentCOM = parentMatrix.TransformVector(parentCOM);
			
			ApplySuspesionForce (timestep,
				m_body0, m_rearHarpointOnThread, threadMatrix, threadCOM, threadVeloc, threadOmega,
				m_body1, m_rearHarpointOnParent, parentMatrix, parentCOM, parentVeloc, parentOmega);

			ApplySuspesionForce (timestep,
				m_body0, m_frontHarpointOnThread, threadMatrix, threadCOM, threadVeloc, threadOmega,
				m_body1, m_frontHarpointOnParent, parentMatrix, parentCOM, parentVeloc, parentOmega);


			CustomSlidingContact::SubmitConstraints (timestep, threadIndex);
		}


		dFloat m_massScale;
		dVector m_frontHarpointOnParent;
		dVector m_frontHarpointOnThread;

		dVector m_rearHarpointOnParent;
		dVector m_rearHarpointOnThread;
	};


	public:
//	FrictionTrank(const dModel& sourceModel, SceneManager* system, NewtonWorld* nWorld, const dMatrix& matrix, int bodyID, int tracksID)
//		:RenderPrimitive(matrix, NULL)
	FrictionTrank()
	{
_ASSERTE (0);
/*
		// open the level data
		InitFromModel (sourceModel);
		system->Append (this);	
		SetMatrix (matrix);

		// create the main body physic object
		m_mainBone = FindBone ("mainBody");
		m_mainBody = CreateBodyPart (m_mainBone, nWorld, 100.0f);
		NewtonBodySetTransformCallback (m_mainBody, SetTransform);

		// create the right thread body 
		m_rightThread = FindBone ("track_right");
		m_rightThreadBody = CreateBodyPart (m_rightThread, nWorld, 100.0f);

		// create the left thread body 
		m_leftThread = FindBone ("track_left");
		m_leftThreadBody = CreateBodyPart (m_leftThread, nWorld, 100.0f);

		// lower the center of mass of the main body to give more stability
		dVector com;
		NewtonBodyGetCentreOfMass(m_mainBody, &com[0]);
		com.m_y -= 0.5f;
		NewtonBodySetCentreOfMass(m_mainBody, &com[0]);

		// set the material ID for the threads
		NewtonBodySetMaterialGroupID(m_mainBody, bodyID);
		NewtonBodySetMaterialGroupID(m_leftThreadBody, tracksID);
		NewtonBodySetMaterialGroupID(m_rightThreadBody, tracksID);

		dMatrix leftThreadPinAndPivot;
		NewtonBodyGetMatrix (m_leftThreadBody, &leftThreadPinAndPivot[0][0]);
		leftThreadPinAndPivot.m_front = matrix.m_up;
		leftThreadPinAndPivot.m_up = matrix.m_right;
		leftThreadPinAndPivot.m_right = leftThreadPinAndPivot.m_front * leftThreadPinAndPivot.m_up;
		new FrictionTrankThreaSuspention (leftThreadPinAndPivot, m_leftThreadBody, m_mainBody);

		dMatrix rightThreadPinAndPivot;
		NewtonBodyGetMatrix (m_rightThreadBody, &rightThreadPinAndPivot[0][0]);
		rightThreadPinAndPivot.m_front = matrix.m_up;
		rightThreadPinAndPivot.m_up = matrix.m_right;
		rightThreadPinAndPivot.m_right = rightThreadPinAndPivot.m_front * rightThreadPinAndPivot.m_up;
		new FrictionTrankThreaSuspention (rightThreadPinAndPivot, m_rightThreadBody, m_mainBody);

		// add a track control joint
		m_controller = new FrictionTractionControl (m_mainBody, m_leftThreadBody, m_rightThreadBody);
*/
	}


	~FrictionTrank(void)
	{
	}

/*
	NewtonBody* GetBody() const 
	{
		return m_myBody;
	}

	void SetSteering (float steer)
	{
		m_controller->m_turnVeloc = steer * 3.0f;
		
	}

	void SetTireTorque(float torque)
	{
		m_controller->m_veloc = torque * 6.0f;
	}

*/

	static void SetTransform (const NewtonBody* body, const dFloat* matrix, int threadIndex)
	{
_ASSERTE (0);
/*
		FrictionTrank* me;
		me = (FrictionTrank*) NewtonBodyGetUserData (body);

		const dMatrix& rootMatrix = *((dMatrix*) matrix);
//		me->m_matrix = me->m_mainBone->m_localMatrix.Inverse() * rootMatrix;
		me->SetMatrix(rootMatrix);

		// calculate the local matrix of the threads 
		dMatrix threadMatrix;
		NewtonBodyGetMatrix (me->m_leftThreadBody, &threadMatrix[0][0]);
		me->m_leftThread->m_localMatrix = threadMatrix * me->m_leftThread->GetParent()->CalcGlobalMatrix().Inverse();

		NewtonBodyGetMatrix (me->m_rightThreadBody, &threadMatrix[0][0]);
		me->m_rightThread->m_localMatrix = threadMatrix * me->m_rightThread->GetParent()->CalcGlobalMatrix().Inverse();;
*/
	}

	private:
//	NewtonBody* CreateBodyPart (dBone* bone, NewtonWorld* nWorld, dFloat mass) const
	NewtonBody* CreateBodyPart () const
	{
_ASSERTE (0);
return NULL;
/*

		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dMesh* geomtry;
		NewtonBody* body;
		NewtonCollision* collision;
		dVector origin;
		dVector inertia;
		//dVector vertex[1024 * 32];

		// find the main body part
		geomtry = FindMesh (bone->GetName());

		// iterate again collecting the vertex array
		collision = NewtonCreateConvexHull (nWorld, geomtry->m_vertexCount, geomtry->m_vertex, 3 * sizeof (dFloat), 0.1f, 0, NULL);

		//create the rigid body
		body = NewtonCreateBody (nWorld, collision);


		// release the collision 
		NewtonReleaseCollision (nWorld, collision);	

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (body, (void*)this);

		// set the material group id for vehicle
		NewtonBodySetMaterialGroupID (body, 0);

		// set a destructor for this rigid body
		NewtonBodySetDestructorCallback (body, PhysicsBodyDestructor);

		// set the force and torque call back function
		NewtonBodySetForceAndTorqueCallback (body, PhysicsApplyGravityForce);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

		Ixx = mass * inertia[0];
		Iyy = mass * inertia[1];
		Izz = mass * inertia[2];

		// set the mass matrix
		NewtonBodySetMassMatrix (body, mass, Ixx, Iyy, Izz);

		// Set the vehicle Center of mass
		// the rear spoilers race the center of mass by a lot for a race car
		// we need to lower some more for the geometrical value of the y axis
		NewtonBodySetCentreOfMass (body, &origin[0]);

		// set the matrix for both the rigid body and the graphic body
		dMatrix matrix (bone->CalcGlobalMatrix(FindBone(0)) * GetMatrix());
		NewtonBodySetMatrix (body, &matrix[0][0]);

		return body;
*/
	}

//	dBone* m_mainBone;
//	dBone* m_leftThread;
//	dBone* m_rightThread;


	NewtonBody* m_mainBody;
	NewtonBody* m_leftThreadBody;
	NewtonBody* m_rightThreadBody;
	FrictionTractionControl* m_controller;
};





void TracktionJoints (NewtonFrame& system)
{
_ASSERTE (0);
/*
	NewtonWorld* world;
//	NewtonBody* floor; 
	LevelPrimitive *level;
	HeightFieldPrimitive* map;

	world = system.m_world;

	// create the sky box and the floor,
	level = LoadLevelAndSceneRoot (system, "flatplane.dae", 0);

	// create a material to collision with this object
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID (world);

	// delete the existing levelMesh
	NewtonDestroyBody (world, level->m_level);
	system.RemoveModel (level);

	// add a height map mesh 
	map = new HeightFieldPrimitive (world);
	system.AddModel___ (map);
	map->Release();

	dVector posit (-20.0f, 0.0f, 0.0f, 0.0f);
	posit.m_y = FindFloor (system.m_world, cameraEyepoint.m_x, cameraEyepoint.m_z) + 4.0f;
	InitEyePoint (dVector (1.0f, 0.0f, 0.0f), posit);

	int defaultID;
	int tracksID;

	defaultID = NewtonMaterialGetDefaultGroupID(system.m_world);
	tracksID = NewtonMaterialCreateGroupID(system.m_world);

	// load a model to be instantiated few time
	OGLModel model;
	char fullPathName[2048];
	GetWorkingFileName ("m1a1.dae", fullPathName);
	OGLLoaderContext context;
	dMatrix rotMatrix (dYawMatrix (-3.14159265f * 0.5f));

_ASSERTE (0);
//	model.LoadCollada(fullPathName, context, rotMatrix, 1.0f);

	for (int x = 0; x < 3; x ++) {
		for (int z = 0; z < 3; z ++) {
			dVector point (cameraEyepoint + dVector (x * 18.0f + 30.0f, 0.0f, z * 18.0f, 1.0f));

			point.m_w = 1.0f;
			dMatrix matrix (GetIdentityMatrix());
			matrix.m_posit = point;
			matrix.m_posit.m_y = FindFloor (system.m_world, point.m_x, point.m_z) + 4.0f;
			new FrictionTrank (model, &system, system.m_world, matrix, defaultID, tracksID);
		}
	}
*/
}

