/*
* Vehicle Multi body Joint + Demo write by Dave Gravel 2018.
* I have write this vehicle multi body code for share with other newton users, and maybe it can help someone.
* 
* Have fun!!!
*
* Informations, Problems:
* You can contact me from the newton forum http://newtondynamics.com/forum/index.php
* Or you can message me on https://www.facebook.com/dave.gravel1
* My youtube channel https://www.youtube.com/user/EvadLevarg/videos
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "HeightFieldPrimitive.h"
#include "DebugDisplay.h"

#define D_MULTIBODY_TIRE_ID							0x26ab752c
#define D_MULTIBODY_TIRE_MAX_ELASTIC_DEFORMATION	(0.05f)

static int UserOnAABBOverlap(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
{
#ifdef _DEBUG
	NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
	NewtonJoint* const contact0 = NewtonBodyFindContact(body0, body1);
	NewtonJoint* const contact1 = NewtonBodyFindContact(body1, body0);
	dAssert(!contact0 || contact0 == contactJoint);
	dAssert(!contact1 || contact1 == contactJoint);
#endif	
	return 1;
}

static void UserContactFriction(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	// call  the basic call back
	GenericContactProcess(contactJoint, timestep, threadIndex);

	const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
	const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

	//now core 3.14 can have per collision user data
	const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
	const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

	NewtonCollisionMaterial material0;
	NewtonCollisionMaterial material1;
	NewtonCollisionGetMaterial(collision0, &material0);
	NewtonCollisionGetMaterial(collision1, &material1);
	//dAssert((material0.m_userId == 1) || (material1.m_userId == 1));
	//dFloat frictionValue = dMax(material0.m_userParam[0], material1.m_userParam[0]);

    // D.G: Normally in my engine I set the material manually body by body userdata, here the material is overall body in the scene.
	// D.G: It's better to implement it by body with your userdata, Because the tire and the vehicle frame need different values.
	// D.G: It's more easy to get personal tire material or frame from the userdata for set your desired values. 
	// D.G: Normally the frame use less friction that the ruber tire do.
	// D.G: The frame or tire can have different bounce values too.
	// D.G: With this setup it can give strange collision result from body frame vs body frame collisions.
	for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
		NewtonMaterial* const material = NewtonContactGetMaterial(contact);
		NewtonMaterialSetContactFrictionCoef(material, 1.4f, 1.4f, 0);
		NewtonMaterialSetContactFrictionCoef(material, 1.4f, 1.4f, 1);
	}
}

static int OnTireContactGeneration(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex)
{
	int id0 = NewtonCollisionGetUserID(collision0);

	//const NewtonBody* const tire = (id0 == D_MULTIBODY_TIRE_ID) ? body0 : body1;
	const NewtonCollision* const tireCollision = (id0 == D_MULTIBODY_TIRE_ID) ? collision0 : collision1;

	const NewtonBody* const terrain = (id0 == D_MULTIBODY_TIRE_ID) ? body1 : body0;
	const NewtonCollision* const terrainCollision = (id0 == D_MULTIBODY_TIRE_ID) ? collision1 : collision0;

	// get the joint information form the collision user data
	NewtonCollisionMaterial collisionMaterial;
	NewtonCollisionGetMaterial(tireCollision, &collisionMaterial);
	dAssert(collisionMaterial.m_userId == D_MULTIBODY_TIRE_ID);

	dCustomTireSpringDG* const tireJoint = (dCustomTireSpringDG*)collisionMaterial.m_userData;
	dAssert(tireJoint->GetBody1() == ((id0 == D_MULTIBODY_TIRE_ID) ? body0 : body1));

	dMatrix tireMatrix;
	dMatrix tireHarpointMatrix;
	tireJoint->CalculateGlobalMatrix(tireHarpointMatrix, tireMatrix);

	// here I need the suspension length, for now assume 1 meter
	dFloat suspensionSpan = 1.0f;
	tireHarpointMatrix.m_posit += tireHarpointMatrix.m_up.Scale(suspensionSpan);

	//dVector veloc0(tireHarpointMatrix.m_up.Scale(-m_info.m_suspensionLength));
	dVector veloc0(tireHarpointMatrix.m_up.Scale(-suspensionSpan));
	dVector tmp(0.0f);
	dVector contact(0.0f);
	dVector normal(0.0f);
	dFloat penetration(0.0f);

	dFloat position = tireHarpointMatrix.m_up.DotProduct3(tireHarpointMatrix.m_posit - tireMatrix.m_posit);
	dFloat param = position / suspensionSpan;

	dMatrix matrixB;
	dLong attributeA;
	dLong attributeB;
	dFloat impactParam;

	NewtonWorld* const world = NewtonBodyGetWorld(terrain);
	NewtonBodyGetMatrix(terrain, &matrixB[0][0]);

	int count = NewtonCollisionCollideContinue(world, 1, 1.0f,
		tireCollision, &tireHarpointMatrix[0][0], &veloc0[0], &tmp[0],
		terrainCollision, &matrixB[0][0], &tmp[0], &tmp[0],
		&impactParam, &contact[0], &normal[0], &penetration,
		&attributeA, &attributeB, 0);

	int contactCount = 0;
	if (count) {
		// calculate tire penetration
		dFloat dist = (param - impactParam) * suspensionSpan;

		if (dist > -D_MULTIBODY_TIRE_MAX_ELASTIC_DEFORMATION) {

			normal.m_w = 0.0f;
			penetration = normal.DotProduct3(tireHarpointMatrix.m_up.Scale(dist));

			dVector longitudinalDir(normal.CrossProduct(tireMatrix.m_front));

			if (longitudinalDir.DotProduct3(longitudinalDir) < 0.1f) {
				dAssert(0);
				//lateralDir = normal.CrossProduct(tireMatrix.m_front.CrossProduct(normal)); 
				longitudinalDir = normal.CrossProduct(tireMatrix.m_up.CrossProduct(normal));
				dAssert(longitudinalDir.DotProduct3(longitudinalDir) > 0.1f);
			}
			longitudinalDir = longitudinalDir.Normalize();
			contact -= tireMatrix.m_up.Scale(dist);

			contactBuffer[contactCount].m_point[0] = contact.m_x;
			contactBuffer[contactCount].m_point[1] = contact.m_y;
			contactBuffer[contactCount].m_point[2] = contact.m_z;
			contactBuffer[contactCount].m_point[3] = 1.0f;
			contactBuffer[contactCount].m_normal[0] = normal.m_x;
			contactBuffer[contactCount].m_normal[1] = normal.m_y;
			contactBuffer[contactCount].m_normal[2] = normal.m_z;
			contactBuffer[contactCount].m_normal[3] = 0.0f;
			contactBuffer[contactCount].m_shapeId0 = collisionMaterial.m_userId;
			contactBuffer[contactCount].m_shapeId1 = NewtonCollisionGetUserID(terrainCollision);
			contactBuffer[contactCount].m_penetration = dClamp(penetration, dFloat(-D_MULTIBODY_TIRE_MAX_ELASTIC_DEFORMATION), dFloat(D_MULTIBODY_TIRE_MAX_ELASTIC_DEFORMATION));
			contactCount++;
		}
	}

	return contactCount;
}


#if 0
class VehicleControllerManagerDG : public dCustomVehicleControllerManagerDG
{
	public:
	class VehicleFrameEntity : public DemoEntity
	{
	private:
		int mTireCount;
		int mMaxTires;
		dFloat mFrameTorqueAffective;
		dFloat mVehicleSpeed;
		dCustomTireSpringDG** mTireJoint;
		VehicleControllerManagerDG* mManager;
		dMatrix mInitialMatrix;
		DemoMesh* mVehicleFrameMesh1;
		DemoMesh* mVehicleFrameMesh2;
		NewtonBody* mFrameBody;
	public:
		VehicleFrameEntity(DemoEntityManager* const scene, VehicleControllerManagerDG* const manager, const int maxTire, const dMatrix& location)
			:DemoEntity(dGetIdentityMatrix(), NULL),
			mManager(manager),
			mTireJoint(NULL),
			mMaxTires(maxTire),
			mInitialMatrix(location),
			mTireCount(0),
			mFrameBody(NULL),
			mVehicleSpeed(0.0f),
			mEngineFpsRequest(120.0f),
			mFrameTorqueAffective(0.25f)
		{
			mTireJoint = new dCustomTireSpringDG*[maxTire];		
			scene->Append(this);
		}

		~VehicleFrameEntity()
		{
			delete[] mTireJoint;
		}
		//
		void SetEngineFpsRequest(dFloat val)
		{
			mEngineFpsRequest = val;
		}
		//
		void SetFrameTorqueAffective(dFloat val)
		{
			mFrameTorqueAffective = val;
		}
		//
		dFloat GetEngineFpsRequest()
		{
			return mEngineFpsRequest;
		}
		//
		dFloat GetFrameTorqueAffective()
		{
			return mFrameTorqueAffective;
		}
		//
		NewtonBody* GetFrameBody()
		{
			return mFrameBody;
		}
		//
		NewtonBody* GetTireBody(int tireID)
		{
			return mTireJoint[tireID]->GetBody1();
		}
		//
		int GetMaxTire()
		{
			return mMaxTires;
		}
		//
		dMatrix GetInitialMatrix()
		{
			return mInitialMatrix;
		}
		//
		dCustomTireSpringDG* GetTireJoint(int jointID)
		{
			return mTireJoint[jointID];
		}
		//
		int GetTireCount()
		{
			return mTireCount;
		}
		//
		virtual void SetVehicleFrameMesh(DemoMesh* dVehicleMesh)
		{
			mVehicleFrameMesh1 = dVehicleMesh;
		}
		//
		virtual void SetVehicleFrameMesh(DemoMesh* dVehicleMesh1, DemoMesh* dVehicleMesh2)
		{
			mVehicleFrameMesh1 = dVehicleMesh1;
			mVehicleFrameMesh2 = dVehicleMesh2;
		}
		//
		void AddVehicleFrameBody(NewtonBody* const vFrame, dVector const vCenterMass, dFloat const Gravedad = -9.81f)
		{
			mFrameBody = vFrame;
			//
			mManager->CreateVehicle(vFrame, mInitialMatrix, NULL, Gravedad);
			NewtonBodySetCentreOfMass(vFrame, &vCenterMass[0]);
			mManager->mVehicleList.Append(this);
			//
		}
		//
		dCustomTireSpringDG* AddTireSuspenssion(NewtonBody* const vTire, dMatrix vTireMatrix)
		{
			mTireJoint[mTireCount] = new dCustomTireSpringDG(vTireMatrix, mFrameBody, vTire);
			mTireJoint[mTireCount]->SetSolverModel(2);
			mTireJoint[mTireCount]->SetEngineFpsRequest(GetEngineFpsRequest());
			//
			mTireCount++;
			return mTireJoint[mTireCount-1];
		}
		//
		void VehicleSimulationPreListener(dFloat timestep)
		{
			dVector FrameTorque;
			dVector FrameForce;
			dVector AttVeloc;
			dVector frmVeloc;
			dVector frmOmega;
			dVector frmCom;
			dVector AttachForce;
			dMatrix frmMatrix;
			dFloat speed;
			dFloat frameMass;
			dFloat vIxx;
			dFloat vIyy;
			dFloat vIzz;
			dVector tireTorque;
			dVector chassisReationTorque;
			//
			NewtonBodySetSleepState(GetFrameBody(), 0);
			NewtonBodyGetMass(GetFrameBody(), &frameMass, &vIxx, &vIyy, &vIzz);
			//
			for (int i = 0; i < GetMaxTire(); i++) {
				dCustomTireSpringDG* cTireSpring = GetTireJoint(i);
				//
				frmCom = dVector(0.0f, 0.0f, 0.0f, 0.0f);
				frmVeloc = dVector(0.0f, 0.0f, 0.0f, 0.0f);
				FrameTorque = dVector(0.0f, 0.0f, 0.0f, 0.0f);
				FrameForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
				//
				cTireSpring->TireMatrixProjection();
				//
				NewtonBodySetSleepState(cTireSpring->GetBody1(), 0);
				NewtonBodyGetVelocity(cTireSpring->GetBody1(), &AttVeloc[0]);
				NewtonBodyGetVelocity(GetFrameBody(), &frmVeloc[0]);
				NewtonBodyGetOmega(GetFrameBody(), &frmOmega[0]);
				NewtonBodyGetMatrix(GetFrameBody(), &frmMatrix[0][0]);
				NewtonBodyGetCentreOfMass(GetFrameBody(), &frmCom[0]);
				//
				mVehicleSpeed = dAbs(frmVeloc.DotProduct3(cTireSpring->GetChassisPivotMatrix().m_right));
				//
				frmCom = frmMatrix.TransformVector(frmCom); //OXTransformVector(frmCom, frmMatrix); 
				frmVeloc = frmVeloc + frmOmega.CrossProduct(cTireSpring->GetCenterInChassis() - frmCom);
				//
			    speed = (frmVeloc - AttVeloc).DotProduct3(cTireSpring->GetChassisPivotMatrix().m_up);
				//
				cTireSpring->SetDistance();
				//
				dFloat effectiveMassHack = 0.5f;
				cTireSpring->SetAccel(NewtonCalculateSpringDamperAcceleration(timestep, cTireSpring->GetSpringK(), cTireSpring->GetDistance(), 
									  cTireSpring->GetSpringD(), speed) * frameMass * effectiveMassHack);
				//
			    FrameForce = (cTireSpring->GetChassisPivotMatrix().m_up * cTireSpring->GetAccel());
			    FrameTorque = (cTireSpring->GetChassisPivotMatrix().m_posit - frmCom).CrossProduct(FrameForce);
				//
				NewtonBodyAddForce(GetFrameBody(), &FrameForce[0]);
				NewtonBodyAddTorque(GetFrameBody(), &FrameTorque[0]);
				//
			    AttachForce = (FrameForce * VEHICLE_ATTACH_FORCE_SIDE);
				NewtonBodyAddForce(cTireSpring->GetBody1(), &AttachForce[0]);
				//
				// When only normal break is use it apply the torque anyway, Because with the 4x4 model the rear tire can break and the front tire can accelerate in same time.
				// The result is a bit strange the vehicle can slide a bit more when you break with 4x4 model and if you accelerate in same time.
				if (cTireSpring->GetUseTorque() && (!cTireSpring->GetUseHardBreak()))
				{
				  // apply engine torque plus some tire angular drag
				  tireTorque = (cTireSpring->GetChassisPivotMatrix().m_front * (cTireSpring->GetTireTorque() - cTireSpring->GetRealTireOmega() * cTireSpring->GetTireIzz()));
				  NewtonBodyAddTorque(cTireSpring->GetBody1(), &tireTorque[0]);
				}
				//
			    chassisReationTorque = (cTireSpring->GetChassisPivotMatrix().m_front * -(cTireSpring->GetTireTorque() * GetFrameTorqueAffective()));
			    NewtonBodyAddTorque(GetFrameBody(), &chassisReationTorque[0]);
			    // 
				//cTireSpring->SetTireTorque(0.0f);
			}
		}
		//
		void VehicleSimulationPostListener(dFloat timestep)
		{

		}
		//
		dFloat mEngineFpsRequest;
	};
	//
	class VehicleInputManager : public dCustomInputManager
	{
	public:
		VehicleInputManager(DemoEntityManager* const scene)
			:dCustomInputManager(scene->GetNewton()),
			m_scene(scene)
		{
			// plug a callback for 2d help display
			scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);
			//scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		}
		//
		void OnBeginUpdate(dFloat timestepInSecunds)
		{
			int Accelerator = int(m_scene->GetKeyState(264)) - int(m_scene->GetKeyState(265));
			int SteerAction = int(m_scene->GetKeyState(263)) - int(m_scene->GetKeyState(262));
			int BreakAction = int(m_scene->GetKeyState('B')) - int(m_scene->GetKeyState(32));

#if 0
	#if 0
			static FILE* file = fopen("log.bin", "wb");
			if (file) {
				fwrite(&Accelerator, sizeof(int), 1, file);
				fflush(file);
			}
	#else 
			static FILE* file = fopen("log.bin", "rb");
			if (file) {
				fread(&Accelerator, sizeof(int), 1, file);
			}
	#endif
#endif


			//int HardBreakAction = ;
			//
			if (m_vehicle) {
              // The best is to create a engine value with graduation for the torque.
			  // On this way with gears you can get the perfect torque to give without make the tire spin to much.
			  if (Accelerator < 0) {
				  m_vehicle->GetTireJoint(0)->SetTireTorque(1600.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
				  m_vehicle->GetTireJoint(1)->SetTireTorque(-1600.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
				  m_vehicle->GetTireJoint(2)->SetTireTorque(1600.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
				  m_vehicle->GetTireJoint(3)->SetTireTorque(-1600.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
			  } else		
			  if (Accelerator > 0) {
				  m_vehicle->GetTireJoint(0)->SetTireTorque(-900.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
				  m_vehicle->GetTireJoint(1)->SetTireTorque(900.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
				  m_vehicle->GetTireJoint(2)->SetTireTorque(-900.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
				  m_vehicle->GetTireJoint(3)->SetTireTorque(900.0f * timestepInSecunds * m_vehicle->GetEngineFpsRequest());
			  }
			  else {
				  m_vehicle->GetTireJoint(0)->SetTireTorque(0.0f);
				  m_vehicle->GetTireJoint(1)->SetTireTorque(0.0f);
				  m_vehicle->GetTireJoint(2)->SetTireTorque(0.0f);
				  m_vehicle->GetTireJoint(3)->SetTireTorque(0.0f);
			  }
			  // It is pretty same here you can general a value from zero to the desired angle with graduation.
			  // On this way you can have more control on the steer and it can acting better and smoother.
			  if (SteerAction < 0) {
				  m_vehicle->GetTireJoint(0)->SetTireSteer(-35.0f);
				  m_vehicle->GetTireJoint(1)->SetTireSteer(-35.0f);
				 // m_vehicle->GetTireJoint(2)->SetTireSteer(-35.0f);
				 // m_vehicle->GetTireJoint(3)->SetTireSteer(-35.0f);
			  }
			  else
			  if (SteerAction > 0) {
				  m_vehicle->GetTireJoint(0)->SetTireSteer(35.0f);
				  m_vehicle->GetTireJoint(1)->SetTireSteer(35.0f);
				 // m_vehicle->GetTireJoint(2)->SetTireSteer(35.0f);
				 // m_vehicle->GetTireJoint(3)->SetTireSteer(35.0f);
			  }
			  else {
				  m_vehicle->GetTireJoint(0)->SetTireSteer(0.0f);
				  m_vehicle->GetTireJoint(1)->SetTireSteer(0.0f);
				  // m_vehicle->GetTireJoint(2)->SetTireSteer(0.0f);
				  // m_vehicle->GetTireJoint(3)->SetTireSteer(0.0f);
			  }
			  //
			  if (BreakAction > 0) {
				 m_vehicle->GetTireJoint(2)->SetTireBreak(5000.0f * 2.0f);
				 m_vehicle->GetTireJoint(3)->SetTireBreak(5000.0f * 2.0f);
			  }
			  else 
			  if (BreakAction < 0) {
				  m_vehicle->GetTireJoint(0)->SetUseHardBreak(true);
				  m_vehicle->GetTireJoint(0)->SetTireBreak(5000.0f * 0.5f);
				  m_vehicle->GetTireJoint(1)->SetUseHardBreak(true);
				  m_vehicle->GetTireJoint(1)->SetTireBreak(5000.0f * 0.5f);
				  m_vehicle->GetTireJoint(2)->SetUseHardBreak(true);
				  m_vehicle->GetTireJoint(2)->SetTireBreak(5000.0f * 0.5f);
				  m_vehicle->GetTireJoint(3)->SetUseHardBreak(true);
				  m_vehicle->GetTireJoint(3)->SetTireBreak(5000.0f * 0.5f);
			  }
			  else {
				  m_vehicle->GetTireJoint(0)->SetTireBreak(0.0f);
				  m_vehicle->GetTireJoint(1)->SetTireBreak(0.0f);
				  m_vehicle->GetTireJoint(2)->SetTireBreak(0.0f);
				  m_vehicle->GetTireJoint(3)->SetTireBreak(0.0f);
				  m_vehicle->GetTireJoint(0)->SetUseHardBreak(false);
				  m_vehicle->GetTireJoint(1)->SetUseHardBreak(false);
				  m_vehicle->GetTireJoint(2)->SetUseHardBreak(false);
				  m_vehicle->GetTireJoint(3)->SetUseHardBreak(false);
			  }
			  //
			  /*if (HardBreakAction > 0) {
				//m_backupbreak[0] = m_vehicle->GetTireJoint(0)->GetUseBreak();
				//m_backupbreak[1] = m_vehicle->GetTireJoint(0)->GetUseBreak();
				//m_backupbreak[2] = m_vehicle->GetTireJoint(0)->GetUseBreak();
				//m_backupbreak[3] = m_vehicle->GetTireJoint(0)->GetUseBreak();
				  printf();
				m_vehicle->GetTireJoint(0)->SetUseHardBreak(true);
				m_vehicle->GetTireJoint(0)->SetTireBreak(50000.0f * 0.5f);
				m_vehicle->GetTireJoint(1)->SetUseHardBreak(true);
				m_vehicle->GetTireJoint(1)->SetTireBreak(50000.0f * 0.5f);
				m_vehicle->GetTireJoint(2)->SetUseHardBreak(true);
				m_vehicle->GetTireJoint(2)->SetTireBreak(50000.0f * 0.5f);
				m_vehicle->GetTireJoint(3)->SetUseHardBreak(true);
				m_vehicle->GetTireJoint(3)->SetTireBreak(50000.0f * 0.5f);
			  }
			  else {

			  }*/
			  //
			}
		}
		//
		void OnEndUpdate(dFloat timestepInSecunds)
		{
		}
		//
		void RenderPlayerHelp(DemoEntityManager* const scene)
		{
			dVector color(0.5f, 0.5f, 0.0f, 0.0f);
			scene->Print(color, "Vehicle Multibody by Dave Gravel");
			scene->Print(color, "Navigation Keys:");
			scene->Print(color, "drive forward:  Arrow Up");
			scene->Print(color, "drive backward: Arrow Down");
			scene->Print(color, "turn Left:      Arrow Left");
			scene->Print(color, "turn Right:     Arrow Right");
			scene->Print(color, "break:          B");
			scene->Print(color, "hardbreak:      Space");
		}
		//
		static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
		{
			VehicleInputManager* const me = (VehicleInputManager*)context;
			me->RenderPlayerHelp(scene);
		}
		//
		void SetVehicleEntity(VehicleFrameEntity* vehicleEnt)
		{
			m_vehicle = vehicleEnt;
		}
		//
	private:
		//bool m_backupbreak[4];
		DemoEntityManager* m_scene;
		VehicleFrameEntity* m_vehicle;
	};
	//
	//
	dList<VehicleFrameEntity*> mVehicleList;
	//
	VehicleControllerManagerDG(DemoEntityManager* const scene, NewtonWorld* const world, int materialsCount, int* const materialList)
		:dCustomVehicleControllerManagerDG(world, materialsCount, materialList),
		nworld(world),
		mscene(scene)
	{
		//
		// add an input Manage to manage the inputs and user interaction 
		minputManager = new VehicleInputManager(mscene);

		// setting up a use contact handle to calculate tire collision with terrain
		int defualtMaterial = NewtonMaterialGetDefaultGroupID(world);
		m_tireMaterial = NewtonMaterialCreateGroupID(world);

		NewtonMaterialSetCallbackUserData(world, m_tireMaterial, defualtMaterial, this);
		NewtonMaterialSetContactGenerationCallback(world, m_tireMaterial, defualtMaterial, OnTireContactGeneration);
		NewtonMaterialSetCollisionCallback(world, m_tireMaterial, defualtMaterial, UserOnAABBOverlap, UserContactFriction);
	}

	~VehicleControllerManagerDG()
	{
	}


	virtual void PreUpdate(dFloat timestep)
	{
		// Julio i'm not sure here if I do it correctly.
		// In pascal my implementation is a bit different for this.
		// I'm not sure if I have inherited correctly the class for this part.
		//
		// do the base class Pre update
		dCustomVehicleControllerManagerDG::PreUpdate(timestep);
		//
		for (dList<VehicleFrameEntity*>::dListNode* node = mVehicleList.GetFirst(); node; node = node->GetNext()) {
		  VehicleFrameEntity* const veh = node->GetInfo();
		  //
		  veh->VehicleSimulationPreListener(timestep);
		}
	}

	virtual void PostUpdate(dFloat timestep)
	{
		// do the base class post update
		dCustomVehicleControllerManagerDG::PostUpdate(timestep);


		// update the camera 
		//UpdateCamera(timestep);
	}

	virtual dCustomVehicleControllerDG* CreateVehicle(NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, dFloat gravityMag)
	{
		dCustomVehicleControllerDG* const controller = CreateController();
		cVcontroller = controller;
		controller->Init(body, vehicleFrame, forceAndTorque, gravityMag);
		return controller;
	}

	VehicleInputManager* GetInputManager()
	{
		return minputManager;
	}

	int GetTireMaterial() const
	{
		return m_tireMaterial;
	}

private:
	NewtonWorld* nworld;
	dCustomVehicleControllerDG* cVcontroller;
	// add an input Manage to manage the inputs and user interaction 
	VehicleInputManager* minputManager;
	DemoEntityManager* mscene;

	int m_tireMaterial;
};

static void BuildPyramid(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int count, PrimitiveType type, const dMatrix& shapeMatrix = dGetIdentityMatrix())
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();

	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);

	NewtonCollision* const collision = CreateConvexCollision(world, shapeMatrix, size, type, defaultMaterialID);
	DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "wood_4.tga", "wood_4.tga", "wood_1.tga");

	//	matrix = dRollMatrix(dPi/2.0f);
	dFloat startElevation = 100.0f;
	dVector floor(FindFloor(world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));

	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f;

	// get the dimension from shape itself
	dVector minP(0.0f);
	dVector maxP(0.0f);
	CalculateAABB(collision, dGetIdentityMatrix(), minP, maxP);

	dFloat stepz = maxP.m_z - minP.m_z + 0.03125f;
	dFloat stepy = (maxP.m_y - minP.m_y) - 0.01f;

	dFloat y0 = matrix.m_posit.m_y + stepy / 2.0f;
	dFloat z0 = matrix.m_posit.m_z - stepz * count / 2;

	matrix.m_posit.m_y = y0;
	for (int j = 0; j < count; j++) {
		matrix.m_posit.m_z = z0;
		for (int i = 0; i < (count - j); i++) {
			CreateSimpleSolid(scene, geometry, mass, matrix, collision, defaultMaterialID);
			matrix.m_posit.m_z += stepz;
		}
		z0 += stepz * 0.5f;
		matrix.m_posit.m_y += stepy;
	}

	// do not forget to release the assets	
	geometry->Release();
	NewtonDestroyCollision(collision);
}


dFloat dRandRangeFloat(dFloat amin, dFloat amax)
{
	dFloat r = (dFloat)dRand() / (dFloat)RAND_MAX;
	return (amin + r * (amax - amin));
}

NewtonBody* VehicleTireCreate(DemoEntityManager* scene, VehicleControllerManagerDG::VehicleFrameEntity* const vVehicle, dFloat const tireMass, dFloat tireRad, dFloat tireHeight, dVector tirePos, int tireMaterial)
{
	NewtonWorld* const world = scene->GetNewton();
	dMatrix matrixVehicle = vVehicle->GetInitialMatrix();
	dMatrix matrix = dGetIdentityMatrix();
	//
	dVector tireRot = dVector(0.0f, 0.0f, 0.0f);
	dVector angdamp = dVector(0.0f, 0.0f, 0.0f);
	//
	// rotate the model for when you use 3d mesh, to make if facing on the good side.
	//
	if ((vVehicle->GetTireCount() % 2) == 0) 
	{
	  tireRot = dVector(90.0f, 0.0f, 0.0f);
	}
	else {
	  tireRot = dVector(-90.0f, 0.0f, 0.0f);
	}
	//matrix
	if (tireRot.m_x != 0.0f) matrix = dYawMatrix(tireRot.m_x * dDegreeToRad);
	if (tireRot.m_y != 0.0f) matrix = dRollMatrix(tireRot.m_y * dDegreeToRad);
	if (tireRot.m_z != 0.0f) matrix = dPitchMatrix(tireRot.m_z * dDegreeToRad);
	//
	matrix = (matrix * matrixVehicle);
	//
	matrix.m_posit = matrixVehicle.TransformVector(tirePos);
	//
	// some extra char in case the count go higher.
	char buff[24] = "";
	sprintf(buff, "VehicleTireMesh%d", vVehicle->GetTireCount());
	NewtonCollision* collision = NewtonCreateChamferCylinder(world, tireRad, tireHeight, 0, NULL);
	DemoMesh* const VehicleTireMesh = new DemoMesh(&buff[0], collision, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const vBody = CreateSimpleSolid(scene, VehicleTireMesh, tireMass, matrix, collision, tireMaterial);
	VehicleTireMesh->Release();

	// Make the tire 100% free rolling(spinning).
	NewtonBodySetLinearDamping(vBody, 0.0);
	NewtonBodySetAngularDamping(vBody, &angdamp[0]);
	//
	NewtonBodySetAutoSleep(vBody, 0);
	NewtonBodySetFreezeState(vBody, 0);
	NewtonBodySetMatrix(vBody, &matrix[0][0]);
	//
	NewtonDestroyCollision(collision); 
	//
	dCustomTireSpringDG* const tireJoint = vVehicle->AddTireSuspenssion(vBody, matrix);
		
	// set the tire material, you can set other stuff her that you can use in the call back
	NewtonCollisionMaterial collisionMaterial;
	NewtonCollisionGetMaterial(NewtonBodyGetCollision(vBody), &collisionMaterial);
	collisionMaterial.m_userId = D_MULTIBODY_TIRE_ID;
	collisionMaterial.m_userData = tireJoint;
	NewtonCollisionSetMaterial(NewtonBodyGetCollision(vBody), &collisionMaterial);
	NewtonBodySetMaterialGroupID(vBody, tireMaterial);

	//
	return vBody;
}

NewtonBody* VehicleFrameCreate(DemoEntityManager* scene, VehicleControllerManagerDG::VehicleFrameEntity* const vVehicle, dFloat const vMass, dVector const vCenterMass, dVector const vScal)
{
	NewtonWorld* const world = scene->GetNewton();
	// create an empty compound collision
	//NewtonCollision* const compound = NewtonCreateCompoundCollision(world, 0);
	//NewtonCompoundCollisionBeginAddRemove(compound);

	dMatrix matrix = dGetIdentityMatrix();

	dVector frameRot = dVector(0.0f, 0.0f, 0.0f);
	dVector angdamp = dVector(0.0f, 0.0f, 0.0f);

	if (frameRot.m_x != 0.0f) matrix = dYawMatrix(frameRot.m_x * dDegreeToRad);
	if (frameRot.m_y != 0.0f) matrix = dRollMatrix(frameRot.m_y * dDegreeToRad);
	if (frameRot.m_z != 0.0f) matrix = dPitchMatrix(frameRot.m_z * dDegreeToRad);

	matrix = matrix * vVehicle->GetInitialMatrix();

	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);

	NewtonCollision* collision = NewtonCreateBox(world, vScal.m_x, vScal.m_y, vScal.m_z, 0, NULL);
	//
	DemoMesh* const VehicleFrameMesh1 = new DemoMesh("VehicleFrameMesh1", collision, "wood_0.tga", "smilli.tga", "wood_0.tga");
	NewtonBody* const vBody = CreateSimpleSolid(scene, VehicleFrameMesh1, vMass, matrix, collision, defaultMaterialID);
	VehicleFrameMesh1->Release();
	// Make the tire 100% free rolling(spinning).
	NewtonBodySetLinearDamping(vBody, 0.0);
	NewtonBodySetAngularDamping(vBody, &angdamp[0]);
	//
	NewtonBodySetAutoSleep(vBody, 0);
	NewtonBodySetFreezeState(vBody, 0);
	NewtonBodySetMatrix(vBody, &matrix[0][0]);
	//
	NewtonDestroyCollision(collision);
	/*
	// we can set a collision id, and use data per sub collision 
	NewtonCollisionSetUserID(collision, 0);
	// add this new collision 
	NewtonCompoundCollisionAddSubCollision(compound, collision);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//
	//
	matrix.m_posit = dVector(matrix.m_posit.m_x, matrix.m_posit.m_y + 2, matrix.m_posit.m_z);
	collision = NewtonCreateBox(world, vScal.m_x, vScal.m_y, vScal.m_z, 0, &matrix[0][0]);
	//
	DemoMesh* const VehicleFrameMesh2 = new DemoMesh("VehicleFrameMesh2", collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
	// we can set a collision id, and use data per sub collision 
	NewtonCollisionSetUserID(collision, 1);
	// add this new collision 
	NewtonCompoundCollisionAddSubCollision(compound, collision);
	NewtonDestroyCollision(collision);
	collision = NULL;
	// finish adding shapes
	NewtonCompoundCollisionEndAddRemove(compound);
	*/
	//scene->Append(VehicleFrameMesh1);
	//scene->Append(VehicleFrameMesh2);geometry->Release(); 
	//
	vVehicle->SetVehicleFrameMesh(VehicleFrameMesh1);
	//NewtonDestroyCollision(compound);
	vVehicle->AddVehicleFrameBody(vBody, vCenterMass);
	//

	//
	return vBody;
}

void BasicMultibodyVehicle(DemoEntityManager* const scene)
{
	//////////////////////////////////////////////////////
	// Scene 3D
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh(scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 10, 8.0f, 5.0f, 0.2f, 200.0f, -50.0f);
	//
	NewtonWorld* const world = scene->GetNewton();
	//
	int defaultMaterial = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	NewtonMaterialSetDefaultFriction(world, defaultMaterial, defaultMaterial, 0.6f, 0.5f);
	//
	int materialList[] = { defaultMaterial };
	//
	dMatrix location(dGetIdentityMatrix());
	location.m_posit = dVector(FindFloor(world, dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	location.m_posit.m_y += 1.0f;
	//
	dMatrix startlocation(dGetIdentityMatrix());
	dMatrix startlocation2 = startlocation;
	dMatrix startlocation3 = startlocation;
	//
	startlocation = startlocation * dRollMatrix(25.0f * dDegreeToRad);
	startlocation.m_posit = dVector(FindFloor(world, dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	startlocation.m_posit.m_x += 50.0f;
	startlocation.m_posit.m_y += 8.5f;
	NewtonCollision* collision = NewtonCreateBox(world, 50.0f, 0.25f, 100.0f, 0, NULL);
//	DemoMesh* const StartBox = new DemoMesh("StartBox", collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
//	CreateSimpleSolid(scene, StartBox, 0.0f, startlocation, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//
	//startlocation2 = startlocation2 * dRollMatrix(2.5f * dDegreeToRad);
	startlocation2.m_posit.m_x += 38.0f;
	startlocation2.m_posit.m_y += 4.0f;
	collision = NewtonCreateBox(world, 20.0f, 0.25f, 40.0f, 0, NULL);
//	DemoMesh* const StartBox3 = new DemoMesh("StartBox", collision, "smilli.tga", "smilli.tga", "smilli.tga");
//	CreateSimpleSolid(scene, StartBox3, 0.0f, startlocation2, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//	
	startlocation3 = startlocation3 * dRollMatrix(32.5f * dDegreeToRad);
	startlocation3.m_posit.m_x -= 31.5f;
	startlocation3.m_posit.m_y += 0.575f;
	collision = NewtonCreateBox(world, 2.0f, 2.0f, 100.0f, 0, NULL);
//	DemoMesh* const StartBox2 = new DemoMesh("StartBox", collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
//	CreateSimpleSolid(scene, StartBox2, 0.0f, startlocation3, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//
//	BuildPyramid(scene, 10.0f, dVector(-20.0f, 0.0f, 10.0f, 1.0f), dVector(0.5f, 0.25f, 0.8f, 0.0), 10, _BOX_PRIMITIVE);
//	BuildPyramid(scene, 10.0f, dVector(-20.0f, 0.0f, -10.0f, 1.0f), dVector(0.5f, 0.25f, 0.8f, 0.0), 10, _BOX_PRIMITIVE);
	//
	location.m_posit = dVector(FindFloor(scene->GetNewton(), dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	dVector origin(FindFloor(world, dVector(-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	origin.m_y += 10.0f;
	origin.m_x += -45.0f;
	//
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
	//
	//////////////////////////////////////////////////////
	// Vehicle Multibody construction
	// Randomize Values.
	dSetRandSeed((int)time(NULL));
	dFloat drandRangef = dRandRangeFloat(-90.0f, 90.0f);
	// Set vehicle rotation and position
	dVector VehicleFrameRotation = dVector(drandRangef, 0.0f, 0.0f);
	//
	dMatrix location3(dGetIdentityMatrix());
	//
	if (VehicleFrameRotation.m_x != 0.0f)
	  location3 = location3 * dYawMatrix(VehicleFrameRotation.m_x * dDegreeToRad) * location;
    //
	if (VehicleFrameRotation.m_y != 0.0f)
	  location3 = location3 * dRollMatrix(VehicleFrameRotation.m_y * dDegreeToRad) * location;
	//
	if (VehicleFrameRotation.m_z != 0.0f)
	  location3 = location3 * dPitchMatrix(VehicleFrameRotation.m_z * dDegreeToRad) * location;
    //
location3 = dGetIdentityMatrix();

	location3.m_posit.m_y += 2.5f;
	location3.m_posit.m_x += 2.0f;
	//
	// Vehicle manager, It permit to create multiple vehicle instance. 
	VehicleControllerManagerDG* const manager = new VehicleControllerManagerDG(scene, world, 1, materialList);
	//
	// VEHICLE BEGIN
	// tire count, vehicle matrix
	VehicleControllerManagerDG::VehicleFrameEntity* const multibodyvehicle = new VehicleControllerManagerDG::VehicleFrameEntity(scene, manager, 4, location3);
	//
	multibodyvehicle->SetEngineFpsRequest(120.0f);
	//
	// Set the main vehicle controlled by keyboard.
	manager->GetInputManager()->SetVehicleEntity(multibodyvehicle);
	//
	// mass, scale
	/*NewtonBody* const VehicleFrameBody = */ 
	// D.G: Here with my box frame vehicle I set the center of mass to get a better simulation.
	// D.G: If you use mesh frame, normally the shape have a better balancing about the mass and you can use zero center of mass.
	// D.G: The best is to try with and without and see the diff.
	// D.G: Exemple the popular subaru model, when this mesh is saved with a good barycenter.
	// D.G: The mesh give a very nice result without need to use the center of mass.
	VehicleFrameCreate(scene, multibodyvehicle, 1200.0f, dVector(-0.15f, -0.65f, 0.0f), dVector(4.0f, 1.125f, 2.55f));
	//
	// Front tires
	/*NewtonBody* const Tire1 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, 1.125f), manager->GetTireMaterial());
	/*NewtonBody* const Tire2 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, -1.125f), manager->GetTireMaterial());
	// Rear Tires
	/*NewtonBody* const Tire3 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, 1.125f), manager->GetTireMaterial());
	/*NewtonBody* const Tire4 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, -1.125f), manager->GetTireMaterial());
	// Suspension spring force, spring damp, mass effect scaled, spring limit 
	multibodyvehicle->GetTireJoint(0)->SetTireSuspenssion(100.0f, 6.0f, -0.275f, 0.0f);
	multibodyvehicle->GetTireJoint(1)->SetTireSuspenssion(100.0f, 6.0f, -0.275f, 0.0f);
	multibodyvehicle->GetTireJoint(2)->SetTireSuspenssion(100.0f, 6.0f, -0.275f, 0.0f);
	multibodyvehicle->GetTireJoint(3)->SetTireSuspenssion(100.0f, 6.0f, -0.275f, 0.0f);
	// Setup for 4x4
	// Setup the tire id that can use the Torque
	multibodyvehicle->GetTireJoint(0)->SetUseTorque(true);
	multibodyvehicle->GetTireJoint(1)->SetUseTorque(true);
	multibodyvehicle->GetTireJoint(2)->SetUseTorque(true);
	multibodyvehicle->GetTireJoint(3)->SetUseTorque(true);
	// Setup the tire id that can use the steer
	multibodyvehicle->GetTireJoint(0)->SetUseSteer(true);
	multibodyvehicle->GetTireJoint(1)->SetUseSteer(true);
	multibodyvehicle->GetTireJoint(2)->SetUseSteer(false);
	multibodyvehicle->GetTireJoint(3)->SetUseSteer(false);
	// Setup the tire id that can use the break
	multibodyvehicle->GetTireJoint(0)->SetUseBreak(false);
	multibodyvehicle->GetTireJoint(1)->SetUseBreak(false);
	multibodyvehicle->GetTireJoint(2)->SetUseBreak(true);
	multibodyvehicle->GetTireJoint(3)->SetUseBreak(true);
	// VEHICLE END
	//
	// MULTIPLE VEHICLES BEGIN
	int count = 0;
	for (int u = 0; u < count; u++) {
		VehicleFrameRotation = dVector(-180.0f, 30.0f, 0.0f);
		dMatrix location4(dGetIdentityMatrix());

		if (VehicleFrameRotation.m_x != 0.0f)
			location4 = location4 * dYawMatrix(VehicleFrameRotation.m_x * dDegreeToRad) * location;
		//
		if (VehicleFrameRotation.m_y != 0.0f)
			location4 = location4 * dRollMatrix(VehicleFrameRotation.m_y * dDegreeToRad) * location;
		//
		if (VehicleFrameRotation.m_z != 0.0f)
			location4 = location4 * dPitchMatrix(VehicleFrameRotation.m_z * dDegreeToRad) * location;
		//
		location4.m_posit.m_x += 65.0f;
		location4.m_posit.m_y = 30.0f;
		//location4.m_posit.m_x = dSin(20.0f)*80-60;
	    location4.m_posit.m_z = -38.0f + u * 4.0f;
		//location4.m_posit.m_x += 2.0f;
		//
		// tire count, vehicle matrix
		VehicleControllerManagerDG::VehicleFrameEntity* const multibodyvehicles = new VehicleControllerManagerDG::VehicleFrameEntity(scene, manager, 4, location4);
		//
		multibodyvehicles->SetEngineFpsRequest(120.0f);
		// mass, scale
		/*NewtonBody* const VehicleFrameBody = */
		// D.G: Here with my box frame vehicle I set the center of mass to get a better simulation.
		// D.G: If you use mesh frame, normally the shape have a better balancing about the mass and you can use zero center of mass.
		// D.G: The best is to try with and without and see the diff.
		// D.G: Exemple the popular subaru model, when this mesh is saved with a good barycenter.
		// D.G: It give very nice result without need to use the center of mass.
		VehicleFrameCreate(scene, multibodyvehicles, 1200.0f, dVector(-0.15f, -0.65f, 0.0f), dVector(4.0f, 1.125f, 2.55f));
		//
		// Front tires
		/*NewtonBody* const Tire1 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, 1.125f), manager->GetTireMaterial());
		/*NewtonBody* const Tire2 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, -1.125f), manager->GetTireMaterial());
		// Rear Tires
		/*NewtonBody* const Tire3 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, 1.125f), manager->GetTireMaterial());
		/*NewtonBody* const Tire4 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, -1.125f), manager->GetTireMaterial());
		// Suspension spring force, spring damp, mass effect scaled, spring limit 
		multibodyvehicle->GetTireJoint(0)->SetTireSuspenssion(150.0f, 5.0f, -0.25f, 0.0f);
		multibodyvehicle->GetTireJoint(1)->SetTireSuspenssion(150.0f, 5.0f, -0.25f, 0.0f);
		multibodyvehicle->GetTireJoint(2)->SetTireSuspenssion(150.0f, 5.0f, -0.25f, 0.0f);
		multibodyvehicle->GetTireJoint(3)->SetTireSuspenssion(150.0f, 5.0f, -0.25f, 0.0f);
		// Setup the tire id that can use the Torque
		multibodyvehicles->GetTireJoint(0)->SetUseTorque(false);
		multibodyvehicles->GetTireJoint(1)->SetUseTorque(false);
		multibodyvehicles->GetTireJoint(2)->SetUseTorque(false);
		multibodyvehicles->GetTireJoint(3)->SetUseTorque(false);
		// Setup the tire id that can use the steer
		multibodyvehicles->GetTireJoint(0)->SetUseSteer(false);
		multibodyvehicles->GetTireJoint(1)->SetUseSteer(false);
		multibodyvehicles->GetTireJoint(2)->SetUseSteer(false);
		multibodyvehicles->GetTireJoint(3)->SetUseSteer(false);
		// Setup the tire id that can use the break
		multibodyvehicles->GetTireJoint(0)->SetUseBreak(false);
		multibodyvehicles->GetTireJoint(1)->SetUseBreak(false);
		multibodyvehicles->GetTireJoint(2)->SetUseBreak(false);
		multibodyvehicles->GetTireJoint(3)->SetUseBreak(false);
	}
	// MULTIPLE VEHICLES END

}

#else

class MultibodyVehicleControllerDG: public dCustomControllerBase
{
	public:
	void Init(NewtonBody* const body)
	{
		m_body = body;
		m_tireCount = 0;
	}

	protected:

	// old style suspension force use as effective to model sprung mass
	// this is a hack that we are getting away off.
	void ApplyTireSuspensionForcesOld (dFloat timestep)
	{
		dVector FrameTorque;
		dVector FrameForce;
		dVector AttVeloc;
		dVector frmVeloc;
		dVector frmOmega;
		dVector frmCom;
		dVector AttachForce;
		dMatrix frmMatrix;
		dFloat speed;
		dFloat frameMass;
		dFloat vIxx;
		dFloat vIyy;
		dFloat vIzz;
		dVector tireTorque;
		dVector chassisReationTorque;
		//

		NewtonBody* const chassis = GetBody();
		NewtonBodySetSleepState(chassis, 0);
		NewtonBodyGetMass(chassis, &frameMass, &vIxx, &vIyy, &vIzz);

		for (int i = 0; i < m_tireCount; i++) {
			dCustomTireSpringDG* cTireSpring = m_tireJoint[i];
			//
			frmCom = dVector(0.0f, 0.0f, 0.0f, 0.0f);
			frmVeloc = dVector(0.0f, 0.0f, 0.0f, 0.0f);
			FrameTorque = dVector(0.0f, 0.0f, 0.0f, 0.0f);
			FrameForce = dVector(0.0f, 0.0f, 0.0f, 0.0f);
			//
			cTireSpring->TireMatrixProjection();
			//
			NewtonBodySetSleepState(cTireSpring->GetBody1(), 0);
			NewtonBodyGetVelocity(cTireSpring->GetBody1(), &AttVeloc[0]);
			NewtonBodyGetVelocity(chassis, &frmVeloc[0]);
			NewtonBodyGetOmega(chassis, &frmOmega[0]);
			NewtonBodyGetMatrix(chassis, &frmMatrix[0][0]);
			NewtonBodyGetCentreOfMass(chassis, &frmCom[0]);
			//
			//dFloat mVehicleSpeed = dAbs(frmVeloc.DotProduct3(cTireSpring->GetChassisPivotMatrix().m_right));
			//
			frmCom = frmMatrix.TransformVector(frmCom); //OXTransformVector(frmCom, frmMatrix); 
			frmVeloc = frmVeloc + frmOmega.CrossProduct(cTireSpring->GetCenterInChassis() - frmCom);
			//
			speed = (frmVeloc - AttVeloc).DotProduct3(cTireSpring->GetChassisPivotMatrix().m_up);
			//
			cTireSpring->SetDistance();
			//
			dFloat sprungMass = 0.5f;
			cTireSpring->SetAccel(NewtonCalculateSpringDamperAcceleration(timestep, cTireSpring->GetSpringK(), cTireSpring->GetDistance(),
								  cTireSpring->GetSpringD(), speed) * frameMass * sprungMass);
			//
			FrameForce = (cTireSpring->GetChassisPivotMatrix().m_up * cTireSpring->GetAccel());
			FrameTorque = (cTireSpring->GetChassisPivotMatrix().m_posit - frmCom).CrossProduct(FrameForce);
			//
			NewtonBodyAddForce(chassis, &FrameForce[0]);
			NewtonBodyAddTorque(chassis, &FrameTorque[0]);
			//
			AttachForce = (FrameForce * VEHICLE_ATTACH_FORCE_SIDE);
			NewtonBodyAddForce(cTireSpring->GetBody1(), &AttachForce[0]);
			//
			// When only normal break is use it apply the torque anyway, Because with the 4x4 model the rear tire can break and the front tire can accelerate in same time.
			// The result is a bit strange the vehicle can slide a bit more when you break with 4x4 model and if you accelerate in same time.
			if (cTireSpring->GetUseTorque() && (!cTireSpring->GetUseHardBreak())) {
				// apply engine torque plus some tire angular drag
				tireTorque = (cTireSpring->GetChassisPivotMatrix().m_front * (cTireSpring->GetTireTorque() - cTireSpring->GetRealTireOmega() * cTireSpring->GetTireIzz()));
				NewtonBodyAddTorque(cTireSpring->GetBody1(), &tireTorque[0]);
			}
			//
			dFloat frameTorqueAffective = 0.25f;
			chassisReationTorque = (cTireSpring->GetChassisPivotMatrix().m_front * -(cTireSpring->GetTireTorque() * frameTorqueAffective));
			NewtonBodyAddTorque(chassis, &chassisReationTorque[0]);
		}
	}

	// new style suspension force calculate the sprung mass for any tire arrangement 
	void ApplyTireSuspensionForces (dFloat timestep)
	{
		dMatrix chassisMatrix;
		dMatrix chassisInvInertia;
		dVector chassisCom;
		dVector chassisOmega;
		dVector chassisVeloc;
		dFloat chassisInvMass;
		dFloat chassisInvIxx;
		dFloat chassisInvIyy;
		dFloat chassisInvIzz;

		const int maxSize = 64;
		dComplementaritySolver::dJacobianPair m_jt[maxSize];
		dComplementaritySolver::dJacobianPair m_jInvMass[maxSize];
		dFloat massMatrix[maxSize * maxSize];
		dFloat accel[maxSize];

		NewtonBody* const chassisBody = GetBody();
		NewtonBodyGetInvInertiaMatrix(chassisBody, &chassisInvInertia[0][0]);

		NewtonBodyGetOmega(chassisBody, &chassisOmega[0]);
		NewtonBodyGetVelocity(chassisBody, &chassisVeloc[0]);
		NewtonBodyGetInvMass(chassisBody, &chassisInvMass, &chassisInvIxx, &chassisInvIyy, &chassisInvIzz);

		NewtonBodyGetMatrix(chassisBody, &chassisMatrix[0][0]);
		NewtonBodyGetCentreOfMass(chassisBody, &chassisCom[0]);

		dVector chassisOrigin(chassisMatrix.TransformVector(chassisCom));
		for (int i = 0; i < m_tireCount; i++) {
			dMatrix tireMatrix;
			dMatrix chassisMatrix;
			dMatrix tireInvInertia;
			dVector tireOmega;
			dVector tireVeloc;

			dFloat tireInvMass;
			dFloat tireInvIxx;
			dFloat tireInvIyy;
			dFloat tireInvIzz;

			dCustomTireSpringDG* const tire = m_tireJoint[i];
			NewtonBody* const tireBody = tire->GetBody1();
			tire->CalculateGlobalMatrix(chassisMatrix, tireMatrix);

			NewtonBodyGetInvInertiaMatrix(tireBody, &tireInvInertia[0][0]);
			NewtonBodyGetInvMass(tireBody, &tireInvMass, &tireInvIxx, &tireInvIyy, &tireInvIzz);

			m_jt[i].m_jacobian_J01.m_linear = chassisMatrix.m_up.Scale(-1.0f);
			m_jt[i].m_jacobian_J01.m_angular = dVector(0.0f);
			m_jt[i].m_jacobian_J10.m_linear = chassisMatrix.m_up;
			m_jt[i].m_jacobian_J10.m_angular = (tireMatrix.m_posit - chassisOrigin).CrossProduct(chassisMatrix.m_up);

			m_jInvMass[i].m_jacobian_J01.m_linear = m_jt[i].m_jacobian_J01.m_linear.Scale(tireInvMass);
			m_jInvMass[i].m_jacobian_J01.m_angular = tireInvInertia.RotateVector(m_jt[i].m_jacobian_J01.m_angular);
			m_jInvMass[i].m_jacobian_J10.m_linear = m_jt[i].m_jacobian_J10.m_linear.Scale(chassisInvMass);
			m_jInvMass[i].m_jacobian_J10.m_angular = chassisInvInertia.RotateVector(m_jt[i].m_jacobian_J10.m_angular);

			NewtonBodyGetOmega(tireBody, &tireOmega[0]);
			NewtonBodyGetVelocity(tireBody, &tireVeloc[0]);

			const dVector v(m_jt[i].m_jacobian_J01.m_linear * tireVeloc + m_jt[i].m_jacobian_J01.m_angular * tireOmega +
							m_jt[i].m_jacobian_J10.m_linear * chassisVeloc + m_jt[i].m_jacobian_J10.m_angular * chassisOmega);
			const dVector s(m_jt[i].m_jacobian_J01.m_linear * tireMatrix.m_posit + m_jt[i].m_jacobian_J10.m_linear * chassisMatrix.m_posit);
			const dFloat dist = -(s.m_x + s.m_y + s.m_z);
			const dFloat speed = -(v.m_x + v.m_y + v.m_z);

			const dFloat kv = tire->GetSpringD();
			const dFloat ks = tire->GetSpringK();
			accel[i] = -NewtonCalculateSpringDamperAcceleration(timestep, ks, dist, kv, speed);
		}

		for (int i = 0; i < m_tireCount; i++) {
			dFloat* const row = &massMatrix[i * m_tireCount];

			dFloat aii = m_jInvMass[i].m_jacobian_J01.m_linear.DotProduct3(m_jt[i].m_jacobian_J01.m_linear) + m_jInvMass[i].m_jacobian_J01.m_angular.DotProduct3(m_jt[i].m_jacobian_J01.m_angular) +
				m_jInvMass[i].m_jacobian_J10.m_linear.DotProduct3(m_jt[i].m_jacobian_J10.m_linear) + m_jInvMass[i].m_jacobian_J10.m_angular.DotProduct3(m_jt[i].m_jacobian_J10.m_angular);

			row[i] = aii * 1.0001f;
			for (int j = i + 1; j < m_tireCount; j++) {
				dFloat aij = m_jInvMass[i].m_jacobian_J10.m_linear.DotProduct3(m_jt[j].m_jacobian_J10.m_linear) + m_jInvMass[i].m_jacobian_J10.m_angular.DotProduct3(m_jt[j].m_jacobian_J10.m_angular);
				row[j] = aij;
				massMatrix[j * m_tireCount + i] = aij;
			}
		}

		dCholeskyFactorization(m_tireCount, massMatrix);
		dCholeskySolve(m_tireCount, m_tireCount, massMatrix, accel);

		dVector chassisForce(0.0f);
		dVector chassisTorque(0.0f);
		for (int i = 0; i < m_tireCount; i++) {
			dCustomTireSpringDG* const tire = m_tireJoint[i];
			NewtonBody* const tireBody = tire->GetBody1();

			dVector force(m_jt[i].m_jacobian_J01.m_linear.Scale(accel[i]));
			NewtonBodyAddForce(tireBody, &force[0]);

			chassisForce += m_jt[i].m_jacobian_J10.m_linear.Scale(accel[i]);
			chassisTorque += m_jt[i].m_jacobian_J10.m_angular.Scale(accel[i]);
		}
		NewtonBodyAddForce(chassisBody, &chassisForce[0]);
		NewtonBodyAddTorque(chassisBody, &chassisTorque[0]);
	}


	virtual void PreUpdate(dFloat timestep, int threadIndex)
	{
		//ApplyTireSuspensionForcesOld (timestep);
		ApplyTireSuspensionForces (timestep);
	}

	virtual void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}
	
	int m_tireCount;
	dCustomTireSpringDG* m_tireJoint[4];
	friend class MultibodyVehicleControllerManagerDG;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class MultibodyVehicleControllerManagerDG: public dCustomControllerManager<MultibodyVehicleControllerDG>
{
	public:
	MultibodyVehicleControllerManagerDG(NewtonWorld* const world)
		:dCustomControllerManager<MultibodyVehicleControllerDG>(world, "Multi body Vehicle Manage DG")
	{
		// setting up a user contact handle to calculate tire collision with terrain
		m_tireMaterial = NewtonMaterialCreateGroupID(world);
		//int defualtMaterial = NewtonMaterialGetDefaultGroupID(world);

		//NewtonMaterialSetCallbackUserData(world, m_tireMaterial, defualtMaterial, this);
		//NewtonMaterialSetContactGenerationCallback(world, m_tireMaterial, defualtMaterial, OnTireContactGeneration);
		//NewtonMaterialSetCollisionCallback(world, m_tireMaterial, defualtMaterial, UserOnAABBOverlap, UserContactFriction);
	}
		
	virtual ~MultibodyVehicleControllerManagerDG()
	{
	}

	NewtonBody* CreateChassis(const dMatrix&matrix, dFloat const vMass, dVector const vCenterMass, dVector const vScal)
	{
		// get the physical world and visual scene
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		
		// create a simple box to serve as chassis
		int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
		NewtonCollision* const collision = NewtonCreateBox(world, vScal.m_x, vScal.m_y, vScal.m_z, 0, NULL);
		DemoMesh* const VehicleFrameMesh1 = new DemoMesh("VehicleFrameMesh1", collision, "wood_0.tga", "smilli.tga", "wood_0.tga");
		NewtonBody* const vBody = CreateSimpleSolid(scene, VehicleFrameMesh1, vMass, matrix, collision, defaultMaterialID);
		VehicleFrameMesh1->Release();
		NewtonDestroyCollision(collision);

		// set some vehicle properties, for better physics vehavior 
		dVector angdamp(0.0f);

		// Make the tire 100% free rolling(spinning).
		NewtonBodySetLinearDamping(vBody, 0.0f);
		NewtonBodySetAngularDamping(vBody, &angdamp[0]);
		//
		NewtonBodySetAutoSleep(vBody, 0);
		NewtonBodySetFreezeState(vBody, 0);
		NewtonBodySetMatrix(vBody, &matrix[0][0]);

		//vVehicle->SetVehicleFrameMesh(VehicleFrameMesh1);
		//vVehicle->AddVehicleFrameBody(vBody, vCenterMass);
		return vBody;
	}

	NewtonBody* CreateTire(MultibodyVehicleControllerDG* const controller, dFloat const tireMass, dFloat tireRad, dFloat tireHeight, dVector tirePos)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		NewtonBody* const chassisBody = controller->GetBody();

		dMatrix matrixVehicle;
		NewtonBodyGetMatrix(chassisBody, &matrixVehicle[0][0]);

		dMatrix matrix(dYawMatrix(dPi * 0.5f));

		//dVector tireRot(0.0f);
		matrix.m_posit = tirePos;
		matrix.m_posit.m_w = 1.0f;
		matrix = matrix * matrixVehicle;

		// lesson two use a unit CreateChamferCylinder and scale it instead of a variable size one.
		//NewtonCollision* const collision = NewtonCreateChamferCylinder(world, tireRad, tireHeight, 0, NULL);
		NewtonCollision* const collision = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
		NewtonCollisionSetScale(collision, 2.0f * tireHeight, 2.0f * tireRad, 2.0f * tireRad);

		DemoMesh* const VehicleTireMesh = new DemoMesh("tireShape", collision, "smilli.tga", "smilli.tga", "smilli.tga");
		NewtonBody* const tireBody = CreateSimpleSolid(scene, VehicleTireMesh, tireMass, matrix, collision, m_tireMaterial);
		VehicleTireMesh->Release();
		NewtonDestroyCollision(collision);

		// Make the tire 100% free rolling(spinning).
		dVector angdamp(0.0f);
		NewtonBodySetLinearDamping(tireBody, 0.0f);
		NewtonBodySetAngularDamping(tireBody, &angdamp[0]);
		//
		//NewtonBodySetAutoSleep(tireBody, 0);
		//NewtonBodySetFreezeState(tireBody, 0);
		//NewtonBodySetMatrix(tireBody, &matrix[0][0]);
		//
		dCustomTireSpringDG* const tireJoint = new dCustomTireSpringDG(matrix, chassisBody, tireBody);
		tireJoint->SetSolverModel(2);
		//tireJoint->SetTireSuspenssion(100.0f, 6.0f, 0.5f, -0.275f, 0.0f);
		tireJoint->SetTireSuspenssion(1200.0f, 30.0f, -0.275f, 0.0f);
		controller->m_tireJoint[controller->m_tireCount] = tireJoint;
		controller->m_tireCount++;

		// set the tire material, you can set other stuff her that you can use in the call back
		//NewtonCollisionMaterial collisionMaterial;
		//NewtonCollisionGetMaterial(NewtonBodyGetCollision(tireBody), &collisionMaterial);
		//collisionMaterial.m_userId = D_MULTIBODY_TIRE_ID;
		//collisionMaterial.m_userData = tireJoint;
		//NewtonCollisionSetMaterial(NewtonBodyGetCollision(tireBody), &collisionMaterial);
		//NewtonBodySetMaterialGroupID(tireBody, tireMaterial);

		//
		return tireBody;
	}

	MultibodyVehicleControllerDG* CreateBasicVehicle(const dMatrix& location)
	{
		NewtonBody* const chassis = CreateChassis(location, 1200.0f, dVector(-0.15f, -0.65f, 0.0f), dVector(4.0f, 1.125f, 2.55f));

		MultibodyVehicleControllerDG* const controller = CreateController();
		//cVcontroller = controller;
		controller->Init(chassis);

		// front tires
		CreateTire(controller, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, 1.125f));
		CreateTire(controller, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, -1.125f));

		// Rear Tires
		CreateTire(controller, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, 1.125f));
		CreateTire(controller, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, -1.125f));

		return controller;
	}
	
	int m_tireMaterial;
	friend class dCustomVehicleControllerDG;
};


void BasicMultibodyVehicle(DemoEntityManager* const scene)
{
	//////////////////////////////////////////////////////
	// Scene 3D load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh(scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain(scene, 10, 8.0f, 5.0f, 0.2f, 200.0f, -50.0f);

	NewtonWorld* const world = scene->GetNewton();

	// encapsulate vehicle controller so that we can apply customizations in the pre and post update function 
	MultibodyVehicleControllerManagerDG* const manager = new MultibodyVehicleControllerManagerDG(world);

	// VEHICLE BEGIN
	// create a basic vehicle matrix
	dMatrix location(dGetIdentityMatrix());
	location.m_posit = dVector(FindFloor(world, dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	location.m_posit.m_y += 2.0f;
	MultibodyVehicleControllerDG* const multibodyvehicle = manager->CreateBasicVehicle (location);

	// set the camera 
	location.m_posit = dVector(FindFloor(scene->GetNewton(), dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	dVector origin(FindFloor(world, dVector(-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	origin.m_y += 10.0f;
	origin.m_x += -45.0f;
	//
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);

}


#endif