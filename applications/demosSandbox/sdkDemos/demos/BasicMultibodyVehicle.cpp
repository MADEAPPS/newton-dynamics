/*
* Vehicle Multibody Joint + Demo write by Dave Gravel 2018.
* I have write this vehicle multibody code for share with other newton users, and maybe it can help someone.
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
		void AddTireSuspenssion(NewtonBody* const vTire, dMatrix vTireMatrix)
		{
			mTireJoint[mTireCount] = new dCustomTireSpringDG(vTireMatrix, mFrameBody, vTire);
			mTireJoint[mTireCount]->SetSolverModel(2);
			mTireJoint[mTireCount]->SetEngineFpsRequest(GetEngineFpsRequest());
			//
			mTireCount++;
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
				cTireSpring->SetAccel(NewtonCalculateSpringDamperAcceleration(timestep, cTireSpring->GetSpringK(), cTireSpring->GetDistance(), 
				cTireSpring->GetSpringD(), speed) * frameMass * cTireSpring->GetSpringMassEffective());
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

private:
	NewtonWorld* nworld;
	dCustomVehicleControllerDG* cVcontroller;
	// add an input Manage to manage the inputs and user interaction 
	VehicleInputManager* minputManager;
	DemoEntityManager* mscene;
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

NewtonBody* VehicleTireCreate(DemoEntityManager* scene, VehicleControllerManagerDG::VehicleFrameEntity* const vVehicle, dFloat const tireMass, dFloat tireRad, dFloat tireHeight, dVector tirePos)
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
	int defaultMaterialID;
	defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetCollisionCallback(world, defaultMaterialID, defaultMaterialID, UserOnAABBOverlap, UserContactFriction);
	// some extra char in case the count go higher.
	char buff[24] = "";
	sprintf(buff, "VehicleTireMesh%d", vVehicle->GetTireCount());
	NewtonCollision* collision = NewtonCreateChamferCylinder(world, tireRad, tireHeight, 0, NULL);
	DemoMesh* const VehicleTireMesh = new DemoMesh(&buff[0], collision, "smilli.tga", "smilli.tga", "smilli.tga");
	NewtonBody* const vBody = CreateSimpleSolid(scene, VehicleTireMesh, tireMass, matrix, collision, defaultMaterialID);
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
	vVehicle->AddTireSuspenssion(vBody, matrix);
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
	DemoMesh* const StartBox = new DemoMesh("StartBox", collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
	CreateSimpleSolid(scene, StartBox, 0.0f, startlocation, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//
	//startlocation2 = startlocation2 * dRollMatrix(2.5f * dDegreeToRad);
	startlocation2.m_posit.m_x += 38.0f;
	startlocation2.m_posit.m_y += 4.0f;
	collision = NewtonCreateBox(world, 20.0f, 0.25f, 40.0f, 0, NULL);
	DemoMesh* const StartBox3 = new DemoMesh("StartBox", collision, "smilli.tga", "smilli.tga", "smilli.tga");
	CreateSimpleSolid(scene, StartBox3, 0.0f, startlocation2, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//	
	startlocation3 = startlocation3 * dRollMatrix(32.5f * dDegreeToRad);
	startlocation3.m_posit.m_x -= 31.5f;
	startlocation3.m_posit.m_y += 0.575f;
	collision = NewtonCreateBox(world, 2.0f, 2.0f, 100.0f, 0, NULL);
	DemoMesh* const StartBox2 = new DemoMesh("StartBox", collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
	CreateSimpleSolid(scene, StartBox2, 0.0f, startlocation3, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	collision = NULL;
	//
	BuildPyramid(scene, 10.0f, dVector(-20.0f, 0.0f, 10.0f, 1.0f), dVector(0.5f, 0.25f, 0.8f, 0.0), 10, _BOX_PRIMITIVE);
	BuildPyramid(scene, 10.0f, dVector(-20.0f, 0.0f, -10.0f, 1.0f), dVector(0.5f, 0.25f, 0.8f, 0.0), 10, _BOX_PRIMITIVE);
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
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, 1.125f));
	/*NewtonBody* const Tire2 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, -1.125f));
	// Rear Tires
	/*NewtonBody* const Tire3 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, 1.125f));
	/*NewtonBody* const Tire4 = */ 
	VehicleTireCreate(scene, multibodyvehicle, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, -1.125f));
	// Suspenssion spring force, spring damp, mass effect scaled, spring limite 
	multibodyvehicle->GetTireJoint(0)->SetTireSuspenssion(100.0f, 6.0f, 0.5f, -0.275f, 0.0);
	multibodyvehicle->GetTireJoint(1)->SetTireSuspenssion(100.0f, 6.0f, 0.5f, -0.275f, 0.0);
	multibodyvehicle->GetTireJoint(2)->SetTireSuspenssion(100.0f, 6.0f, 0.5f, -0.275f, 0.0);
	multibodyvehicle->GetTireJoint(3)->SetTireSuspenssion(100.0f, 6.0f, 0.5f, -0.275f, 0.0);
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
	for (int u = 0; u < 20; u++) {
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
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, 1.125f));
		/*NewtonBody* const Tire2 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(1.0f, -0.75f, -1.125f));
		// Rear Tires
		/*NewtonBody* const Tire3 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, 1.125f));
		/*NewtonBody* const Tire4 = */
		VehicleTireCreate(scene, multibodyvehicles, 75.0f, 0.35f, 0.4f, dVector(-1.25f, -0.75f, -1.125f));
		// Suspenssion spring force, spring damp, mass effect scaled, spring limite 
		multibodyvehicle->GetTireJoint(0)->SetTireSuspenssion(150.0f, 5.0f, 0.5f, -0.25f, 0.0);
		multibodyvehicle->GetTireJoint(1)->SetTireSuspenssion(150.0f, 5.0f, 0.5f, -0.25f, 0.0);
		multibodyvehicle->GetTireJoint(2)->SetTireSuspenssion(150.0f, 5.0f, 0.5f, -0.25f, 0.0);
		multibodyvehicle->GetTireJoint(3)->SetTireSuspenssion(150.0f, 5.0f, 0.5f, -0.25f, 0.0);
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