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

#define TMODE_NO_STEER_NO_TORQUE  0
#define TMODE_TORQUE 1
#define TMODE_TORQUE_BREAK 2
#define TMODE_TORQUE_BREAK_HARDBREAK 3
#define TMODE_STEER 4
#define TMODE_STEER_BREAK 5
#define TMODE_STEER_BREAK_HARDBREAK 6
#define TMODE_TORQUE_STEER 7
#define TMODE_TORQUE_STEER_BREAK 8
#define TMODE_TORQUE_STEER_BREAK_HARDBREAK 9


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
		NewtonMaterialSetContactFrictionCoef(material, 1.1f, 1.1f, 0);
		NewtonMaterialSetContactFrictionCoef(material, 1.1f, 1.1f, 1);
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

static dFloat dRandRangeFloat(dFloat amin, dFloat amax)
{
	dFloat r = (dFloat)dRand() / (dFloat)RAND_MAX;
	return (amin + r * (amax - amin));
}


class MultibodyVehicleControllerDG;
static bool IsPlayer(const MultibodyVehicleControllerDG* const controller);

class MultibodyVehicleControllerDG: public dCustomControllerBase
{
	public:
	void Init(NewtonBody* const body)
	{
		m_body = body;
		m_tireCount = 0;
		mEngineFpsRequest = 120.0f;
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

			tire->TireMatrixProjection();

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

	dFloat GetEngineFpsRequest()
	{
		return mEngineFpsRequest;
	}

	void OnBeginUpdate(dFloat timestepInSecunds)
	{
		NewtonWorld* const world = GetManager()->GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		int Accelerator = int(scene->GetKeyState(265)) - int(scene->GetKeyState(264));
		int SteerAction = int(scene->GetKeyState(263)) - int(scene->GetKeyState(262));
		int BreakAction = int(scene->GetKeyState('B')) - int(scene->GetKeyState(32));

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
		// The best is to create a engine value with graduation for the torque.
		// On this way with gears you can get the perfect torque to give without make the tire spin to much.
		if (Accelerator < 0) {
			m_tireJoint[0]->SetTireTorque(-1200.0f * timestepInSecunds * GetEngineFpsRequest());
			m_tireJoint[1]->SetTireTorque(1200.0f * timestepInSecunds * GetEngineFpsRequest());
			m_tireJoint[2]->SetTireTorque(-1200.0f * timestepInSecunds * GetEngineFpsRequest());
			m_tireJoint[3]->SetTireTorque(1200.0f * timestepInSecunds * GetEngineFpsRequest());
		} else if (Accelerator > 0) {
			m_tireJoint[0]->SetTireTorque(1500.0f * timestepInSecunds * GetEngineFpsRequest());
			m_tireJoint[1]->SetTireTorque(-1500.0f * timestepInSecunds * GetEngineFpsRequest());
			m_tireJoint[2]->SetTireTorque(1500.0f * timestepInSecunds * GetEngineFpsRequest());
			m_tireJoint[3]->SetTireTorque(-1500.0f * timestepInSecunds * GetEngineFpsRequest());
		} else {
			//m_tireJoint[0]->SetTireTorque(0.0f);
			//m_tireJoint[1]->SetTireTorque(0.0f);
			m_tireJoint[2]->SetTireTorque(0.0f);
			m_tireJoint[3]->SetTireTorque(0.0f);
		}
		// It is pretty same here you can general a value from zero to the desired angle with graduation.
		// On this way you can have more control on the steer and it can acting better and smoother.
		if (SteerAction < 0) {
			m_tireJoint[0]->SetTireSteer(-35.0f);
			m_tireJoint[1]->SetTireSteer(-35.0f);
			// m_vehicle->GetTireJoint(2)->SetTireSteer(-35.0f);
			// m_vehicle->GetTireJoint(3)->SetTireSteer(-35.0f);
		}
		else
			if (SteerAction > 0) {
				m_tireJoint[0]->SetTireSteer(35.0f);
				m_tireJoint[1]->SetTireSteer(35.0f);
				// m_vehicle->GetTireJoint(2)->SetTireSteer(35.0f);
				// m_vehicle->GetTireJoint(3)->SetTireSteer(35.0f);
			}
			else {
				m_tireJoint[0]->SetTireSteer(0.0f);
				m_tireJoint[1]->SetTireSteer(0.0f);
				// m_vehicle->GetTireJoint(2)->SetTireSteer(0.0f);
				// m_vehicle->GetTireJoint(3)->SetTireSteer(0.0f);
			}
		//
		if (BreakAction >= 1) {
			m_tireJoint[2]->SetTireBreak(5000.0f * 2.0f);
			m_tireJoint[3]->SetTireBreak(5000.0f * 2.0f);
		}
		else
		if (BreakAction <= -1) {
			//m_tireJoint[0]->SetUseHardBreak(true);
			m_tireJoint[0]->SetTireBreak(5000.0f * 0.75f);
			//m_tireJoint[1]->SetUseHardBreak(true);
			m_tireJoint[1]->SetTireBreak(5000.0f * 0.75f);
			//m_tireJoint[2]->SetUseHardBreak(true);
			m_tireJoint[2]->SetTireBreak(5000.0f * 0.75f);
			//m_tireJoint[3]->SetUseHardBreak(true);
			m_tireJoint[3]->SetTireBreak(5000.0f * 0.75f);
		} else
		if (BreakAction == 0) {
		  m_tireJoint[0]->SetTireBreak(0.0f);
		  m_tireJoint[1]->SetTireBreak(0.0f);
		  m_tireJoint[2]->SetTireBreak(0.0f);
		  m_tireJoint[3]->SetTireBreak(0.0f);
		}

#if 0
			// It is pretty same here you can general a value from zero to the desired angle with graduation.
			// On this way you can have more control on the steer and it can acting better and smoother.
			if (SteerAction < 0) {
				m_vehicle->GetTireJoint(0)->SetTireSteer(-35.0f);
				m_vehicle->GetTireJoint(1)->SetTireSteer(-35.0f);
				// m_vehicle->GetTireJoint(2)->SetTireSteer(-35.0f);
				// m_vehicle->GetTireJoint(3)->SetTireSteer(-35.0f);
			} else
				if (SteerAction > 0) {
					m_vehicle->GetTireJoint(0)->SetTireSteer(35.0f);
					m_vehicle->GetTireJoint(1)->SetTireSteer(35.0f);
					// m_vehicle->GetTireJoint(2)->SetTireSteer(35.0f);
					// m_vehicle->GetTireJoint(3)->SetTireSteer(35.0f);
				} else {
					m_vehicle->GetTireJoint(0)->SetTireSteer(0.0f);
					m_vehicle->GetTireJoint(1)->SetTireSteer(0.0f);
					// m_vehicle->GetTireJoint(2)->SetTireSteer(0.0f);
					// m_vehicle->GetTireJoint(3)->SetTireSteer(0.0f);
				}
			//
			if (BreakAction > 0) {
				m_vehicle->GetTireJoint(2)->SetTireBreak(5000.0f * 2.0f);
				m_vehicle->GetTireJoint(3)->SetTireBreak(5000.0f * 2.0f);
			} else
				if (BreakAction < 0) {
					m_vehicle->GetTireJoint(0)->SetUseHardBreak(true);
					m_vehicle->GetTireJoint(0)->SetTireBreak(5000.0f * 0.5f);
					m_vehicle->GetTireJoint(1)->SetUseHardBreak(true);
					m_vehicle->GetTireJoint(1)->SetTireBreak(5000.0f * 0.5f);
					m_vehicle->GetTireJoint(2)->SetUseHardBreak(true);
					m_vehicle->GetTireJoint(2)->SetTireBreak(5000.0f * 0.5f);
					m_vehicle->GetTireJoint(3)->SetUseHardBreak(true);
					m_vehicle->GetTireJoint(3)->SetTireBreak(5000.0f * 0.5f);
				} else {
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
		
#endif
	}

	virtual void PreUpdate(dFloat timestep, int threadIndex)
	{
		if (IsPlayer(this)) {
			OnBeginUpdate(timestep);
		}

		//ApplyTireSuspensionForcesOld (timestep);
		ApplyTireSuspensionForces (timestep);

		for (int i = 0; i < 4; i++) {
			dCustomTireSpringDG* const cTireSpring = m_tireJoint[i];
			if (cTireSpring->GetUseTorque()) {
				dVector tireTorque(cTireSpring->GetChassisPivotMatrix().m_front.Scale(cTireSpring->GetTireTorque() - cTireSpring->GetRealTireOmega() * cTireSpring->GetTireIzz()));
				NewtonBodyAddTorque(cTireSpring->GetBody1(), &tireTorque[0]);
			}
		}
	}

	virtual void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}
	
	int m_tireCount;
	dFloat mEngineFpsRequest;
	dCustomTireSpringDG* m_tireJoint[4];
	friend class MultibodyVehicleControllerManagerDG;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class MultibodyVehicleControllerManagerDG: public dCustomControllerManager<MultibodyVehicleControllerDG>
{
	public:
	MultibodyVehicleControllerManagerDG(NewtonWorld* const world)
		:dCustomControllerManager<MultibodyVehicleControllerDG>(world, "Multi body Vehicle Manage DG")
		,m_currentController(NULL)
		,m_tireMaterial(0)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// plug a callback for 2d help display
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);
		//scene->SetUpdateCameraFunction(UpdateCameraCallback, this);

		// setting up a user contact handle to calculate tire collision with terrain
		m_tireMaterial = NewtonMaterialCreateGroupID(world);
		int defualtMaterial = NewtonMaterialGetDefaultGroupID(world);

		NewtonMaterialSetCallbackUserData(world, m_tireMaterial, defualtMaterial, this);
		NewtonMaterialSetContactGenerationCallback(world, m_tireMaterial, defualtMaterial, OnTireContactGeneration);
		NewtonMaterialSetCollisionCallback(world, m_tireMaterial, defualtMaterial, UserOnAABBOverlap, UserContactFriction);
	}
		
	virtual ~MultibodyVehicleControllerManagerDG()
	{
	}

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

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		MultibodyVehicleControllerManagerDG* const me = (MultibodyVehicleControllerManagerDG*)context;
		me->RenderPlayerHelp(scene);
	}

	NewtonBody* CreateChassis(const dMatrix&matrix, dFloat const vMass, dVector const vCenterMass, dVector const vScal)
	{
		// get the physical world and visual scene
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
		
		// create a simple box to serve as chassis
		int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
		NewtonCollision* const collision = NewtonCreateBox(world, vScal.m_x, vScal.m_y, vScal.m_z, 0, NULL);
		DemoMesh* const VehicleFrameMesh1 = new DemoMesh("VehicleFrameMesh1", scene->GetShaderCache(), collision, "wood_0.tga", "smilli.tga", "wood_0.tga");
		NewtonBody* const vBody = CreateSimpleSolid(scene, VehicleFrameMesh1, vMass, matrix, collision, defaultMaterialID);
		VehicleFrameMesh1->Release();
		NewtonDestroyCollision(collision);

		// set some vehicle properties, for better physics behavior 
		dVector angdamp(0.0f);

		// Make the tire 100% free rolling(spinning).
		NewtonBodySetLinearDamping(vBody, 0.0f);
		NewtonBodySetAngularDamping(vBody, &angdamp[0]);

		// set the vehicle matrix and cent of mass
		NewtonBodySetCentreOfMass(vBody, &vCenterMass[0]);
		NewtonBodySetMatrix(vBody, &matrix[0][0]);

		return vBody;
	}

	NewtonBody* CreateTire(MultibodyVehicleControllerDG* const controller, int TireMode, dFloat const tireMass, dFloat tireRad, dFloat tireHeight, dVector tirePos)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		NewtonBody* const chassisBody = controller->GetBody();

		dMatrix matrixVehicle;
		NewtonBodyGetMatrix(chassisBody, &matrixVehicle[0][0]);

		dMatrix matrix(dYawMatrix(dPi * 0.5f));

		dVector tireRot(0.0f);
		//matrix.m_posit = tirePos;
		//matrix.m_posit.m_w = 1.0f;

		//
		// rotate the model for when you use 3d mesh, to make if facing on the good side.
		//

		if (((controller->m_tireCount) % 2) == 0)
		{
			tireRot = dVector(90.0f, 0.0f, 0.0f);
		} else {
			tireRot = dVector(-90.0f, 0.0f, 0.0f);
		}

		// rotate the tire to make sure it is in the good side for the torque and visual.
		// normally 3d model have the tiremag facing in one side only, you need to rotate the mesh.
		// the same is apply here for the torque direction.
		matrix = dYawMatrix(tireRot.m_x * dDegreeToRad);

		// in the demo I use this 3 rotation, because normally in my real demo I can rotate the vehicle in any direction at the creation.
		// many user have problem with this, I talk about create a vehicle in any direction.
		//
		//if (tireRot.m_x != 0.0f) matrix = dYawMatrix(tireRot.m_x * dDegreeToRad);
		//if (tireRot.m_y != 0.0f) matrix = dRollMatrix(tireRot.m_y * dDegreeToRad);
		//if (tireRot.m_z != 0.0f) matrix = dPitchMatrix(tireRot.m_z * dDegreeToRad);

		matrix = matrix * matrixVehicle;

		matrix.m_posit = matrixVehicle.TransformVector(tirePos);

		// lesson two use a unit CreateChamferCylinder and scale it instead of a variable size one.
		//NewtonCollision* const collision = NewtonCreateChamferCylinder(world, tireRad, tireHeight, 0, NULL);
		NewtonCollision* const collision = NewtonCreateChamferCylinder(world, 0.5f, 1.0f, 0, NULL);
		NewtonCollisionSetScale(collision, 2.0f * tireHeight, 2.0f * tireRad, 2.0f * tireRad);

		DemoMesh* const VehicleTireMesh = new DemoMesh("tireShape", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");
		NewtonBody* const tireBody = CreateSimpleSolid(scene, VehicleTireMesh, tireMass, matrix, collision, m_tireMaterial);
		VehicleTireMesh->Release();
		NewtonDestroyCollision(collision);

		// lesson three: make tire inertia spherical to avoid bad gyro torques
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		NewtonBodyGetMass(tireBody, &mass, &Ixx, &Iyy, &Izz);
		Ixx = dMax(dMax(Ixx, Iyy), Izz);
		NewtonBodySetMassMatrix(tireBody, mass, Ixx, Ixx, Ixx);

		// Make the tire 100% free rolling(spinning).
		dVector angdamp(0.0f);
		NewtonBodySetLinearDamping(tireBody, 0.0f);
		NewtonBodySetAngularDamping(tireBody, &angdamp[0]);

		// make the tire joint
		dCustomTireSpringDG* const tireJoint = new dCustomTireSpringDG(matrix, chassisBody, tireBody);
		tireJoint->SetSolverModel(2);
		//tireJoint->SetTireSuspenssion(100.0f, 6.0f, 0.5f, -0.275f, 0.0f);
		tireJoint->SetTireSuspenssion(1300.0f, 40.0f, -0.275f, 0.0f);
		controller->m_tireJoint[controller->m_tireCount] = tireJoint;
	
		tireJoint->SetUseTorque(false);
		tireJoint->SetUseBreak(false);
		tireJoint->SetUseHardBreak(false);
		tireJoint->SetUseSteer(false);

		switch(TireMode) {
		  case TMODE_NO_STEER_NO_TORQUE: {
		    break;
		  }
		  case TMODE_TORQUE: {
		    tireJoint->SetUseTorque(true);
		    break;
		  }
	  	  case TMODE_TORQUE_BREAK: {
			tireJoint->SetUseTorque(true);
			tireJoint->SetUseBreak(true);
		    break;
		  }
		  case TMODE_TORQUE_BREAK_HARDBREAK: {
		    tireJoint->SetUseTorque(true);
			tireJoint->SetUseBreak(true);
			tireJoint->SetUseHardBreak(true);
		    break;
		  }
		  case TMODE_STEER: {
			  tireJoint->SetUseSteer(true);
			  break;
		  }
		  case TMODE_STEER_BREAK: {
			  tireJoint->SetUseBreak(true);
			  tireJoint->SetUseSteer(true);
			  break;
		  }
		  case TMODE_STEER_BREAK_HARDBREAK: {
			  tireJoint->SetUseSteer(true);
			  tireJoint->SetUseBreak(true);
			  tireJoint->SetUseHardBreak(true);
			  break;
		  }
		  case TMODE_TORQUE_STEER: {
			  tireJoint->SetUseTorque(true);
			  tireJoint->SetUseSteer(true);
			  break;
		  }
		  case TMODE_TORQUE_STEER_BREAK: {
			  tireJoint->SetUseTorque(true);
			  tireJoint->SetUseSteer(true);
			  tireJoint->SetUseBreak(true);
			  break;
		  }
		  case TMODE_TORQUE_STEER_BREAK_HARDBREAK: {
			  tireJoint->SetUseTorque(true);
			  tireJoint->SetUseSteer(true);
			  tireJoint->SetUseBreak(true);
			  tireJoint->SetUseHardBreak(true);
			  break;
		  }
		}

		//

		controller->m_tireCount++;

		// set the tire material, you can set other stuff her that you can use in the call back
		NewtonCollisionMaterial collisionMaterial;
		NewtonCollisionGetMaterial(NewtonBodyGetCollision(tireBody), &collisionMaterial);
		collisionMaterial.m_userId = D_MULTIBODY_TIRE_ID;
		collisionMaterial.m_userData = tireJoint;
		NewtonCollisionSetMaterial(NewtonBodyGetCollision(tireBody), &collisionMaterial);
		NewtonBodySetMaterialGroupID(tireBody, m_tireMaterial);

		return tireBody;
	}

	MultibodyVehicleControllerDG* CreateBasicVehicle(const dMatrix& location)
	{
		NewtonBody* const chassis = CreateChassis(location, 1200.0f, dVector(-0.15f, -0.65f, 0.0f), dVector(4.0f, 1.125f, 2.55f));

		MultibodyVehicleControllerDG* const controller = CreateController();
		//cVcontroller = controller;
		controller->Init(chassis);

		// front tires
		CreateTire(controller, /*TMODE_STEER_BREAK_HARDBREAK*/TMODE_TORQUE_STEER_BREAK_HARDBREAK, 50.0f, 0.35f, 0.25f, dVector(1.2f, -0.75f, 1.125f));
		CreateTire(controller, /*TMODE_STEER_BREAK_HARDBREAK*/TMODE_TORQUE_STEER_BREAK_HARDBREAK, 50.0f, 0.35f, 0.25f, dVector(1.2f, -0.75f, -1.125f));

		// rear Tires
		CreateTire(controller, TMODE_TORQUE_BREAK_HARDBREAK, 50.0f, 0.35f, 0.25f, dVector(-1.4f, -0.75f, 1.125f));
		CreateTire(controller, TMODE_TORQUE_BREAK_HARDBREAK, 50.0f, 0.35f, 0.25f, dVector(-1.4f, -0.75f, -1.125f));

		return controller;
	}
	
	MultibodyVehicleControllerDG* m_currentController;
	int m_tireMaterial;
	friend class dCustomVehicleControllerDG;
};

static bool IsPlayer(const MultibodyVehicleControllerDG* const controller)
{
	return ((MultibodyVehicleControllerManagerDG*)controller->GetManager())->m_currentController ? true : false;
}

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
	DemoMesh* const geometry = new DemoMesh("cylinder_1", scene->GetShaderCache(), collision, "wood_4.tga", "wood_4.tga", "wood_1.tga");

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

static void AddRamps (DemoEntityManager* const scene)
{
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterial = NewtonMaterialGetDefaultGroupID(world);

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
	DemoMesh* const StartBox = new DemoMesh("StartBox", scene->GetShaderCache(), collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
	CreateSimpleSolid(scene, StartBox, 0.0f, startlocation, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	StartBox->Release();
	
	//
	//startlocation2 = startlocation2 * dRollMatrix(2.5f * dDegreeToRad);
	startlocation2.m_posit.m_x += 38.0f;
	startlocation2.m_posit.m_y += 4.0f;
	collision = NewtonCreateBox(world, 20.0f, 0.25f, 40.0f, 0, NULL);
	DemoMesh* const StartBox3 = new DemoMesh("StartBox", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");
	CreateSimpleSolid(scene, StartBox3, 0.0f, startlocation2, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	StartBox3->Release();
	
	//	
	startlocation3 = startlocation3 * dRollMatrix(32.5f * dDegreeToRad);
	startlocation3.m_posit.m_x -= 31.5f;
	startlocation3.m_posit.m_y += 0.575f;
	collision = NewtonCreateBox(world, 2.0f, 2.0f, 100.0f, 0, NULL);
	DemoMesh* const StartBox2 = new DemoMesh("StartBox", scene->GetShaderCache(), collision, "wood_0.tga", "wood_0.tga", "wood_0.tga");
	CreateSimpleSolid(scene, StartBox2, 0.0f, startlocation3, collision, defaultMaterial);
	NewtonDestroyCollision(collision);
	StartBox2->Release();
}

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
	MultibodyVehicleControllerDG* const multiBodyVehicle = manager->CreateBasicVehicle (location);
	manager->m_currentController = multiBodyVehicle;


	int count = 10;
	count = 0;
	for (int u = 0; u < count; u++) {

		dVector VehicleFrameRotation(-180.0f, 30.0f, 0.0f);
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
		manager->CreateBasicVehicle (location4);
	}


	// add some more stuff to drive over
//	AddRamps (scene);
//	BuildPyramid(scene, 10.0f, dVector(-20.0f, 0.0f, 10.0f, 1.0f), dVector(0.5f, 0.25f, 0.8f, 0.0f), 10, _BOX_PRIMITIVE);
//	BuildPyramid(scene, 10.0f, dVector(-20.0f, 0.0f, -10.0f, 1.0f), dVector(0.5f, 0.25f, 0.8f, 0.0f), 10, _BOX_PRIMITIVE);

	// set the camera 
	location.m_posit = dVector(FindFloor(scene->GetNewton(), dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	dVector origin(FindFloor(world, dVector(-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	origin.m_y += 10.0f;
	origin.m_x += -45.0f;
	//
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}
