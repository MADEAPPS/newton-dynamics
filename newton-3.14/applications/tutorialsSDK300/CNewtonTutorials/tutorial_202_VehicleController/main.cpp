/* 
 * File:   main.cpp
 * Author: main
 *
 * Created on 24 de julio de 2014, 21:17
 */

#include "stdafx.h"
#include <iostream>
#include <dVector.h>
#include <dMatrix.h>
#include <dNewton.h>
#include <dNewtonCollision.h>
#include <dNewtonDynamicBody.h>
#include <dNewtonVehicleManager.h>

struct BasciCarParameters
{
	enum DifferentialType
	{
		m_4WD,
		m_RWD,
		m_FWD,
	};

	dFloat MASS;
	dFloat TIRE_MASS;
	dFloat STEER_ANGLE;
	dFloat BRAKE_TORQUE;
	dFloat COM_Y_OFFSET;
	dFloat TIRE_TOP_SPEED_KMH;

	dFloat IDLE_TORQUE;
	dFloat IDLE_TORQUE_RPM;

	dFloat PEAK_TORQUE;
	dFloat PEAK_TORQUE_RPM;

	dFloat PEAK_HP;
	dFloat PEAK_HP_RPM;

	dFloat REDLINE_TORQUE;
	dFloat REDLINE_TORQUE_RPM;

	dFloat GEAR_1;
	dFloat GEAR_2;
	dFloat GEAR_3;
	dFloat REVERSE_GEAR;

	dFloat SUSPENSION_LENGTH;
	dFloat SUSPENSION_SPRING;
	dFloat SUSPENSION_DAMPER;
	dFloat LATERAL_STIFFNESS;
	dFloat LONGITUDINAL_STIFFNESS;
	dFloat ALIGNING_MOMENT_TRAIL;

	DifferentialType m_differentialType;
	dMatrix m_tireaLigment;
};

static BasciCarParameters basicCarParameters = 
{
	2500.0f,	// MASS
	100.0f,	// TIRE_MASS
	25.0f,	// STEER_ANGLE
	10000.0f,	// BRAKE_TORQUE
	-0.6f,	// COM_Y_OFFSET
	120.0f,		// TIRE_TOP_SPEED_KMH
	400.0f,	// IDLE_TORQUE
	500.0f,		// IDLE_TORQUE_RPM
	500.0f,	// PEAK_TORQUE
	3000.0f,	// PEAK_TORQUE_RPM
	300.0f,	// PEAK_HP
	4000.0f,	// PEAK_HP_RPM
	50.0f,		// REDLINE_TORQUE
	4500.0f,	// REDLINE_TORQUE_RPM
	2.5f,	// GEAR_1
	2.0f,	// GEAR_2
	1.5f,	// GEAR_3
	2.9f,	// REVERSE_GEAR
	0.7f,	// SUSPENSION_LENGTH
	700.0f,	// SUSPENSION_SPRING
	40.0f,	// SUSPENSION_DAMPER
	20.0f,	// LATERAL_STIFFNESS
	10000.0f,	// LONGITUDINAL_STIFFNESS
	1.5f,	// ALIGNING_MOMENT_TRAIL
	BasciCarParameters::m_4WD,

	dGetIdentityMatrix(),
};


static float VehicleHullShape0[][3] =  
{
	{-2.3f, 0.0f, -0.9f}, {-2.3f, 0.0f, 0.9f}, {2.3f, 0.0f, -0.9f}, {2.3f, 0.0f, 0.9f},
	{-2.1f, 0.7f, -0.9f}, {-2.1f, 0.7f, 0.9f}, {2.1f, 0.7f, -0.9f}, {2.1f, 0.7f, 0.9f},
};

static float VehicleHullShape1[][3] =  
{
	{-1.5f, 0.0f, -0.9f}, {-1.5f, 0.0f, 0.9f}, {1.2f, 0.0f, -0.9f}, {1.2f, 0.0f, 0.9f},
	{-1.1f, 0.7f, -0.9f}, {-1.1f, 0.7f, 0.9f}, {0.8f, 0.7f, -0.9f}, {0.8f, 0.7f, 0.9f},
};


//-----------------------------------------------------------------------------
using namespace std;
//-----------------------------------------------------------------------------
class MyDynamicBody: public dNewtonDynamicBody
{
   public:
   MyDynamicBody(dNewton* const world, dFloat mass, const dNewtonCollision* const collision, void* const userData, const dMatrix& matrix)
      :dNewtonDynamicBody (world, mass, collision, userData, &matrix[0][0], NULL)
   {
   }

   // the end application need to overload this function from dNetwonBody
   void OnBodyTransform (const dFloat* const matrix, int threadIndex)
   {
      Update (matrix);
   }

   // the end application need to overload this function from dNetwonDynamicBody
   void OnForceAndTorque (dFloat timestep, int threadIndex)
   {
      // apply gravity force to the body
      dFloat mass;
      dFloat Ixx;
      dFloat Iyy;
      dFloat Izz;

      GetMassAndInertia (mass, Ixx, Iyy, Izz);
      dVector gravityForce (0.0f, -9.8f * mass, 0.0f);
      SetForce (&gravityForce[0]);
   }
};



//-----------------------------------------------------------------------------
class MyVehicle : public dNewtonVehicleManager::dNewtonVehicle

{
	public:
	MyVehicle (dNewtonVehicleManager* const manager, const dNewtonCollision& collisiosnShape, void* const userData, const dFloat* const location, dLong collisionMask)
            : dNewtonVehicle (manager, collisiosnShape, userData, location)
    {
    }
    
    ~MyVehicle()
    {
    }

/*
	NewtonCollision* const chassisCollision = CreateChassisCollision (world);

	// caret the visual mesh form the collision shape
	DemoMesh* const visualMesh = new DemoMesh ("vehicle chassis", chassisCollision, "metal_30.tga", "metal_30.tga", "metal_30.tga");
	SetMesh (visualMesh, dGetIdentityMatrix());
	visualMesh->Release();

	// create the coordinate system 
	dMatrix chassisMatrix;
	chassisMatrix.m_front = dVector (1.0f, 0.0f, 0.0f, 0.0f);			// this is the vehicle direction of travel
	chassisMatrix.m_up	  = dVector (0.0f, 1.0f, 0.0f, 0.0f);			// this is the downward vehicle direction
	chassisMatrix.m_right = chassisMatrix.m_front * chassisMatrix.m_up;	// this is in the side vehicle direction (the plane of the wheels)
	chassisMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);

	// create a default vehicle controller
	m_controller = manager->CreateVehicle (chassisCollision, chassisMatrix, parameters.MASS, dVector (0.0f, DEMO_GRAVITY, 0.0f, 0.0f));

	// get body the vehicle rigid body and set the Newton rigid body physics properties
	NewtonBody* const body = m_controller->GetBody();

	// set the user data
	NewtonBodySetUserData(body, this);

	// set the transform callback
	NewtonBodySetTransformCallback (body, DemoEntity::TransformCallback);

	// set the standard force and torque call back
	NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);

	// set the player matrix 
	NewtonBodySetMatrix(body, &location[0][0]);

	// destroy the collision helper shape 
	NewtonDestroyCollision(chassisCollision);

	// map the gear to a look up table: gear 0 is reverse, gea 1 is neutral, gear 1 is first, gear 2 is second and so on
	for (int i = 0; i < int ((sizeof (m_gearMap) / sizeof (m_gearMap[0]))); i ++) {
		m_gearMap[i] = i;
	}
	m_gearMap[0] = 1;
	m_gearMap[1] = 0;
*/


	void BuidlBasicCar (const BasciCarParameters& parameters)
	{
/*
		// step one: find the location of each tire, in the visual mesh and add them one by one to the vehicle controller 
		dFloat width = 0.35f;
		dFloat radius = 0.5f;

		// Muscle cars have the front engine, we need to shift the center of mass to the front to represent that
		m_controller->SetCenterOfGravity (dVector (0.0f, parameters.COM_Y_OFFSET, 0.0f, 0.0f)); 

		// add left tires
		CustomVehicleControllerBodyStateTire* leftTire[2]; 
		dVector offset (1.5f, 0.0f, -1.0f, 1.0f);
		leftTire[0] = AddTire (offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireaLigment);
		offset = dVector (-1.7f, 0.0f, -1.0f, 1.0f);
		leftTire[1] = AddTire (offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireaLigment);

		// add right tires
		CustomVehicleControllerBodyStateTire* rightTire[2];
		offset = dVector (1.5f, 0.0f, 1.0f, 1.0f);		
		rightTire[0] = AddTire (offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireaLigment);
		offset = dVector (-1.7f, 0.0f, 1.0f, 1.0f);
		rightTire[1] = AddTire (offset, width, radius, parameters.TIRE_MASS, parameters.SUSPENSION_LENGTH, parameters.SUSPENSION_SPRING, parameters.SUSPENSION_DAMPER, parameters.LATERAL_STIFFNESS, parameters.LONGITUDINAL_STIFFNESS, parameters.ALIGNING_MOMENT_TRAIL, parameters.m_tireaLigment);

		// add a steering Wheel
		CustomVehicleControllerComponentSteering* const steering = new CustomVehicleControllerComponentSteering (m_controller, parameters.STEER_ANGLE * 3.141592f / 180.0f);
		steering->AddSteeringTire (leftTire[0]);
		steering->AddSteeringTire (rightTire[0]);
		m_controller->SetSteering(steering);

		// add all wheels brakes
		CustomVehicleControllerComponentBrake* const brakes = new CustomVehicleControllerComponentBrake (m_controller, parameters.BRAKE_TORQUE);
		for (int i = 0; i < 2; i ++) {
			brakes->AddBrakeTire (leftTire[i]);
			brakes->AddBrakeTire (rightTire[i]);
		}
		m_controller->SetBrakes(brakes);

		// add hand brakes
		CustomVehicleControllerComponentBrake* const handBrakes = new CustomVehicleControllerComponentBrake (m_controller, parameters.BRAKE_TORQUE);
		handBrakes->AddBrakeTire (leftTire[1]);
		handBrakes->AddBrakeTire (rightTire[1]);
		m_controller->SetHandBrakes (handBrakes);

		// add an engine
		CustomVehicleControllerComponentEngine::dMultiAxelDifferential* differencial = NULL;
		switch (parameters.m_differentialType)
		{
		case BasciCarParameters::m_4WD:
			{
				CustomVehicleControllerComponentEngine::dSingleAxelDifferential* axles[2];
				for (int i = 0; i < 2; i ++) {
					axles[i] = new CustomVehicleControllerComponentEngine::dSingleAxelDifferential (m_controller, leftTire[i], rightTire[i]);
				}
				differencial = new CustomVehicleControllerComponentEngine::dMultiAxelDifferential (m_controller, 2, axles);
				break;
			}

		case BasciCarParameters::m_FWD:
			{
				CustomVehicleControllerComponentEngine::dSingleAxelDifferential* axles[1];
				axles[0] = new CustomVehicleControllerComponentEngine::dSingleAxelDifferential (m_controller, leftTire[0], rightTire[0]);
				differencial = new CustomVehicleControllerComponentEngine::dMultiAxelDifferential (m_controller, 1, axles);
				break;
			}

		case BasciCarParameters::m_RWD:
		default:
			{
				CustomVehicleControllerComponentEngine::dSingleAxelDifferential* axles[1];
				axles[0] = new CustomVehicleControllerComponentEngine::dSingleAxelDifferential (m_controller, leftTire[1], rightTire[1]);
				differencial = new CustomVehicleControllerComponentEngine::dMultiAxelDifferential (m_controller, 1, axles);
				break;
			}
		}


		dFloat fowardSpeedGearsBoxRatios[] = {parameters.GEAR_1, parameters.GEAR_1, parameters.GEAR_3};
		CustomVehicleControllerComponentEngine::dGearBox* const gearBox = new CustomVehicleControllerComponentEngine::dGearBox(m_controller, parameters.REVERSE_GEAR, sizeof (fowardSpeedGearsBoxRatios) / sizeof (fowardSpeedGearsBoxRatios[0]), fowardSpeedGearsBoxRatios); 
		CustomVehicleControllerComponentEngine* const engine = new CustomVehicleControllerComponentEngine (m_controller, gearBox, differencial);

		// the the default transmission type
		engine->SetTransmissionMode(m_automaticTransmission.GetPushButtonState());
		m_controller->SetEngine(engine);


		dFloat viperIdleRPM = parameters.IDLE_TORQUE_RPM;
		dFloat viperIdleTorquePoundPerFoot = parameters.IDLE_TORQUE;

		dFloat viperPeakTorqueRPM = parameters.PEAK_TORQUE_RPM;
		dFloat viperPeakTorquePoundPerFoot = parameters.PEAK_TORQUE;

		dFloat viperPeakHorsePowerRPM = parameters.PEAK_HP_RPM;
		dFloat viperPeakHorsePower = parameters.PEAK_HP;

		dFloat viperRedLineRPM = parameters.REDLINE_TORQUE_RPM;
		dFloat viperRedLineTorquePoundPerFoot = parameters.REDLINE_TORQUE;

		dFloat vehicleTopSpeedKPH = parameters.TIRE_TOP_SPEED_KMH;
		engine->InitEngineTorqueCurve (vehicleTopSpeedKPH, viperIdleTorquePoundPerFoot, viperIdleRPM, viperPeakTorquePoundPerFoot, viperPeakTorqueRPM, viperPeakHorsePower, viperPeakHorsePowerRPM, viperRedLineTorquePoundPerFoot, viperRedLineRPM);

		// do not forget to call finalize after all components are added or after any change is made to the vehicle
		m_controller->Finalize();
*/
	}

	private:
/*
	NewtonCollision* CreateChassisCollision (NewtonWorld* const world) const
	{
		dMatrix offset (dGetIdentityMatrix());
		offset.m_posit.m_y = 0.7f;

		NewtonCollision* const convex0 = NewtonCreateConvexHull (world, 8, &VehicleHullShape0[0][0], 3 * sizeof (dFloat), 0.001f, 0, NULL);
		NewtonCollision* const convex1 = NewtonCreateConvexHull (world, 8, &VehicleHullShape1[0][0], 3 * sizeof (dFloat), 0.001f, 0, &offset[0][0]);

		NewtonCollision* const collision = NewtonCreateCompoundCollision(world, 0);

		NewtonCompoundCollisionBeginAddRemove(collision);
		NewtonCompoundCollisionAddSubCollision (collision, convex0);
		NewtonCompoundCollisionAddSubCollision (collision, convex1);
		NewtonCompoundCollisionEndAddRemove (collision);	

		NewtonDestroyCollision (convex0);
		NewtonDestroyCollision (convex1);

		return collision;
	}
*/

    
/*
    void OnPlayerMove (dFloat timestep)
    {
        dVector gravity(0.0f, -9.81f, 0.0f);
        SetPlayerVelocity (0.0f, 0.0f, 0.0f, 0.0f, &gravity[0], timestep);
    }
*/
};


class MyVehicleManager: public dNewtonVehicleManager
{
	public:
	MyVehicleManager (dNewton* const world)
		:dNewtonVehicleManager (world)
	{
	}

	~MyVehicleManager()
	{
	}

	dNewtonCollisionCompound MakeChassisShape (dLong collisionMask) const
	{
		dNewton* const workd = GetWorld();

		dNewtonCollisionCompound shape (workd, 0);

		dMatrix offset (dGetIdentityMatrix());
		offset.m_posit.m_y = 0.7f;

		dNewtonCollisionConvexHull convex0 (workd, 8, &VehicleHullShape0[0][0], 3 * sizeof (dFloat), 0.001f, collisionMask);
		dNewtonCollisionConvexHull convex1 (workd, 8, &VehicleHullShape1[0][0], 3 * sizeof (dFloat), 0.001f, collisionMask);
		convex1.SetMatrix (&offset[0][0]);

		shape.BeginAddRemoveCollision();
		shape.AddCollision (&convex0);
		shape.AddCollision (&convex1);
		shape.EndAddRemoveCollision();
		return shape;
	}
};

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    dNewton world;

    // create floor
    dNewtonCollisionBox floor_collision (&world, 100, 1, 100, 1);

	// add a plane to support the vehicle
    MyDynamicBody* const floor = new MyDynamicBody (&world, 0, &floor_collision, NULL, dGetIdentityMatrix());
    
   
	// create a vehicle manager
    MyVehicleManager* const manager = new MyVehicleManager(&world);

	// create as many vehicle as you want
	dNewtonCollisionCompound shape (manager->MakeChassisShape (1));

//    MyVehicle* const player = new MyVehicle (manager, NULL, mass, outerRadius, innerRadius, height, stairStep, &upDir[0], &frontDir[0], 1);
/*    
    dMatrix matrix;
    player->GetMatrix(&matrix[0][0]);
    matrix.m_posit.m_y = 5.0f;
    player->SetMatrix(&matrix[0][0]);
    
    std::cout << "height: " << std::endl;
    
    for (int i = 0; i < 1500; i ++)
    {
        world.UpdateOffLine(1.0f/60.f);
        dMatrix matrix;
		player->InterpolateMatrix(1.0f, &matrix[0][0]);
		std::cout << matrix.m_posit.m_y << " ";
    }
*/
    return 0;
}

