#include "stdafx.h"
#include "NewtonWrapper.h"
#include "CustomJoint.h"
#include "CustomVehicleControllerManager.h"

class VehicleParameters
{
public:
	float weightDistribution;
	float downForceWeightFactor0;
	float downForceweightFactor1;
	float downForceWeightFactorSpeed;
	float gravity;
};

class TireParameters
{
public:
	float width;
	float radius;
	float mass;
	float maxSteeringAngle;
	bool hasFender;
	CustomVehicleController::BodyPartTire::Info::SuspensionType suspensionType;

	float suspensionLength;
	float springStrength;
	float dampingRatio;

	float lateralStiffness;
	float longitudinalStiffness;

	float aligningMomentTrail;
};

class EngineParameters
{
public:
	float mass;
	float ratio;
	float topSpeed;
	float clutchFrictionTorque;

	float peakTorque;
	float rpmAtPeakTorque;
	float peakHorsePower;
	float rpmAtPeakHorsePower;
	float redLineTorque;
	float rpmAtReadLineTorque;
	float idleTorque;
	float rpmAtIdleTorque;

	int	gearsCount;
	float gearRatioReverse;
	float gearRatio1;
	float gearRatio2;
	float gearRatio3;
	float gearRatio4;
	float gearRatio5;
	float gearRatio6;
	float gearRatio7;
	float gearRatio8;
	float gearRatio9;
	float gearRatio10;
};

extern "C"
{

	NEWTONWRAPPER_API CustomVehicleControllerManager* WRAP_NewtonCreateVehicleManager(NewtonWorld* world)
	{
		int defaultMaterial = NewtonMaterialGetDefaultGroupID(world);
		int materialList[] = { defaultMaterial };

		return new CustomVehicleControllerManager(world, 1, materialList);
	}

	NEWTONWRAPPER_API void WRAP_NewtonDestroyVehicleManager(CustomVehicleControllerManager* vehicleManager)
	{
		delete vehicleManager;
	}

	NEWTONWRAPPER_API CustomVehicleController* WRAP_NewtonCreateVehicle(CustomVehicleControllerManager* vehicleManager, NewtonCollision* chassiShape, const dMatrix& matrix, float mass, float maxBrakeTorque, NewtonApplyForceAndTorque forceCallback)
	{
		dMatrix vehicleFrame;
		vehicleFrame.m_front = dVector(1.0f, 0.0f, 0.0f, 0.0f);				// this is the vehicle direction of travel
		vehicleFrame.m_up = dVector(0.0f, 1.0f, 0.0f, 0.0f);				// this is the downward vehicle direction
		vehicleFrame.m_right = vehicleFrame.m_front * vehicleFrame.m_up;	// this is in the side vehicle direction (the plane of the wheels)
		vehicleFrame.m_posit = dVector(0.0f, 0.0f, 0.0f, 1.0f);

		CustomVehicleController* v = vehicleManager->CreateVehicle(chassiShape, vehicleFrame, mass, forceCallback, 0);

		NewtonBody* carBody = v->GetBody();
		NewtonBodySetMatrix(carBody, &matrix[0][0]);

		CustomVehicleController::SteeringController* steering = new CustomVehicleController::SteeringController(v);
		v->SetSteering(steering);

		CustomVehicleController::BrakeController* brakes = new CustomVehicleController::BrakeController(v, maxBrakeTorque);
		v->SetBrakes(brakes);

		CustomVehicleController::BrakeController* handBrakes = new CustomVehicleController::BrakeController(v, maxBrakeTorque);
		v->SetHandBrakes(handBrakes);

		return v;
	}

	NEWTONWRAPPER_API CustomVehicleController::BodyPartTire* WRAP_NewtonVehicleAddTire(CustomVehicleController* vehicle, float* pos, TireParameters* params)
	{
		CustomVehicleController::BodyPartTire::Info tireInfo;
		dVector tirePos;
		tirePos[0] = pos[0];
		tirePos[1] = pos[1];
		tirePos[2] = pos[2];
		tirePos[3] = 0;

		tireInfo.m_location = tirePos;
		tireInfo.m_mass = params->mass;
		tireInfo.m_radio = params->radius;
		tireInfo.m_width = params->width;
		tireInfo.m_maxSteeringAngle = params->maxSteeringAngle * 3.1416f / 180.0f;
		tireInfo.m_dampingRatio = params->dampingRatio;
		tireInfo.m_springStrength = params->springStrength;
		tireInfo.m_suspesionlenght = params->suspensionLength;
		tireInfo.m_lateralStiffness = dAbs(params->lateralStiffness);
		tireInfo.m_longitudialStiffness = dAbs(params->longitudinalStiffness);
		tireInfo.m_aligningMomentTrail = params->aligningMomentTrail;
		tireInfo.m_hasFender = params->hasFender;
		tireInfo.m_suspentionType = params->suspensionType;

		return vehicle->AddTire(tireInfo);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetEngineParams(CustomVehicleController* vehicle, VehicleParameters* vparams, EngineParameters* params,
		CustomVehicleController::BodyPartTire* leftFrontTire, CustomVehicleController::BodyPartTire* rightFrontTire,
		CustomVehicleController::BodyPartTire* leftRearTire, CustomVehicleController::BodyPartTire* rightRearTire, int differentialType)
	{
		CustomVehicleController::EngineController::Info engineInfo;
		engineInfo.m_mass = params->mass;
		engineInfo.m_radio = params->ratio;
		engineInfo.m_vehicleTopSpeed = params->topSpeed;
		engineInfo.m_clutchFrictionTorque = params->clutchFrictionTorque;

		engineInfo.m_peakTorque = params->peakTorque;
		engineInfo.m_rpmAtPeakTorque = params->rpmAtPeakTorque;
		engineInfo.m_peakHorsePower = params->peakHorsePower;
		engineInfo.m_rpmAtPeakHorsePower = params->rpmAtPeakHorsePower;
		engineInfo.m_redLineTorque = params->redLineTorque;
		engineInfo.m_rpmAtReadLineTorque = params->rpmAtReadLineTorque;
		engineInfo.m_idleTorque = params->idleTorque;
		engineInfo.m_rpmAtIdleTorque = params->rpmAtIdleTorque;

		engineInfo.m_gearsCount = params->gearsCount;
		engineInfo.m_gearRatios[0] = params->gearRatio1;
		engineInfo.m_gearRatios[1] = params->gearRatio2;
		engineInfo.m_gearRatios[2] = params->gearRatio3;
		engineInfo.m_gearRatios[3] = params->gearRatio4;
		engineInfo.m_gearRatios[4] = params->gearRatio5;
		engineInfo.m_gearRatios[5] = params->gearRatio6;
		engineInfo.m_gearRatios[6] = params->gearRatio7;
		engineInfo.m_gearRatios[7] = params->gearRatio8;
		engineInfo.m_gearRatios[8] = params->gearRatio9;
		engineInfo.m_gearRatios[9] = params->gearRatio10;
		engineInfo.m_reverseGearRatio = params->gearRatioReverse;

		CustomVehicleController::EngineController::Differential4wd differential;
		switch (differentialType)
		{
		case 0:
			differential.m_type = CustomVehicleController::EngineController::Differential::m_2wd;
			differential.m_axel.m_leftTire = leftRearTire;
			differential.m_axel.m_rightTire = rightRearTire;
			break;
		case 1:
			differential.m_type = CustomVehicleController::EngineController::Differential::m_2wd;
			differential.m_axel.m_leftTire = leftFrontTire;
			differential.m_axel.m_rightTire = rightFrontTire;
			break;

		default:
			differential.m_type = CustomVehicleController::EngineController::Differential::m_4wd;
			differential.m_axel.m_leftTire = leftRearTire;
			differential.m_axel.m_rightTire = rightRearTire;
			differential.m_secundAxel.m_axel.m_leftTire = leftFrontTire;
			differential.m_secundAxel.m_axel.m_rightTire = rightFrontTire;
		}

		engineInfo.m_differentialLock = 0;

		CustomVehicleController::EngineController* engineControl = new CustomVehicleController::EngineController(vehicle, engineInfo, differential);

		vehicle->SetEngine(engineControl);

		engineControl->SetTransmissionMode(true); //Automatic Transmission
		engineControl->SetIgnition(true);
		engineControl->SetClutchParam(1);

		vehicle->SetWeightDistribution(vparams->weightDistribution);

		dFloat weightRatio0 = vparams->downForceWeightFactor0;
		dFloat weightRatio1 = vparams->downForceweightFactor1;
		dFloat speedFactor = vparams->downForceWeightFactorSpeed / params->topSpeed;
		vehicle->SetAerodynamicsDownforceCoefficient(vparams->gravity, weightRatio0, speedFactor, weightRatio1);

	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSteeringAddTire(CustomVehicleController* vehicle, CustomVehicleController::BodyPartTire* tire)
	{
		CustomVehicleController::SteeringController* steering = vehicle->GetSteering();
		steering->AddTire(tire);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleCalculateAckermannParameters(CustomVehicleController* vehicle,
		CustomVehicleController::BodyPartTire* leftFrontTire, CustomVehicleController::BodyPartTire* rightFrontTire,
		CustomVehicleController::BodyPartTire* leftRearTire, CustomVehicleController::BodyPartTire* rightRearTire)
	{
		CustomVehicleController::SteeringController* steering = vehicle->GetSteering();

		steering->CalculateAkermanParameters(leftRearTire, rightRearTire, leftFrontTire, rightFrontTire);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleBrakesAddTire(CustomVehicleController* vehicle, CustomVehicleController::BodyPartTire* tire)
	{
		CustomVehicleController::BrakeController* brakes = vehicle->GetBrakes();
		brakes->AddTire(tire);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleHandBrakesAddTire(CustomVehicleController* vehicle, CustomVehicleController::BodyPartTire* tire)
	{
		CustomVehicleController::BrakeController* handBrakes = vehicle->GetHandBrakes();
		handBrakes->AddTire(tire);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleFinalize(CustomVehicleController* vehicle)
	{
		vehicle->Finalize();
	}


	NEWTONWRAPPER_API void WRAP_NewtonDestroyVehicle(CustomVehicleControllerManager* vehicleManager, CustomVehicleController* vehicle)
	{
		// Destroy chassi body
		NewtonDestroyBody(vehicle->GetBody());

		// Destroy tires
		for (dList<CustomVehicleController::BodyPartTire>::dListNode* node = vehicle->GetFirstTire(); node; node = node->GetNext())
		{
			CustomVehicleController::BodyPartTire* const tire = &node->GetInfo();
			NewtonBody* tireBody = tire->GetBody();
			NewtonDestroyBody(tireBody);
		}

		// Destroy vehicle controller
		vehicleManager->DestroyController(vehicle);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetThrottle(CustomVehicleController* vehicle, float throttle)
	{
		vehicle->GetEngine()->SetParam(throttle);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetSteering(CustomVehicleController* vehicle, float steering)
	{
		vehicle->GetSteering()->SetParam(steering);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetBrakes(CustomVehicleController* vehicle, float brakes)
	{
		vehicle->GetBrakes()->SetParam(brakes);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetHandBrakes(CustomVehicleController* vehicle, float brakes)
	{
		vehicle->GetHandBrakes()->SetParam(brakes);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetClutch(CustomVehicleController* vehicle, float clutch)
	{
		vehicle->GetEngine()->SetClutchParam(clutch);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetGear(CustomVehicleController* vehicle, int gear)
	{
		vehicle->GetEngine()->SetGear(gear);
	}

	NEWTONWRAPPER_API float WRAP_NewtonVehicleGetSpeed(CustomVehicleController* vehicle)
	{
		return vehicle->GetEngine()->GetSpeed();
	}

	NEWTONWRAPPER_API int WRAP_NewtonVehicleGetGear(CustomVehicleController* vehicle)
	{
		return vehicle->GetEngine()->GetGear();
	}

	NEWTONWRAPPER_API float WRAP_NewtonVehicleGetRPM(CustomVehicleController* vehicle)
	{
		return vehicle->GetEngine()->GetRPM();
	}

	NEWTONWRAPPER_API NewtonBody* WRAP_NewtonVehicleGetBody(CustomVehicleController* vehicle)
	{
		return vehicle->GetBody();
	}

	NEWTONWRAPPER_API NewtonBody* WRAP_NewtonVehicleTireGetBody(CustomVehicleController::BodyPartTire* tire)
	{
		return tire->GetBody();
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetCOM(CustomVehicleController* vehicle, dVector& com)
	{
		vehicle->SetCenterOfGravity(com);
	}

	NEWTONWRAPPER_API void WRAP_NewtonVehicleSetDifferentialLock(CustomVehicleController* vehicle, bool mode)
	{
		vehicle->GetEngine()->SetDifferentialLock(mode);
	}

}