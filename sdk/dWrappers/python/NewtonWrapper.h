
/*
#ifdef NEWTONWRAPPER_EXPORTS
#define NEWTONWRAPPER_API extern "C" __declspec(dllexport)
#else
#define NEWTONWRAPPER_API __declspec(dllimport)
#endif


#include "Newton.h"

#include "CustomVehicleControllerManager.h"
#include "CustomBallAndSocket.h"
#include "CustomHinge.h"
#include "CustomDryRollingFriction.h"

// Joint Wrappers
NEWTONWRAPPER_API CustomBallAndSocket* NewtonCreateBallAndSocket(float* matrix, NewtonBody* child, NewtonBody* parent);
NEWTONWRAPPER_API CustomBallAndSocketWithFriction* NewtonCreateBallAndSocketWithFriction(float* matrix, NewtonBody* child, NewtonBody* parent, float friction);
NEWTONWRAPPER_API CustomHinge* NewtonCreateHinge(float* matrix, NewtonBody* child, NewtonBody* parent);
NEWTONWRAPPER_API void NewtonHingeEnableLimits(CustomHinge* hinge, bool state);
NEWTONWRAPPER_API void NewtonHingeSetLimits(CustomHinge* hinge, float minAngle, float maxAngle);
NEWTONWRAPPER_API void NewtonHingeSetFriction(CustomHinge* hinge, float friction);
NEWTONWRAPPER_API CustomDryRollingFriction* NewtonCreateRollingFriction(NewtonBody* child, float radius, float coeff);
NEWTONWRAPPER_API void NewtonDestroyCustomJoint(CustomJoint* joint);

// Vehicle Wrappers
class TireParameters
{
public:
	float width;
	float radius;
	float mass;
	float suspensionLenght;
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
	float peakTorque;
	float rpmAtPeakTorque;
	float peakHorsePower;
	float rpmAtPeakHorsePower;
	float redLineTorque;
	float rpmAtRedLineTorque;
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
	float clutchFrictionTorque;
};

NEWTONWRAPPER_API CustomVehicleControllerManager* NewtonCreateVehicleManager(NewtonWorld* world);
NEWTONWRAPPER_API void NewtonDestroyVehicleManager(CustomVehicleControllerManager* vehicleManager);
NEWTONWRAPPER_API CustomVehicleController* NewtonCreateVehicle(CustomVehicleControllerManager* vehicleManager, NewtonCollision* chassiShape, const dMatrix& matrix, float mass, float steeringMaxAngle, float maxBrakeTorque, NewtonApplyForceAndTorque forceCallback);
NEWTONWRAPPER_API void NewtonVehicleSetCOM(CustomVehicleController* vehicle, dVector& com);
NEWTONWRAPPER_API CustomVehicleController::BodyPartTire* NewtonVehicleAddTire(CustomVehicleController* vehicle, dVector& pos, TireParameters* params);
NEWTONWRAPPER_API void NewtonVehicleSteeringAddTire(CustomVehicleController* vehicle, CustomVehicleController::BodyPartTire* tire);
NEWTONWRAPPER_API void NewtonVehicleBrakesAddTire(CustomVehicleController* vehicle, CustomVehicleController::BodyPartTire* tire);
NEWTONWRAPPER_API void NewtonVehicleHandBrakesAddTire(CustomVehicleController* vehicle, CustomVehicleController::BodyPartTire* tire);
NEWTONWRAPPER_API void NewtonVehicleSetEngineParams(CustomVehicleController* vehicle, EngineParameters* params, CustomVehicleController::BodyPartTire* leftTire, CustomVehicleController::BodyPartTire* rightTire,
					CustomVehicleController::BodyPartTire* leftRearTire, CustomVehicleController::BodyPartTire* rightRearTire, int differentialType);
NEWTONWRAPPER_API void NewtonVehicleFinalize(CustomVehicleController* vehicle);
NEWTONWRAPPER_API void NewtonDestroyVehicle(CustomVehicleControllerManager* vehicleManager, CustomVehicleController* vehicle);
NEWTONWRAPPER_API void NewtonVehicleSetThrottle(CustomVehicleController* vehicle, float throttle);
NEWTONWRAPPER_API void NewtonVehicleSetSteering(CustomVehicleController* vehicle, float steering);
NEWTONWRAPPER_API void NewtonVehicleSetBrakes(CustomVehicleController* vehicle, float brakes);
NEWTONWRAPPER_API void NewtonVehicleSetHandBrakes(CustomVehicleController* vehicle, float brakes);
NEWTONWRAPPER_API void NewtonVehicleSetGear(CustomVehicleController* vehicle, int gear);
NEWTONWRAPPER_API float NewtonVehicleGetSpeed(CustomVehicleController* vehicle);
NEWTONWRAPPER_API int NewtonVehicleGetGear(CustomVehicleController* vehicle);
NEWTONWRAPPER_API float NewtonVehicleGetRPM(CustomVehicleController* vehicle);
NEWTONWRAPPER_API NewtonBody* NewtonVehicleGetBody(CustomVehicleController* vehicle);
NEWTONWRAPPER_API NewtonBody* NewtonVehicleTireGetBody(CustomVehicleController::BodyPartTire* tire);
*/

