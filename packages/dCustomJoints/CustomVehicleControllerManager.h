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


// NewtonVehicleControllerManager.h: interface for the NewtonVehicleControllerManager class.
//
//////////////////////////////////////////////////////////////////////

#ifndef D_CUSTOM_VEHICLE_CONTROLLER_MANAGER_H_
#define D_CUSTOM_VEHICLE_CONTROLLER_MANAGER_H_

#include "CustomJointLibraryStdAfx.h"
#include "CustomControllerManager.h"


#define VEHICLE_PLUGIN_NAME			"vehicleManager"


class CustomVehicleController
{
	public:
	class BodyState;
	class TireBodyState;
	class ChassisBodyState;

	class TireList: public CustomList<TireBodyState>
	{
	};

	class TireCreationInfo
	{
		public:
		dVector m_location;
		dFloat m_mass;
		dFloat m_radio;
		dFloat m_width;
		dFloat m_dampingRatio;
		dFloat m_springStrength;
		dFloat m_suspesionlenght;
		void* m_userData;
	};

	class InterpolationCurve
	{
		public:
		class dNot
		{
			public:
			dFloat m_param;
			dFloat m_value;
		};

		InterpolationCurve ()
		{	
			m_count = 0;
		}

		~InterpolationCurve ()
		{
		}

		NEWTON_API void InitalizeCurve (int points, const dFloat* const steps, const dFloat* const values);
		NEWTON_API dFloat GetValue (dFloat param) const;

		dNot m_nodes[6];
		int m_count;
	};


	class Component
	{
		public:
		void SetParam(dFloat param)
		{
			m_param = param;
		}

		dFloat GetParam() 
		{
			return m_param;
		}

		NEWTON_API void* operator new (size_t size);
		NEWTON_API void operator delete (void* ptr);

		protected:
		Component (CustomVehicleController* const controller)
			:m_controller(controller)
			,m_param(0.0f)
		{
		}

		virtual ~Component()
		{
		}
		
		NEWTON_API virtual void Update (dFloat timestep) = 0;
		
		CustomVehicleController* m_controller; 
		dFloat m_param;
	};

	class EngineComponent: public Component
	{
		public:
		class GearBox
		{
			public:
			enum Gear
			{
				m_reverseGear = 0,
				m_newtralGear,
				m_firstGear,
				m_maxGears = 16
			};

			NEWTON_API void* operator new (size_t size);
			NEWTON_API void operator delete (void* ptr);

			NEWTON_API GearBox(CustomVehicleController* const controller, dFloat reverseGearRatio, int gearCount, const dFloat* const gearBoxRatios);

			NEWTON_API int GetGear() const {return m_currentGear;}
			NEWTON_API void SetGear(int gear) {m_currentGear = dClamp (gear, int (m_reverseGear), m_gearsCount - 1);}
			NEWTON_API int GetGearCount() const {return m_gearsCount;}
			NEWTON_API dFloat GetGearRatio(int gear) const {return (gear != m_reverseGear) ? gears[gear] : -gears[gear];}

			int m_gearsCount;
			int m_currentGear;
			dFloat gears[m_maxGears];
		};


		NEWTON_API EngineComponent (CustomVehicleController* const controller, GearBox* const gearBox, TireBodyState* const leftTire, TireBodyState* const righTire);
		NEWTON_API ~EngineComponent();

		NEWTON_API virtual void Update (dFloat timestep);
		
		NEWTON_API void InitEngineTorqueCurve (dFloat vehicleSpeedKPH,
									dFloat poundFootIdleTorque, dFloat idleTorqueRPM, 
									dFloat poundFootPeakTorque, dFloat peakTorqueRPM, 
									dFloat poundFootPeackHorsePower, dFloat peakHorsePowerRPM, 
									dFloat poundFootRedLineTorque, dFloat redLineTorqueRPM);

		NEWTON_API int GetGear () const;
		NEWTON_API void SetGear (int gear);
		NEWTON_API dFloat GetRPM () const;
		NEWTON_API dFloat GetSpeed () const;
		NEWTON_API dFloat GetTopSpeed () const;
		NEWTON_API dFloat GetIdleFakeInertia() const;

		protected:
		NEWTON_API void SetIdleFakeInertia(dFloat inertia);
		NEWTON_API void SetTopSpeed (dFloat topSpeedMeterPerSecunds);
		NEWTON_API dFloat CaculateEngineRPS (const TireBodyState* const tire, dFloat gearGain) const;
		
		dFloat m_speedMPS;
		dFloat m_currentRPS;
		dFloat m_topSpeedMPS;
		dFloat m_fakeIdleInertia;
		dFloat m_engineInternalInertia;
		dFloat m_differentialGearRatio;
		dFloat m_engineOptimalRevPerSec;

		GearBox* m_gearBox;
		TireList::CustomListNode* m_leftTire;
		TireList::CustomListNode* m_righTire;
		InterpolationCurve m_torqueCurve;
	};

	class BrakeComponent: public Component
	{
		public:
		NEWTON_API BrakeComponent (CustomVehicleController* const controller, dFloat maxBrakeTorque);
		NEWTON_API void AddBrakeTire (TireBodyState* const tire);
		NEWTON_API virtual void Update (dFloat timestep);

		dFloat m_maxBrakeTorque;
		CustomList<TireList::CustomListNode*> m_brakeTires;
	};

	class SteeringComponent: public Component
	{
		public:
		class TireSignPair
		{
			public:
			dFloat m_sign;
			TireList::CustomListNode* m_tireNode;
		};

		NEWTON_API SteeringComponent (CustomVehicleController* const controller, dFloat maxAngleInRadians);
		NEWTON_API void AddSteeringTire (TireBodyState* const tire, dFloat sign);
		NEWTON_API virtual void Update (dFloat timestep);

		CustomList<TireSignPair> m_steeringTires;
		dFloat m_maxAngle;
	};


	class VehicleJoint
	{
		public:
		class Jacobian
		{
			public:
			dVector m_linear;
			dVector m_angular;
		};

		class JacobianPair
		{
			public:
			Jacobian m_jacobian_IM0;
			Jacobian m_jacobian_IM1;
		};

		class JacobianColum
		{
			public:
			dFloat m_force;
			dFloat m_diagDamp;
			dFloat m_deltaAccel;
			dFloat m_invDJMinvJt;
			dFloat m_coordenateAccel;
			dFloat m_jointLowFriction;
			dFloat m_jointHighFriction;
		};

		class ParamInfo
		{
			public:
			JacobianPair m_jacobians[8];
			dFloat m_jointAccel[8];
			dFloat m_jointLowFriction[8];
			dFloat m_jointHighFriction[8];
			int m_count;
			dFloat m_timestep;
			dFloat m_timestepInv;
		};

		class PointDerivativeParam
		{
			public:
			dVector m_r0;
			dVector m_posit0;
			dVector m_veloc0;
			dVector m_centripetal0;

			dVector m_r1;
			dVector m_posit1;
			dVector m_veloc1;
			dVector m_centripetal1;
		};

		class JointAccelerationDecriptor
		{
			public:
			int m_rowsCount;
			dFloat m_timeStep;
			dFloat m_invTimeStep;
			dFloat m_firstPassCoefFlag;
			JacobianPair* m_rowMatrix;
			JacobianColum* m_colMatrix;
		};


		VehicleJoint(){}
		virtual ~VehicleJoint(){}

		NEWTON_API virtual void Init(CustomVehicleController* const controller, BodyState* const state0, BodyState* const state1);

		NEWTON_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const = 0; 
		NEWTON_API virtual void JacobianDerivative (ParamInfo* const constraintParams) = 0; 
		NEWTON_API virtual void JointAccelerations (JointAccelerationDecriptor* const accelParam);

		NEWTON_API void InitPointParam (PointDerivativeParam& param, const dVector& pivot) const;
		NEWTON_API void CalculateAngularDerivative (ParamInfo* const constraintParams, const dVector& dir,	dFloat jointAngle);
		NEWTON_API void CalculatePointDerivative (ParamInfo* const constraintParams, const dVector& dir, const PointDerivativeParam& param);
		NEWTON_API void AddLinearRowJacobian (ParamInfo* const constraintParams, const dVector& pivot, const dVector& dir);

		int m_rowIsMotor[8];
		dFloat m_motorAcceleration[8];
		dFloat m_jointFeebackForce[8];
		BodyState* m_state0;
		BodyState* m_state1;
		CustomVehicleController* m_controller; 
		int m_start;
		int m_count;
	};

	class TireJoint: public VehicleJoint
	{
		public:
		NEWTON_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const; 
		NEWTON_API virtual void JacobianDerivative (ParamInfo* const constraintParams); 
	};

	class ContactJoint: public VehicleJoint
	{
		public:
		ContactJoint ()
			:m_contactCount(0)
		{
		}
		NEWTON_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const; 
		NEWTON_API virtual void JacobianDerivative (ParamInfo* const constraintParams); 
		NEWTON_API virtual void JointAccelerations (JointAccelerationDecriptor* const accelParam);

		int m_contactCount;
		NewtonWorldConvexCastReturnInfo m_contacts[4];
	};


	class BodyState
	{
		public:
		NEWTON_API BodyState();
		NEWTON_API virtual ~BodyState() {}
		
		NEWTON_API void Init(CustomVehicleController* const controller);
		NEWTON_API void UpdateInertia();
		NEWTON_API virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
		
		dMatrix m_matrix;
		dMatrix m_localFrame;
		dMatrix m_inertia;
		dMatrix m_invInertia;

		dVector m_localInertia;
		dVector m_localInvInertia;

		dVector m_veloc;
		dVector m_omega;
		dVector m_externalForce;
		dVector m_externalTorque;
		dVector m_globalCentreOfMass;
		
		dFloat m_mass;

		dFloat m_invMass;
		int m_myIndex;
		CustomVehicleController* m_controller;
	};

	class ChassisBodyState: public BodyState
	{
		public:
		NEWTON_API void Init (CustomVehicleController* const controller, const dMatrix& localframe);
		NEWTON_API void UpdateDynamicInputs();

		dVector m_com;
		dVector m_comOffset;
		dVector m_gravity;
	};

	class TireBodyState: public BodyState
	{
		public:
		NEWTON_API void Init (CustomVehicleController* const controller, const TireCreationInfo& tireInfo);

		NEWTON_API dFloat GetAdhesionCoefficient() const;
		NEWTON_API void SetAdhesionCoefficient(dFloat Coefficient);

		NEWTON_API dMatrix CalculateMatrix () const;
		NEWTON_API dMatrix CalculateSteeringMatrix () const;
		NEWTON_API dMatrix CalculateSuspensionMatrix () const;
		
		NEWTON_API void Collide (CustomControllerFilterCastFilter& filter, dFloat timestepInv);
		NEWTON_API void UpdateDynamicInputs(dFloat timestep);

		NEWTON_API void UpdateTransform ();
		NEWTON_API virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);

		dVector m_tireLoad;
		dVector m_lateralForce;
		dVector m_longitidinalForce;
		dFloat m_radio;
		dFloat m_width;
		dFloat m_posit;
		dFloat m_speed;
		dFloat m_breakTorque;
		dFloat m_engineTorque;
		dFloat m_rotatonSpeed;
		dFloat m_rotationAngle;
		dFloat m_steeringAngle;
		dFloat m_dampingRatio;
		dFloat m_springStrength;
		dFloat m_suspensionlenght;
		dFloat m_adhesionCoefficient; 
		dFloat m_idleRollingResistance;
		dFloat m_engineTorqueResistance;
	
		void* m_userData;
		NewtonCollision* m_shape;
		TireJoint m_chassisJoint;
		ContactJoint m_contactJoint;
	};


	CUSTOM_CONTROLLER_GLUE(CustomVehicleController);
	public:

	// public functions
	NEWTON_API TireBodyState* AddTire (const TireCreationInfo& tireInfo);
	NEWTON_API TireBodyState* GetFirstTire () const ;
	NEWTON_API TireBodyState* GetNextTire (TireBodyState* const tire) const;

	NEWTON_API void* GetUserData (TireBodyState* const tireNode) const;
	NEWTON_API dMatrix GetTireLocalMatrix (TireBodyState* const tireNode) const;
	NEWTON_API dMatrix GetTireGlobalMatrix (TireBodyState* const tireNode) const;

	NEWTON_API const ChassisBodyState& GetChassisState () const;

	NEWTON_API void SetLongitudinalSlipRatio(dFloat maxLongitudinalSlipRatio);
	NEWTON_API void SetLateralSlipAngle(dFloat maxLongitudinalSlipAngleIndDegrees);

	NEWTON_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);

	NEWTON_API BrakeComponent* GetBrakes() const;
	NEWTON_API EngineComponent* GetEngine() const;
	NEWTON_API BrakeComponent* GetHandBrakes() const;
	NEWTON_API SteeringComponent* GetSteering() const;

	NEWTON_API void SetBrakes(BrakeComponent* const brakes);
	NEWTON_API void SetEngine(EngineComponent* const engine);
	NEWTON_API void SetHandBrakes(BrakeComponent* const brakes);
	NEWTON_API void SetSteering(SteeringComponent* const steering);

	protected:
	NEWTON_API void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	NEWTON_API void Cleanup();

	NEWTON_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	NEWTON_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	NEWTON_API int GetActiveJoints(VehicleJoint** const jointArray);
	NEWTON_API int BuildJacobianMatrix (int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray);
	NEWTON_API void CalculateReactionsForces(int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray);
	
	NEWTON_API void UpdateTireTransforms ();

	BodyState m_staticWorld;
	ChassisBodyState m_chassisState;
	TireList m_tireList;

	BrakeComponent* m_brakes;
	EngineComponent* m_engine;
	BrakeComponent* m_handBrakes;
	SteeringComponent* m_steering; 
	NewtonCollision* m_tireCastShape;
	CustomList<BodyState*> m_stateList;
	
	InterpolationCurve m_tireLateralSlipAngle;
	InterpolationCurve m_tireLongitidialSlipRatio;
	friend class CustomVehicleControllerManager;

};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
	public:
	NEWTON_API CustomVehicleControllerManager(NewtonWorld* const world);
	NEWTON_API virtual ~CustomVehicleControllerManager();

	NEWTON_API virtual CustomController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	NEWTON_API virtual void DestroyController (CustomController* const controller);
};


#endif 

