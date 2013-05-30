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

/*
class CustomVehicleController
{
	public:
	class BodyState;
	class TireBodyState;
	class ChassisBodyState;
	class TireList: public dList<TireBodyState>
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

		void InitalizeCurve (int points, const dFloat* const steps, const dFloat* const values);
		dFloat GetValue (dFloat param) const;

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

		protected:
		Component (CustomVehicleController* const controller)
			:m_controller(controller)
			,m_param(0.0f)
		{
		}

		virtual ~Component()
		{
		}
		
		virtual void Update (dFloat timestep) = 0;
		
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

			GearBox(CustomVehicleController* const controller, dFloat reverseGearRatio, int gearCount, const dFloat* const gearBoxRatios);

			int GetGear() const {return m_currentGear;}
			void SetGear(int gear) {m_currentGear = dClamp (gear, int (m_reverseGear), m_gearsCount - 1);}
			int GetGearCount() const {return m_gearsCount;}
			dFloat GetGearRatio(int gear) const {return (gear != m_reverseGear) ? gears[gear] : -gears[gear];}

			int m_gearsCount;
			int m_currentGear;
			dFloat gears[m_maxGears];
		};


		EngineComponent (CustomVehicleController* const controller, GearBox* const gearBox, TireList::dListNode* const leftTire, TireList::dListNode* const righTire);
		~EngineComponent();

		virtual void Update (dFloat timestep);
		
		void InitEngineTorqueCurve (dFloat vehicleSpeedKPH,
									dFloat poundFootIdleTorque, dFloat idleTorqueRPM, 
									dFloat poundFootPeakTorque, dFloat peakTorqueRPM, 
									dFloat poundFootPeackHorsePower, dFloat peakHorsePowerRPM, 
									dFloat poundFootRedLineTorque, dFloat redLineTorqueRPM);

		int GetGear () const;
		void SetGear (int gear);
		dFloat GetRPM () const;
		dFloat GetSpeed () const;
		dFloat GetTopSpeed () const;
		dFloat GetIdleFakeInertia() const;

		protected:
		void SetIdleFakeInertia(dFloat inertia);
		void SetTopSpeed (dFloat topSpeedMeterPerSecunds);
		dFloat CaculateEngineRPS (const TireBodyState* const tire, dFloat gearGain) const;
		
		dFloat m_speedMPS;
		dFloat m_currentRPS;
		dFloat m_topSpeedMPS;
		dFloat m_fakeIdleInertia;
		dFloat m_engineInternalInertia;
		dFloat m_differentialGearRatio;
		dFloat m_engineOptimalRevPerSec;

		GearBox* m_gearBox;
		TireList::dListNode* m_leftTire;
		TireList::dListNode* m_righTire;
		InterpolationCurve m_torqueCurve;
	};

	class BrakeComponent: public Component
	{
		public:
		BrakeComponent (CustomVehicleController* const controller, dFloat maxBrakeTorque);
		void AddBrakeTire (TireList::dListNode* const tire);
		virtual void Update (dFloat timestep);

		dFloat m_maxBrakeTorque;
		dList<TireList::dListNode*> m_brakeTires;
	};

	class SteeringComponent: public Component
	{
		public:
		class TireSignPair
		{
			public:
			dFloat m_sign;
			TireList::dListNode* m_tireNode;
		};

		SteeringComponent (CustomVehicleController* const controller, dFloat maxAngleInRadians);
		void AddSteeringTire (TireList::dListNode* const tire, dFloat sign);
		virtual void Update (dFloat timestep);

		dList<TireSignPair> m_steeringTires;
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

		virtual void Init(CustomVehicleController* const controller, BodyState* const state0, BodyState* const state1);

		virtual void UpdateSolverForces (const JacobianPair* const jacobians) const = 0; 
		virtual void JacobianDerivative (ParamInfo* const constraintParams) = 0; 
		virtual void JointAccelerations (JointAccelerationDecriptor* const accelParam);

		void InitPointParam (PointDerivativeParam& param, const dVector& pivot) const;
		void CalculateAngularDerivative (ParamInfo* const constraintParams, const dVector& dir,	dFloat jointAngle);
		void CalculatePointDerivative (ParamInfo* const constraintParams, const dVector& dir, const PointDerivativeParam& param);
		void AddLinearRowJacobian (ParamInfo* const constraintParams, const dVector& pivot, const dVector& dir);

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
		virtual void UpdateSolverForces (const JacobianPair* const jacobians) const; 
		virtual void JacobianDerivative (ParamInfo* const constraintParams); 
	};

	class ContactJoint: public VehicleJoint
	{
		public:
		ContactJoint ()
			:m_contactCount(0)
		{
		}
		virtual void UpdateSolverForces (const JacobianPair* const jacobians) const; 
		virtual void JacobianDerivative (ParamInfo* const constraintParams); 
		virtual void JointAccelerations (JointAccelerationDecriptor* const accelParam);

		int m_contactCount;
		NewtonWorldConvexCastReturnInfo m_contacts[4];
	};


	class BodyState
	{
		public:
		BodyState();
		virtual ~BodyState() {}
		
		void Init(CustomVehicleController* const controller);
		void UpdateInertia();
		virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
		
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
		void Init (CustomVehicleController* const controller, const dMatrix& localframe);
		void UpdateDynamicInputs();

		dVector m_com;
		dVector m_comOffset;
		dVector m_gravity;
	};

	class TireBodyState: public BodyState
	{
		public:
		void Init (CustomVehicleController* const controller, const TireCreationInfo& tireInfo);

		dFloat GetAdhesionCoefficient() const;
		void SetAdhesionCoefficient(dFloat Coefficient);

		dMatrix CalculateMatrix () const;
		dMatrix CalculateSteeringMatrix () const;
		dMatrix CalculateSuspensionMatrix () const;
		
		void Collide (CustomControllerFilterCastFilter& filter, dFloat timestepInv);
		void UpdateDynamicInputs(dFloat timestep);

		void UpdateTransform ();
		virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);

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

int xxx;
		
		void* m_userData;
		NewtonCollision* m_shape;
		TireJoint m_chassisJoint;
		ContactJoint m_contactJoint;
	};


	CUSTOM_CONTROLLER_GLUE(CustomVehicleController);
	public:

	// public functions
	TireList::dListNode* AddTire (const TireCreationInfo& tireInfo);
	TireList::dListNode* GetFirstTire () const ;
	void* GetUserData (TireList::dListNode* const tireNode) const;
	dMatrix GetTireLocalMatrix (TireList::dListNode* const tireNode) const;
	dMatrix GetTireGlobalMatrix (TireList::dListNode* const tireNode) const;

	const ChassisBodyState& GetChassisState () const;

	void SetLongitudinalSlipRatio(dFloat maxLongitudinalSlipRatio);
	void SetLateralSlipAngle(dFloat maxLongitudinalSlipAngleIndDegrees);


	void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);

	BrakeComponent* GetBrakes() const;
	EngineComponent* GetEngine() const;
	BrakeComponent* GetHandBrakes() const;
	SteeringComponent* GetSteering() const;

	void SetBrakes(BrakeComponent* const brakes);
	void SetEngine(EngineComponent* const engine);
	void SetHandBrakes(BrakeComponent* const brakes);
	void SetSteering(SteeringComponent* const steering);

	protected:
	void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	void Cleanup();

	virtual void PreUpdate(dFloat timestep, int threadIndex);
	virtual void PostUpdate(dFloat timestep, int threadIndex);

	int GetActiveJoints(VehicleJoint** const jointArray);
	int BuildJacobianMatrix (int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray);
	void CalculateReactionsForces(int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray);
	
	void UpdateTireTransforms ();

	BodyState m_staticWorld;
	ChassisBodyState m_chassisState;
	TireList m_tireList;

	BrakeComponent* m_brakes;
	EngineComponent* m_engine;
	BrakeComponent* m_handBrakes;
	SteeringComponent* m_steering; 
	NewtonCollision* m_tireCastShape;
	dList<BodyState*> m_stateList;
	
	InterpolationCurve m_tireLateralSlipAngle;
	InterpolationCurve m_tireLongitidialSlipRatio;
	friend class CustomVehicleControllerManager;

};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
	public:
	CustomVehicleControllerManager(NewtonWorld* const world);
	virtual ~CustomVehicleControllerManager();

	virtual CustomController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	virtual void DestroyController (CustomController* const controller);
};
*/

#endif 

