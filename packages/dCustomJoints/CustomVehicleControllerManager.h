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
#include "CustomAlloc.h"
#include "CustomControllerManager.h"


#define VEHICLE_PLUGIN_NAME			"__vehicleManager__"


class CustomVehicleController: public CustomControllerBase
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

		CUSTOM_JOINTS_API void InitalizeCurve (int points, const dFloat* const steps, const dFloat* const values);
		CUSTOM_JOINTS_API dFloat GetValue (dFloat param) const;

		dNot m_nodes[6];
		int m_count;
	};


	class Component: public CustomAlloc  
	{
		public:
		void SetParam(dFloat param)
		{
			m_param = param;
		}

		dFloat GetParam() const 
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
		
		CUSTOM_JOINTS_API virtual void Update (dFloat timestep) = 0;
		
		CustomVehicleController* m_controller; 
		dFloat m_param;
	};

	class EngineComponent: public Component
	{
		public:
		class GearBox
		{
			public:
			enum GearID
			{
				m_reverseGear = 0,
				m_newtralGear,
				m_firstGear,
				m_maxGears = 16
			};

			class GearState
			{
				public:
                GearState (dFloat ratio, dFloat shiftUp, dFloat shiftDown, GearID id) 
                    :m_ratio(ratio)
                    ,m_shiftUp (shiftUp)
                    ,m_shiftDown (shiftDown)
                    ,m_next(NULL)
                    ,m_prev(NULL)
                    ,m_id(id)
                {
                }

                virtual ~GearState()
                {
                }
                virtual GearState* Update(CustomVehicleController* const vehicle);

				dFloat m_ratio;
				dFloat m_shiftUp;
				dFloat m_shiftDown;
				GearState* m_next;
				GearState* m_prev;
                GearState* m_neutral;
                GearState* m_reverse;
                GearID m_id;
			};

            class ReverseGearState: public GearState
            {
                public:
                ReverseGearState (dFloat ratio)
                    :GearState(ratio, 1000.0f, -1000.0f, m_reverseGear)
                {
                }

                 GearState* Update(CustomVehicleController* const vehicle)
                 {
                     return NULL;
                 }
            };
            
            class NeutralGearState: public GearState
            {
                public:
                NeutralGearState (GearState* const first, GearState* const reverse)
                    :GearState(0.0f, 1000.0f, -1000.0f, m_newtralGear)
                {
                    m_next = first;
                    m_prev = reverse;
                }
                GearState* Update(CustomVehicleController* const vehicle);
            };

			CUSTOM_JOINTS_API GearBox (CustomVehicleController* const controller, dFloat reverseGearRatio, int gearCount, const dFloat* const gearBoxRatios);
            CUSTOM_JOINTS_API ~GearBox ();
			CUSTOM_JOINTS_API void Update (dFloat timestep);
			CUSTOM_JOINTS_API dFloat GetGearRatio(int gear) const;


            CUSTOM_JOINTS_API void SetGear(int gear);
			CUSTOM_JOINTS_API int GetGear() const {return m_currentGear->m_id;}
			CUSTOM_JOINTS_API int GetGearCount() const {return m_gearsCount;}

			bool GetTransmissionMode () const;
			void SetTransmissionMode (bool mode);
			void SetOptimalShiftLimits (dFloat minShift, dFloat maxShift);

			GearState* m_gears[m_maxGears];
            GearState* m_currentGear;
            CustomVehicleController* m_controller;
			int m_gearsCount;
			bool m_automatic;
		};


		CUSTOM_JOINTS_API EngineComponent (CustomVehicleController* const controller, GearBox* const gearBox, TireBodyState* const leftTire, TireBodyState* const righTire);
		CUSTOM_JOINTS_API ~EngineComponent();

		CUSTOM_JOINTS_API virtual void Update (dFloat timestep);
		
		CUSTOM_JOINTS_API void InitEngineTorqueCurve (
                                    dFloat vehicleSpeedInKilometerPerHours, dFloat engineMomentOfInertia,
									dFloat idleTorqueInPoundFoot, dFloat revolutionsPerMinutesAtIdleTorque, 
									dFloat peakTorqueInPoundFoot, dFloat revolutionsPerMinutesAtPeakTorque, 
									dFloat peakHorsePower, dFloat revolutionsPerMinutesAtPeakHorsePower, 
									dFloat torqueArRedLineInPoundFoot, dFloat revolutionsPerMinutesAtRedLineTorque);

		CUSTOM_JOINTS_API int GetGear () const;
		CUSTOM_JOINTS_API void SetGear (int gear);
		CUSTOM_JOINTS_API dFloat GetRPM () const;
        CUSTOM_JOINTS_API dFloat GetTopRPM () const;
		CUSTOM_JOINTS_API dFloat GetSpeed () const;
		CUSTOM_JOINTS_API dFloat GetTopSpeed () const;
		CUSTOM_JOINTS_API dFloat GetInertia() const;
		CUSTOM_JOINTS_API void SetInertia(dFloat inertia);
		CUSTOM_JOINTS_API bool GetTransmissionMode () const;
		CUSTOM_JOINTS_API void SetTransmissionMode (bool mode);


        GearBox* GetGearBox() const;
        dFloat GetIdleResistance () const;
        dFloat GetRedLineResistance () const;
		dFloat GetIdleRadianPerSeconds () const;
        dFloat GetDifferencialGearRatio () const;
        dFloat GetTorque (dFloat radianPerSeconds) const;

        void SetTopSpeed (dFloat topSpeedMeterPerSecunds);
        TireList::CustomListNode* GetLeftTireNode() const;
        TireList::CustomListNode* GetRightTireNode() const;

		protected:
		

		GearBox* m_gearBox;
		TireList::CustomListNode* m_leftTire;
		TireList::CustomListNode* m_righTire;
		InterpolationCurve m_torqueCurve;
		
		dFloat m_speedMPS;
		dFloat m_topSpeedMPS;
        dFloat m_momentOfInertia;
		dFloat m_engineResistance;
        dFloat m_engineIdleResistance;
		dFloat m_differentialGearRatio;
		dFloat m_radiansPerSecundsAtRedLine;
		dFloat m_radiansPerSecundsAtPeakPower;
		dFloat m_radiansPerSecundsAtIdleTorque;
	};

	class BrakeComponent: public Component
	{
		public:
		CUSTOM_JOINTS_API BrakeComponent (CustomVehicleController* const controller, dFloat maxBrakeTorque);
		CUSTOM_JOINTS_API void AddBrakeTire (TireBodyState* const tire);
		CUSTOM_JOINTS_API virtual void Update (dFloat timestep);

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

		CUSTOM_JOINTS_API SteeringComponent (CustomVehicleController* const controller, dFloat maxAngleInRadians);
		CUSTOM_JOINTS_API void AddSteeringTire (TireBodyState* const tire, dFloat sign);
		CUSTOM_JOINTS_API virtual void Update (dFloat timestep);

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

		CUSTOM_JOINTS_API virtual void Init(CustomVehicleController* const controller, BodyState* const state0, BodyState* const state1);

		CUSTOM_JOINTS_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const = 0; 
		CUSTOM_JOINTS_API virtual void JacobianDerivative (ParamInfo* const constraintParams) = 0; 
		CUSTOM_JOINTS_API virtual void JointAccelerations (JointAccelerationDecriptor* const accelParam);

		CUSTOM_JOINTS_API void InitPointParam (PointDerivativeParam& param, const dVector& pivot) const;
		CUSTOM_JOINTS_API void CalculateAngularDerivative (ParamInfo* const constraintParams, const dVector& dir,	dFloat jointAngle);
		CUSTOM_JOINTS_API void CalculatePointDerivative (ParamInfo* const constraintParams, const dVector& dir, const PointDerivativeParam& param);
		CUSTOM_JOINTS_API void AddLinearRowJacobian (ParamInfo* const constraintParams, const dVector& pivot, const dVector& dir);

		int m_rowIsMotor[8];
		dFloat m_motorAcceleration[8];
		dFloat m_jointFeebackForce[8];
		BodyState* m_state0;
		BodyState* m_state1;
		CustomVehicleController* m_controller; 
		int m_start;
		int m_count;
	};

    class EngineGearJoint: public VehicleJoint
    {
        public:
        CUSTOM_JOINTS_API virtual void JacobianDerivative (ParamInfo* const constraintParams); 
        CUSTOM_JOINTS_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const;

        dFloat m_powerTrainGain;
    };

	class EngineIdleJoint: public VehicleJoint
	{
		public:
		CUSTOM_JOINTS_API virtual void JacobianDerivative (ParamInfo* const constraintParams); 
		CUSTOM_JOINTS_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const;

		dFloat m_omega;
		dFloat m_friction;
	};


	class TireJoint: public VehicleJoint
	{
		public:
        CUSTOM_JOINTS_API virtual void JacobianDerivative (ParamInfo* const constraintParams); 
		CUSTOM_JOINTS_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const; 
	};

	class ContactJoint: public VehicleJoint
	{
		public:
		CUSTOM_JOINTS_API ContactJoint ();
		CUSTOM_JOINTS_API virtual void UpdateSolverForces (const JacobianPair* const jacobians) const; 
		CUSTOM_JOINTS_API virtual void JacobianDerivative (ParamInfo* const constraintParams); 
		CUSTOM_JOINTS_API virtual void JointAccelerations (JointAccelerationDecriptor* const accelParam);

		int m_contactCount;
		NewtonWorldConvexCastReturnInfo m_contacts[4];
	};


	class BodyState
	{
		public:
		CUSTOM_JOINTS_API BodyState();
		CUSTOM_JOINTS_API virtual ~BodyState() {}
		
		CUSTOM_JOINTS_API void Init(CustomVehicleController* const controller);
		CUSTOM_JOINTS_API void UpdateInertia();
		CUSTOM_JOINTS_API virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
        CUSTOM_JOINTS_API virtual void CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega);
		
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
		CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller, const dMatrix& localframe);
		CUSTOM_JOINTS_API void UpdateDynamicInputs();
        CUSTOM_JOINTS_API virtual void CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega);

		dVector m_com;
		dVector m_comOffset;
		dVector m_gravity;
	};

    class EngineBodyState: public BodyState
    {
        public:
        CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller);

        CUSTOM_JOINTS_API void Update (dFloat timestep, CustomVehicleController* const controller);
        CUSTOM_JOINTS_API void CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega);
        CUSTOM_JOINTS_API int CalculateActiveJoints (CustomVehicleController* const controller, VehicleJoint** const jointArray);
        
        EngineGearJoint m_leftTire;
        EngineGearJoint m_rightTire;
		EngineIdleJoint	m_idleFriction;
        dFloat m_radianPerSecund;
    };

	class TireBodyState: public BodyState
	{
		public:
		CUSTOM_JOINTS_API void Init (CustomVehicleController* const controller, const TireCreationInfo& tireInfo);

		CUSTOM_JOINTS_API dFloat GetAdhesionCoefficient() const;
		CUSTOM_JOINTS_API void SetAdhesionCoefficient(dFloat Coefficient);

		CUSTOM_JOINTS_API dMatrix CalculateMatrix () const;
		CUSTOM_JOINTS_API dMatrix CalculateSteeringMatrix () const;
		CUSTOM_JOINTS_API dMatrix CalculateSuspensionMatrix () const;
		
		CUSTOM_JOINTS_API void Collide (CustomControllerConvexCastPreFilter& filter, dFloat timestepInv);
		CUSTOM_JOINTS_API void UpdateDynamicInputs(dFloat timestep);

		CUSTOM_JOINTS_API void UpdateTransform ();
		CUSTOM_JOINTS_API virtual void IntegrateForce (dFloat timestep, const dVector& force, const dVector& torque);
        CUSTOM_JOINTS_API virtual void CalculateAverageAcceleration (dFloat invTimestep, const dVector& veloc, const dVector& omega);

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
//		dFloat m_engineTorqueResistance;
	
		void* m_userData;
		NewtonCollision* m_shape;
		TireJoint m_chassisJoint;
		ContactJoint m_contactJoint;
	};

	public:
	CUSTOM_JOINTS_API TireBodyState* AddTire (const TireCreationInfo& tireInfo);
	CUSTOM_JOINTS_API TireBodyState* GetFirstTire () const ;
	CUSTOM_JOINTS_API TireBodyState* GetNextTire (TireBodyState* const tire) const;

	CUSTOM_JOINTS_API void* GetUserData (TireBodyState* const tireNode) const;
	CUSTOM_JOINTS_API dMatrix GetTireLocalMatrix (TireBodyState* const tireNode) const;
	CUSTOM_JOINTS_API dMatrix GetTireGlobalMatrix (TireBodyState* const tireNode) const;

	CUSTOM_JOINTS_API const ChassisBodyState& GetChassisState () const;

	CUSTOM_JOINTS_API void SetLongitudinalSlipRatio(dFloat maxLongitudinalSlipRatio);
	CUSTOM_JOINTS_API void SetLateralSlipAngle(dFloat maxLongitudinalSlipAngleIndDegrees);

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);

	CUSTOM_JOINTS_API BrakeComponent* GetBrakes() const;
	CUSTOM_JOINTS_API EngineComponent* GetEngine() const;
	CUSTOM_JOINTS_API BrakeComponent* GetHandBrakes() const;
	CUSTOM_JOINTS_API SteeringComponent* GetSteering() const;

	CUSTOM_JOINTS_API void SetBrakes(BrakeComponent* const brakes);
	CUSTOM_JOINTS_API void SetEngine(EngineComponent* const engine);
	CUSTOM_JOINTS_API void SetHandBrakes(BrakeComponent* const brakes);
	CUSTOM_JOINTS_API void SetSteering(SteeringComponent* const steering);

	protected:
	CUSTOM_JOINTS_API void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	CUSTOM_JOINTS_API void Cleanup();

	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	CUSTOM_JOINTS_API int GetActiveJoints(VehicleJoint** const jointArray);
	CUSTOM_JOINTS_API int BuildJacobianMatrix (int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray);
	CUSTOM_JOINTS_API void CalculateReactionsForces(int jointCount, VehicleJoint** const jointArray, dFloat timestep, VehicleJoint::JacobianPair* const jacobianArray, VehicleJoint::JacobianColum* const jacobianColumnArray);
	
	CUSTOM_JOINTS_API void UpdateTireTransforms ();

	BodyState m_staticWorld;
    EngineBodyState m_engineState;
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
	CUSTOM_JOINTS_API CustomVehicleControllerManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~CustomVehicleControllerManager();

	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, const dVector& gravityVector);
	CUSTOM_JOINTS_API virtual void DestroyController (CustomVehicleController* const controller);
};


#endif 

