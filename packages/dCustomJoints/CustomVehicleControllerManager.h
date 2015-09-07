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

#include <CustomJointLibraryStdAfx.h>
#include <CustomAlloc.h>
#include <CustomControllerManager.h>

#define VEHICLE_PLUGIN_NAME			"__vehicleManager__"


//#define __TEST_VEHICLE_XXX__

/*
class CustomVehicleControllerTireCollisionFilter: public CustomControllerConvexCastPreFilter
{	
	public:
	CustomVehicleControllerTireCollisionFilter (){}
	CUSTOM_JOINTS_API CustomVehicleControllerTireCollisionFilter (const CustomVehicleController* const controller);

	CUSTOM_JOINTS_API virtual unsigned Prefilter(const NewtonBody* const body, const NewtonCollision* const myCollision)
	{
		return 1;
	}

	CUSTOM_JOINTS_API virtual dFloat GetTireFrictionCoefficient (const CustomVehicleControllerBodyStateTire& tire, const NewtonBody* const body, const NewtonCollision* const myCollision, dLong contacID) const
	{
		//return 1.5f;
		return 1.0f;
	}

	const CustomVehicleController* m_controller;
};
*/

class CustomJoint;
class CustomVehicleController: public CustomControllerBase
{
	public:
	class dWeightDistibutionSolver;

	class dInterpolationCurve
	{
		public:
		class dNot
		{
		public:
			dFloat m_param;
			dFloat m_value;
		};

		dInterpolationCurve()
		{
			m_count = 0;
		}

		~dInterpolationCurve()
		{
		}

		CUSTOM_JOINTS_API void InitalizeCurve(int points, const dFloat* const steps, const dFloat* const values);
		CUSTOM_JOINTS_API dFloat GetValue(dFloat param) const;

		dNot m_nodes[6];
		int m_count;
	};


	
	class BodyPart: public CustomAlloc
	{
		public:
		BodyPart()
			:m_parent(NULL)
			,m_body(NULL)
			,m_joint(NULL)
			,m_userData(NULL)
			,m_controller(NULL)
		{
		}
		virtual ~BodyPart(){}

		BodyPart* GetParent() const {return m_parent;}
		NewtonBody* GetBody() const	{return m_body;}
		CustomJoint* GetJoint() const{return m_joint;}
		void* GetUserData() const {return m_userData;}
		CustomVehicleController* GetController() const{return m_controller;}
		
		protected:
		BodyPart* m_parent;
		NewtonBody* m_body;
		CustomJoint* m_joint;
		
		void* m_userData;
		CustomVehicleController* m_controller;
		friend class CustomVehicleController;
	};

	class BodyPartChassis: public BodyPart
	{
		public:
		BodyPartChassis ()
			:BodyPart()
		{
		}

		void Init(CustomVehicleController* const controller, void* const userData)
		{
			m_joint = NULL;
			m_userData = userData;
			m_controller = controller;
			m_body = controller->GetBody();
		}
	};
	

	class BodyPartTire: public BodyPart
	{
		public:
		class WheelJoint;
		class Info
		{
			public:
			dVector m_location;
			dFloat m_mass;
			dFloat m_radio;
			dFloat m_width;
			dFloat m_dampingRatio;
			dFloat m_aligningPinDir;
			dFloat m_springStrength;
			dFloat m_suspesionlenght;
			dFloat m_lateralStiffness;
			dFloat m_longitudialStiffness;
			dFloat m_aligningMomentTrail;
			void* m_userData;
		};

		BodyPartTire();
		~BodyPartTire();
		void Init (BodyPart* const parentPart, const dMatrix& locationInGlobalSpase, const Info& info);

		void SetSteerAngle (dFloat angle);
		void SetBrakeTorque (dFloat torque);

		Info m_data;
	};

	class BodyPartDifferential: public BodyPart
	{
		public:
		class DifferentialJoint;
		class DifferentialSpiderGearJoint;

		BodyPartDifferential (BodyPartChassis* const chassis, const BodyPartTire* const leftTire, const BodyPartTire* const rightTire);
		virtual ~BodyPartDifferential();
		
		int GetTireCounty() const ;
		const BodyPartTire* GetTire(int index) const ;

		protected:
		int m_tireCount;
		const BodyPartTire* m_tire[8];
	};

	class BodyPartEngine: public BodyPart
	{
		public:
		class EngineJoint;
		class Info
		{
			public:
			dFloat m_mass;
			dFloat m_radio;
			dFloat m_peakTorque; 
			dFloat m_rpmAtPeakTorque;
			dFloat m_peakHorsePower; 
			dFloat m_rpmAtPeakHorsePower; 
			dFloat m_redLineTorque; 
			dFloat m_rpmAtReadLineTorque; 
			dFloat m_idleTorque; 
			dFloat m_rpmAtIdleTorque;
			dFloat m_vehicleTopSpeed;
			void* m_userData;

			private:
			void ConvertToMetricSystem();

			dFloat m_crownGearRatio;
			dFloat m_peakPowerTorque;
			dFloat m_engineIdleDryDrag;
			dFloat m_engineIdleViscousDrag;
			friend class BodyPartEngine;
		};

		BodyPartEngine(BodyPartChassis* const chassis, BodyPartDifferential* const differential, const Info& info, int gearCount, dFloat* const gearRatios, dFloat reverseGear);
		virtual ~BodyPartEngine();
		void Update (dFloat timestep, dFloat gasVal);

		dFloat GetRPM() const;
		dFloat GetRedLineRPM() const;

		protected:
		void SetTopSpeed(BodyPartDifferential* const differential);
		void InitEngineTorqueCurve(BodyPartDifferential* const differential);
		
		Info m_data;
		dFloat m_gear[10];
		int m_gearCount;
		dInterpolationCurve m_torqueRPMCurve;
	};

	class Controller: public CustomAlloc
	{
		public:
		Controller (CustomVehicleController* const controller)
			:m_controller(controller)
			,m_param(0.0f)
			,m_paramMemory(0.0f)
			,m_timer(60)
		{
		}

		~Controller()
		{
		}

		void SetParam( dFloat param)
		{
			m_paramMemory = m_param;
			m_param = param;
		}

		dFloat GetParam() const
		{
			return m_param;
		}

		bool ParamChanged() const
		{
			m_timer--;
			if (dAbs(m_paramMemory - m_param) > 1.e-3f) {
				m_timer = 30;
			}
			return m_timer > 0;
		}


		virtual void Update(dFloat timestep)
		{
		}

		CustomVehicleController* m_controller;
		
		dFloat m_param;
		dFloat m_paramMemory;
		mutable dFloat m_timer;
	};

	class SteeringController: public Controller
	{
		public:
		CUSTOM_JOINTS_API SteeringController (CustomVehicleController* const controller, dFloat maxAngle);
		CUSTOM_JOINTS_API void AddTire (BodyPartTire* const tire);
		CUSTOM_JOINTS_API void CalculateAkermanParameters(const BodyPartTire* const rearLeftTire, const BodyPartTire* const rearRightTire,
														  const BodyPartTire* const frontLeftTire, const BodyPartTire* const frontRightTire);

		protected:
		virtual void Update(dFloat timestep);

		dList<BodyPartTire*> m_tires;
		dFloat m_maxAngle;
		dFloat m_akermanWheelBaseWidth;
		dFloat m_akermanAxelSeparation;
		friend class CustomVehicleController;
	};

	class BrakeController: public Controller
	{
		public:
		CUSTOM_JOINTS_API BrakeController (CustomVehicleController* const controller, dFloat maxBrakeTorque);
		CUSTOM_JOINTS_API void AddTire (BodyPartTire* const tire);

		protected:
		virtual void Update(dFloat timestep);

		dList<BodyPartTire*> m_tires;
		dFloat m_brakeTorque;
		friend class CustomVehicleController;
	};

	class EngineController: public Controller
	{
		public:
		CUSTOM_JOINTS_API EngineController (CustomVehicleController* const controller, BodyPartEngine* const engine);

		CUSTOM_JOINTS_API int GetGear() const;
		CUSTOM_JOINTS_API dFloat GetRPM() const;
		CUSTOM_JOINTS_API dFloat GetRedLineRPM() const;
		CUSTOM_JOINTS_API dFloat GetSpeed() const;

		CUSTOM_JOINTS_API bool GetClutch() const;
		CUSTOM_JOINTS_API void SetClutch(bool state);

		protected:
		virtual void Update(dFloat timestep);

		BodyPartEngine* m_engine;
		friend class CustomVehicleController;
	};


#if 0
	CUSTOM_JOINTS_API CustomVehicleControllerBodyStateTire* GetFirstTire () const ;
	CUSTOM_JOINTS_API CustomVehicleControllerBodyStateTire* GetNextTire (CustomVehicleControllerBodyStateTire* const tire) const;

	CUSTOM_JOINTS_API const CustomVehicleControllerBodyStateChassis& GetChassisState () const;

	CUSTOM_JOINTS_API dFloat GetAerodynamicsDowforceCoeficient () const;
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient (dFloat maxDownforceInGravities, dFloat topSpeed);

	CUSTOM_JOINTS_API void SetDryRollingFrictionTorque (dFloat torque);
	CUSTOM_JOINTS_API dFloat GetDryRollingFrictionTorque () const;

	CUSTOM_JOINTS_API CustomVehicleControllerComponentBrake* GetBrakes() const;
	CUSTOM_JOINTS_API CustomVehicleControllerComponentEngine* GetEngine() const;
	CUSTOM_JOINTS_API CustomVehicleControllerComponentBrake* GetHandBrakes() const;
	CUSTOM_JOINTS_API void SetEngine(CustomVehicleControllerComponentEngine* const engine);

	CUSTOM_JOINTS_API void SetContactFilter(CustomVehicleControllerTireCollisionFilter* const filter);
	CUSTOM_JOINTS_API void LinksTiresKinematically (int count, CustomVehicleControllerBodyStateTire** const tires);
	

	protected:
	CustomVehicleControllerTireCollisionFilter* m_contactFilter;
	CustomVehicleControllerBodyStateContact* m_externalContactStates[16];

#endif

	CUSTOM_JOINTS_API void Finalize();

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);
	CUSTOM_JOINTS_API BodyPartTire* AddTire (const BodyPartTire::Info& tireInfo);
	CUSTOM_JOINTS_API BodyPartDifferential* AddDifferential2WD (const BodyPartTire* const leftTire, const BodyPartTire* const rightire);
	CUSTOM_JOINTS_API BodyPartEngine* AddEngine (const BodyPartEngine::Info& engineInfo, BodyPartDifferential* const differential, int gearCount, dFloat* const gearRatios, dFloat reverseGear);

	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetFirstBodyPart() const;
	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetNextBodyPart(dList<BodyPart*>::dListNode* const part) const;

	CUSTOM_JOINTS_API BrakeController* GetBrakes() const;
	CUSTOM_JOINTS_API EngineController* GetEngine() const;
	CUSTOM_JOINTS_API BrakeController* GetHandBrakes() const;
	CUSTOM_JOINTS_API SteeringController* GetSteering() const;
	
	CUSTOM_JOINTS_API void SetBrakes(BrakeController* const brakes);
	CUSTOM_JOINTS_API void SetHandBrakes(BrakeController* const brakes);
	CUSTOM_JOINTS_API void SetSteering(SteeringController* const steering);
	CUSTOM_JOINTS_API void SetEngine(EngineController* const engine);

	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	bool ControlStateChanged() const;
	void Init (NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	
	void Cleanup();
	
	dMatrix m_localFrame;
	BodyPartChassis m_chassis;
	dList<BodyPartTire> m_tireList;
	dList<BodyPart*> m_bodyPartsList;
	
	NewtonSkeletonContainer* m_skeleton;
	void* m_collisionAggregate;
	NewtonCollision* m_tireCastShape;
	BodyPartEngine* m_engine;
	BodyPartDifferential* m_differential;
	
	BrakeController* m_brakesControl;
	EngineController* m_engineControl;
	BrakeController* m_handBrakesControl;
	SteeringController* m_steeringControl; 


	NewtonApplyForceAndTorque m_forceAndTorque;

	bool m_finalized;
	friend class CustomVehicleControllerManager;
};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
	public:
	CUSTOM_JOINTS_API CustomVehicleControllerManager(NewtonWorld* const world);
	CUSTOM_JOINTS_API virtual ~CustomVehicleControllerManager();

	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	CUSTOM_JOINTS_API virtual void DestroyController (CustomVehicleController* const controller);

//	CUSTOM_JOINTS_API void DrawSchematic (const CustomVehicleController* const controller, dFloat scale) const;
	protected:
//	CUSTOM_JOINTS_API virtual void DrawSchematicCallback (const CustomVehicleController* const controller, const char* const partName, dFloat value, int pointCount, const dVector* const lines) const;
	
	friend class CustomVehicleController;
};


#endif 

