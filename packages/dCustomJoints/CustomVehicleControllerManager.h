/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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


class CustomVehicleController;

class CustomVehicleController: public CustomControllerBase
{
	public:
	class AxelJoint;
	class WheelJoint;
	class EngineJoint;
	class EngineController;
	class DifferentialJoint;
	class SteeringController;

	class Controller: public CustomAlloc
	{
		public:
		Controller(CustomVehicleController* const controller)
			:m_controller(controller)
			,m_param(0.0f)
			,m_paramMemory(0.0f)
			,m_timer(60)
		{
		}

		~Controller()
		{
		}

		void SetParam(dFloat param)
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
		virtual void ProjectError() {}
		
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
			,m_aerodynamicsDownForce0(0.0f)
			,m_aerodynamicsDownForce1(0.0f)
			,m_aerodynamicsDownSpeedCutOff(0.0f)
			,m_aerodynamicsDownForceCoefficient(0.0f)
		{
		}

		void Init(CustomVehicleController* const controller, void* const userData)
		{
			m_joint = NULL;
			m_userData = userData;
			m_controller = controller;
			m_body = controller->GetBody();
		}

		CUSTOM_JOINTS_API void ApplyDownForce ();

		dFloat m_aerodynamicsDownForce0;
		dFloat m_aerodynamicsDownForce1;
		dFloat m_aerodynamicsDownSpeedCutOff;
		dFloat m_aerodynamicsDownForceCoefficient;
	};

	class BodyPartEngine: public BodyPart
	{
		public:
		BodyPartEngine(CustomVehicleController* const controller, dFloat mass, dFloat amatureRadius);
		~BodyPartEngine();

		void ApplyTorque(dFloat torque);
		void ProjectError();
	};
	
	class BodyPartTire: public BodyPart
	{
		public:
		class Info
		{
			public:
			enum SuspensionType
			{
				m_offroad,
				m_confort,
				m_race,
				m_roller,
			};

			Info ()
			{
				memset (this, 0, sizeof (Info));
			}

			dVector m_location;
			dFloat m_mass;
			dFloat m_radio;
			dFloat m_width;
			dFloat m_maxSteeringAngle;
			dFloat m_dampingRatio;
			dFloat m_springStrength;
			dFloat m_suspesionlenght;
			dFloat m_lateralStiffness;
			dFloat m_longitudialStiffness;
			dFloat m_aligningMomentTrail;
			int m_hasFender;
			SuspensionType m_suspentionType;
			void* m_userData;
		};

		class FrictionModel
		{
			public:
			FrictionModel(const CustomVehicleController* const controller)
				:m_controller(controller)
			{
			}

			virtual dFloat GetFrictionCoefficient(const NewtonMaterial* const material, const NewtonBody* const tireBody, const NewtonBody* const otherBody) const
			{
				// the vehicle model is realistic, please do not use fake larger than one fiction coefficient, or you vehicle will simply role over
				return 1.0f;
			}

			virtual void GetForces(const BodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonMaterial* const material, dFloat tireLoad, dFloat& longitudinalForce, dFloat& lateralForce, dFloat& aligningTorque) const;

			const CustomVehicleController* m_controller;
		};

		CUSTOM_JOINTS_API BodyPartTire();
		CUSTOM_JOINTS_API ~BodyPartTire();

		CUSTOM_JOINTS_API dFloat GetRPM() const; 
		CUSTOM_JOINTS_API dFloat GetLateralSlip () const;
		CUSTOM_JOINTS_API dFloat GetLongitudinalSlip () const;

		CUSTOM_JOINTS_API Info GetInfo() const {return m_data;}
		CUSTOM_JOINTS_API void SetInfo(const Info& info) {};

		protected:
		void Init(BodyPart* const parentPart, const dMatrix& locationInGlobalSpase, const Info& info);

		void ProjectError();
		void SetBrakeTorque(dFloat torque);
		void SetSteerAngle(dFloat angleParam, dFloat timestep);

		Info m_data;
		dFloat m_lateralSlip;
		dFloat m_longitudinalSlip;
		dFloat m_aligningTorque;
		int m_index;
		int m_collidingCount;
		NewtonWorldConvexCastReturnInfo m_contactInfo[4];
		friend class WheelJoint;
		friend class CustomVehicleController;
		friend class CustomVehicleControllerManager;
	};

	class EngineController: public Controller
	{
		public:
		class DifferentialAxel
		{
			public:
			DifferentialAxel()
				:m_leftTire(NULL)
				,m_rightTire(NULL)
			{
			}

			BodyPartTire* m_leftTire;
			BodyPartTire* m_rightTire;
		};

		class Differential
		{
			public:
			enum type
			{
				m_2wd,
				m_4wd,
				m_8wd,
				m_track,
				m_undefined,
			};

			Differential()
				:m_axel()
				,m_type(m_undefined)
			{
			}

			DifferentialAxel m_axel;
			int m_type;
		};

		class Differential2wd : public Differential
		{
			public:
			Differential2wd()
				:Differential()
			{
				m_type = m_2wd;
			}
		};

		class Differential4wd: public Differential
		{
			public:
			Differential4wd()
				:Differential()
				,m_secondAxel()
			{
				m_type = m_4wd;
			}

			Differential m_secondAxel;
		};

		class Differential8wd: public Differential4wd
		{
			public:
			Differential8wd()
				:Differential4wd()
				,m_second4Wd()
			{
				m_type = m_4wd;
			}

			Differential4wd m_second4Wd;
		};

		class DifferentialTracked: public Differential
		{
			public:
			DifferentialTracked(int tiresCount, BodyPartTire** const leftTrack, BodyPartTire** const rightTrack)
				:Differential()
				,m_leftTrack(leftTrack)
				,m_rightTrack(rightTrack)
				,m_count (tiresCount)
			{
				m_type = m_track;
				m_axel.m_leftTire = leftTrack[0];
				m_axel.m_rightTire = rightTrack[0];
			}

			BodyPartTire** m_leftTrack;
			BodyPartTire** m_rightTrack;
			int m_count;
		};

		class Info
		{
			public:
			Info()
			{
				memset(this, 0, sizeof (Info));
			}

			dVector m_location;
			dFloat m_mass;
			dFloat m_radio;
			dFloat m_idleTorque;
			dFloat m_rpmAtIdleTorque;
			dFloat m_peakTorque;
			dFloat m_rpmAtPeakTorque;
			dFloat m_peakHorsePower;
			dFloat m_rpmAtPeakHorsePower;
			dFloat m_rpmAtRedLine;

			dFloat m_vehicleTopSpeed;
			dFloat m_reverseGearRatio;
			dFloat m_gearRatios[10];
			dFloat m_clutchFrictionTorque;

			dFloat m_aerodynamicDownforceFactor; 
			dFloat m_aerodynamicDownforceFactorAtTopSpeed; 
			dFloat m_aerodynamicDownForceSurfaceCoeficident;

			int m_gearsCount;
			int m_differentialLock;
			void* m_userData;

			private:
			void ConvertToMetricSystem();

			dFloat m_crownGearRatio;
			dFloat m_idleFriction;
			dFloat m_peakPowerTorque;
			dFloat m_viscousDrag0;
			dFloat m_viscousDrag1;
			dFloat m_viscousDrag2;
			friend class EngineController;
		};

		public:
		CUSTOM_JOINTS_API EngineController(CustomVehicleController* const controller, const Info& info, const Differential& differential);
		CUSTOM_JOINTS_API ~EngineController();

		CUSTOM_JOINTS_API void ApplyTorque(dFloat torque);

		CUSTOM_JOINTS_API Info GetInfo() const;
		CUSTOM_JOINTS_API void SetInfo(const Info& info);

		CUSTOM_JOINTS_API dFloat GetRPM() const;
		CUSTOM_JOINTS_API dFloat GetIdleRPM() const;
		CUSTOM_JOINTS_API dFloat GetRedLineRPM() const;
		CUSTOM_JOINTS_API dFloat GetSpeed() const;
		CUSTOM_JOINTS_API dFloat GetTopSpeed() const;

		CUSTOM_JOINTS_API int GetGear() const;
		CUSTOM_JOINTS_API void SetGear(int gear);

		CUSTOM_JOINTS_API void SetIgnition(bool key);
		CUSTOM_JOINTS_API bool GetIgnition() const;

		CUSTOM_JOINTS_API int GetFirstGear() const;
		CUSTOM_JOINTS_API int GetLastGear() const;
		CUSTOM_JOINTS_API int GetNeutralGear() const;
		CUSTOM_JOINTS_API int GetReverseGear() const;

		CUSTOM_JOINTS_API bool GetTransmissionMode() const;
		CUSTOM_JOINTS_API void SetTransmissionMode(bool mode);

		CUSTOM_JOINTS_API bool GetDifferentialLock () const;
		CUSTOM_JOINTS_API void SetDifferentialLock (bool mode);
		CUSTOM_JOINTS_API void SetClutchParam (dFloat cluthParam);

		CUSTOM_JOINTS_API void PlotEngineCurve () const;

		protected:
		dFloat GetTopGear() const;
		dFloat GetRadiansPerSecond() const;
		void InitEngineTorqueCurve();
		void CalculateCrownGear();
		dFloat GetGearRatio () const;
		virtual void Update(dFloat timestep);
		void UpdateAutomaticGearBox(dFloat timestep, dFloat omega);

		Info m_info;
		Info m_infoCopy;
		AxelJoint* m_leftAxel;
		AxelJoint* m_rightAxel;
		CustomVehicleController* m_controller;
		BodyPartTire* m_crownGearCalculator;
		dFloat m_clutchParam;
		int m_gearTimer;
		int m_currentGear;
		bool m_ignitionKey;
		bool m_automaticTransmissionMode;
		friend class CustomVehicleController;
	};

	class SteeringController: public Controller
	{
		public:
		CUSTOM_JOINTS_API SteeringController (CustomVehicleController* const controller);
		CUSTOM_JOINTS_API void AddTire (BodyPartTire* const tire);
		
		protected:
		virtual void Update(dFloat timestep);

		dList<BodyPartTire*> m_tires;
		bool m_isSleeping;
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
		dFloat m_maxTorque;
		friend class CustomVehicleController;
	};


	CUSTOM_JOINTS_API void Finalize();

	CUSTOM_JOINTS_API BodyPartTire* AddTire (const BodyPartTire::Info& tireInfo);
	CUSTOM_JOINTS_API BodyPartEngine* AddEngine (dFloat mass, dFloat armatureRadius);

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);
	const CUSTOM_JOINTS_API BodyPart* GetChassis() const;
//	const CUSTOM_JOINTS_API dMatrix& GetLocalFrame() const;

	CUSTOM_JOINTS_API dMatrix GetTransform() const;
	CUSTOM_JOINTS_API void SetTransform(const dMatrix& matrix);

	CUSTOM_JOINTS_API dList<BodyPartTire>::dListNode* GetFirstTire() const;
	CUSTOM_JOINTS_API dList<BodyPartTire>::dListNode* GetNextTire (dList<BodyPartTire>::dListNode* const tireNode) const;

	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetFirstBodyPart() const;
	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetNextBodyPart(dList<BodyPart*>::dListNode* const partNode) const;

	CUSTOM_JOINTS_API dVector GetTireNormalForce(const BodyPartTire* const tire) const;
	CUSTOM_JOINTS_API dVector GetTireLateralForce(const BodyPartTire* const tire) const;
	CUSTOM_JOINTS_API dVector GetTireLongitudinalForce(const BodyPartTire* const tire) const;

	CUSTOM_JOINTS_API BrakeController* GetBrakes() const;
	CUSTOM_JOINTS_API EngineController* GetEngine() const;
	CUSTOM_JOINTS_API BrakeController* GetHandBrakes() const;
	CUSTOM_JOINTS_API SteeringController* GetSteering() const;
	
	CUSTOM_JOINTS_API void SetEngine(EngineController* const engine);
	CUSTOM_JOINTS_API void SetBrakes(BrakeController* const brakes);
	CUSTOM_JOINTS_API void SetHandBrakes(BrakeController* const brakes);
	CUSTOM_JOINTS_API void SetSteering(SteeringController* const steering);
	CUSTOM_JOINTS_API void SetContactFilter(BodyPartTire::FrictionModel* const filter);

	CUSTOM_JOINTS_API dFloat GetAerodynamicsDowforceCoeficient() const;
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient(dFloat downWeightRatioAtSpeedFactor, dFloat speedFactor, dFloat maxWeightAtTopSpeed);

	CUSTOM_JOINTS_API dFloat GetWeightDistribution() const;
	CUSTOM_JOINTS_API void SetWeightDistribution(dFloat weightDistribution);

	CUSTOM_JOINTS_API void DrawSchematic (dFloat scale) const;	

	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	bool ControlStateChanged() const;
	void Init (NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag);
	void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag);
	
	void CalculateLateralDynamicState(dFloat timestep);
	void ApplySuspensionForces (dFloat timestep) const;
	dVector GetLastLateralForce(BodyPartTire* const tire) const;
	void Cleanup();
	
//	dMatrix m_localFrame;
	BodyPartChassis m_chassis;
	dList<BodyPartTire> m_tireList;
	dList<BodyPart*> m_bodyPartsList;
	BodyPartEngine* m_engine;
	
	void* m_collisionAggregate;
	
	BrakeController* m_brakesControl;
	EngineController* m_engineControl;
	BrakeController* m_handBrakesControl;
	SteeringController* m_steeringControl; 
	BodyPartTire::FrictionModel* m_contactFilter;
	NewtonApplyForceAndTorque m_forceAndTorque;

	dMatrix m_vehicleGlobalMatrix;
	dVector m_localOmega;
	dVector m_localVeloc;
	dFloat m_speed;
	dFloat m_totalMass;
	dFloat m_speedRate;
	dFloat m_sideSlipRate;
	dFloat m_sideSlipAngle;
	dFloat m_gravityMag;
	dFloat m_weightDistribution;

	bool m_finalized;

	friend class CustomVehicleControllerManager;
};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
	class TireFilter;
	public:
	CUSTOM_JOINTS_API CustomVehicleControllerManager(NewtonWorld* const world, int materialCount, int* const otherMaterials);
	CUSTOM_JOINTS_API virtual ~CustomVehicleControllerManager();

	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag);
	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData, dFloat gravityMag);
	CUSTOM_JOINTS_API virtual void DestroyController (CustomVehicleController* const controller);

	CUSTOM_JOINTS_API virtual int OnTireAABBOverlap(const NewtonMaterial* const material, const CustomVehicleController::BodyPartTire* const tire, const NewtonBody* const otherBody) const;

	CUSTOM_JOINTS_API int GetTireMaterial() const;
	CUSTOM_JOINTS_API void DrawSchematic (const CustomVehicleController* const controller, dFloat scale) const;

	protected:
	CUSTOM_JOINTS_API void OnTireContactsProcess (const NewtonJoint* const contactJoint, CustomVehicleController::BodyPartTire* const tire, const NewtonBody* const otherBody, dFloat timestep);
	CUSTOM_JOINTS_API virtual void DrawSchematicCallback (const CustomVehicleController* const controller, const char* const partName, dFloat value, int pointCount, const dVector* const lines) const;
	CUSTOM_JOINTS_API int OnContactGeneration (const CustomVehicleController::BodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonCollision* const othercollision, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex) const;

	int Collide (CustomVehicleController::BodyPartTire* const tire, int threadIndex) const;
	static void OnTireContactsProcess(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex);
	static int OnTireAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);
	static int OnContactGeneration (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);

	const void* m_tireShapeTemplateData;
	NewtonCollision* m_tireShapeTemplate;
	int m_tireMaterial;

	friend class CustomVehicleController;
};


#endif 

