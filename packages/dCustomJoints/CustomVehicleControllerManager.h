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


class CustomVehicleController;

class CustomVehicleController: public CustomControllerBase
{
	public:
	class WheelJoint;
//	class dWeightDistibutionSolver;

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

		void InitalizeCurve(int points, const dFloat* const steps, const dFloat* const values);
		dFloat GetValue(dFloat param) const;
		const dNot& GetValue(int entry) const {return m_nodes[entry];}

		dNot m_nodes[6];
		int m_count;
	};

	class Controller: public CustomAlloc
	{
		public:
		Controller(CustomVehicleController* const controller)
			:m_controller(controller)
			, m_param(0.0f)
			, m_paramMemory(0.0f)
			, m_timer(60)
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

			// Using brush tire model explained on this paper
			// https://ddl.stanford.edu/sites/default/files/2013_Thesis_Hindiyeh_Dynamics_and_Control_of_Drifting_in_Automobiles.pdf
			// 
			virtual void GetForces(const BodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonMaterial* const material, dFloat tireLoad, dFloat longitudinalSlip, dFloat lateralSlip, dFloat& longitudinalForce, dFloat& lateralForce, dFloat& aligningTorque) const
			{
				dFloat phy_y = lateralSlip * tire->m_data.m_lateralStiffness;
				dFloat phy_x = longitudinalSlip * tire->m_data.m_longitudialStiffness;
				dFloat gamma = dSqrt(phy_x * phy_x + phy_y * phy_y);

				dFloat fritionCoeficicent = dClamp (GetFrictionCoefficient (material, tire->GetBody(), otherBody), dFloat (0.0f), dFloat (1.0f));
				tireLoad *= fritionCoeficicent;
				dFloat phyMax = 3.0f * tireLoad + 1.0f;

				dFloat F = (gamma <= phyMax) ? (gamma * (1.0f - gamma / phyMax  + gamma * gamma / (3.0f * phyMax * phyMax))) : tireLoad;

				dFloat fraction = F / gamma;
				lateralForce = - phy_y * fraction;
				longitudinalForce = - phy_x * fraction;

				aligningTorque = 0.0f;
			}

			const CustomVehicleController* m_controller;
		};

		CUSTOM_JOINTS_API BodyPartTire();
		CUSTOM_JOINTS_API ~BodyPartTire();

		CUSTOM_JOINTS_API void Init (BodyPart* const parentPart, const dMatrix& locationInGlobalSpase, const Info& info);

		CUSTOM_JOINTS_API void SetSteerAngle (dFloat angleParam);
		CUSTOM_JOINTS_API void SetBrakeTorque (dFloat torque);

		CUSTOM_JOINTS_API dFloat GetRPM() const; 

		CUSTOM_JOINTS_API Info GetInfo() const {return m_data;}
		CUSTOM_JOINTS_API void SetInfo(const Info& info) {};

		protected:
		Info m_data;
		dFloat m_lateralSlip;
		dFloat m_longitudinalSlip;
		dFloat m_aligningTorque;
		dFloat m_driveTorque;
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
			};

			Differential()
				:m_axel()
				,m_type(m_2wd)
			{
			}

			DifferentialAxel m_axel;
			int m_type;
		};

		class Differential4wd: public Differential
		{
			public:
			Differential4wd()
				:Differential()
				,m_secundAxel()
			{
				m_type = m_4wd;
			}

			Differential m_secundAxel;
		};


		class Differential8wd: public Differential4wd
		{
			public:
			Differential8wd()
				:Differential4wd()
				,m_secund4Wd()
			{
				m_type = m_4wd;
			}

			Differential4wd m_secund4Wd;
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
			dFloat m_idleTorqueRpm;
			dFloat m_peakTorque;
			dFloat m_peakTorqueRpm;
			dFloat m_peakHorsePower;
			dFloat m_peakHorsePowerRpm;
			dFloat m_readLineRpm;

			dFloat m_vehicleTopSpeed;
			dFloat m_reverseGearRatio;
			dFloat m_gearRatios[10];
			dFloat m_clutchFrictionTorque;

			int m_gearsCount;
			int m_differentialLock;
			void* m_userData;
	
			private:
			void ConvertToMetricSystem();

			dFloat m_crownGearRatio;
			dFloat m_idleFriction;
			dFloat m_peakPowerTorque;
			dFloat m_readLineTorque;
			dFloat m_idleViscousDrag2;
			dFloat m_driveViscousDrag2;

			friend class EngineController;
		};

		private:
		class DriveTrainTire;
		class DriveTrainEngine;
		class DriveTrainSlipDifferential;

		class DriveTrain: public CustomAlloc
		{
			public:
			DriveTrain(const dVector& invInertia, DriveTrain* const parent = NULL);
			virtual ~DriveTrain();

			virtual DriveTrainTire* CastAsTire() {return NULL;}
			virtual DriveTrainEngine* CastAsEngine() {return NULL;}
			virtual DriveTrainSlipDifferential* CastAsSlipDifferential() {return NULL;}
			
			virtual void SetPartMasses (const dVector& invInertia);
			virtual void SetExternalTorque(EngineController* const controller);
			virtual void Integrate(EngineController* const controller, dFloat timestep);
			virtual void SetGearRatioJacobian(dFloat gearRatio) {};
			virtual void ApplyInternalTorque(EngineController* const controller, dFloat timestep, dFloat* const lambda);
			virtual void CalculateRightSide (EngineController* const controller, dFloat timestep, dFloat* const rightSide, dFloat* const low, dFloat* const high);
			virtual void ApplyTireTorque(EngineController* const controller) {}
			
			void SetInvMassJt();
			void BuildMassMatrix (dFloat* const massMatrix);
			void ReconstructInvMassJt ();
			void SetDifferentialJacobian (dFloat gearGain);

			void GetRow(dVector* const row) const;
			void GetInvRowT(dVector* const row) const;
			
			int GetNodeArray(DriveTrain** const array);
			int GetNodeArray(DriveTrain** const array, int& index);
			
			dVector m_J01;
			dVector m_J10;
			dVector m_invMassJt01;
			dVector m_invMassJt10;
			dVector m_omega;
			dVector m_torque;
			dVector m_inertiaInv;
			DriveTrain* m_parent;
			DriveTrain* m_child;
			DriveTrain* m_sibling;
			int m_index;
			int m_sortKey;
		};

		class DriveTrainEngineFriction: public DriveTrain
		{
			public:
			DriveTrainEngineFriction(DriveTrain* const parent);

			virtual void SetPartMasses (const dVector& invInertia);
			virtual void CalculateRightSide(EngineController* const controller, dFloat timestep, dFloat* const rightSide, dFloat* const low, dFloat* const high);

			dFloat m_friction;
		};

		class DriveTrainEngine: public DriveTrain
		{
			public:
			DriveTrainEngine (const dVector& invInertia);

			virtual DriveTrainEngine* CastAsEngine() {return this;}
			void SetGearRatio (dFloat gearRatio);
			void SetGearRatioJacobian(dFloat gearRatio);
			void SetExternalTorque(EngineController* const controller);
			void RebuildEngine (const dVector& invInertia);
			dFloat GetClutchTorque(EngineController* const controller) const;
			void Update(EngineController* const controller, dFloat engineTorque, dFloat timestep);

			void SetFriction(dFloat friction);

			dFloat m_gearSign;
			dFloat m_engineTorque;
			DriveTrainEngineFriction* m_internalFiction;
		};

		class DriveTrainEngine2W: public DriveTrainEngine
		{
			public:
			DriveTrainEngine2W (const dVector& invInertia, const DifferentialAxel& axel);
		};

		class DriveTrainEngine4W: public DriveTrainEngine
		{
			public:
			DriveTrainEngine4W (const dVector& invInertia, const DifferentialAxel& axel0, const DifferentialAxel& axel1);
		};

		class DriveTrainEngine8W: public DriveTrainEngine
		{
			public:
			DriveTrainEngine8W(const dVector& invInertia, const DifferentialAxel& axel0, const DifferentialAxel& axel1, const DifferentialAxel& axel2, const DifferentialAxel& axel3);
		};

		class DriveTrainEngineTracked: public DriveTrainEngine2W
		{
			public:
			DriveTrainEngineTracked (const dVector& invInertia, const DifferentialTracked& axel);
		};


		class DriveTrainDifferentialGear: public DriveTrain
		{
			public:
			DriveTrainDifferentialGear (const dVector& invInertia, DriveTrain* const parent, const DifferentialAxel& axel, dFloat side);
			DriveTrainDifferentialGear (const dVector& invInertia, DriveTrain* const parent, const DifferentialAxel& axel0, const DifferentialAxel& axel1, dFloat side);
			DriveTrainDifferentialGear (const dVector& invInertia, DriveTrain* const parent, const DifferentialAxel& axel0, const DifferentialAxel& axel1, const DifferentialAxel& axel2, const DifferentialAxel& axel3, dFloat side);
			
			virtual void SetGearRatioJacobian(dFloat gearGain);
			void Integrate(EngineController* const controller, dFloat timestep);
		};

		class DriveTrainSlipDifferential: public DriveTrain
		{
			public:
			DriveTrainSlipDifferential (DriveTrain* const parent);

			DriveTrainSlipDifferential* CastAsSlipDifferential() {return this;}
			virtual void SetPartMasses (const dVector& invInertia);
			virtual void CalculateRightSide (EngineController* const controller, dFloat timestep, dFloat* const rightSide, dFloat* const low, dFloat* const high);
		};

		class DriveTrainTire: public DriveTrain
		{
			public:
			DriveTrainTire (BodyPartTire* const tire, DriveTrain* const parent);

			virtual DriveTrainTire* CastAsTire() {return this;}
			virtual void SetExternalTorque(EngineController* const controller);
			virtual void SetPartMasses (const dVector& invInertia);
			virtual void Integrate(EngineController* const controller, dFloat timestep) {};
			virtual void ApplyTireTorque(EngineController* const controller);
			virtual void ApplyInternalTorque(EngineController* const controller, dFloat timestep, dFloat* const lambda);
			
			dVector m_reactionTorque;
			BodyPartTire* m_tire;
		};

		class DriveTrainSlaveTire: public DriveTrainTire
		{
			public:
			DriveTrainSlaveTire (BodyPartTire* const tire, DriveTrainTire* const parent);
			virtual void ApplyInternalTorque(EngineController* const controller, dFloat timestep, dFloat* const lambda);
			virtual void CalculateRightSide (EngineController* const controller, dFloat timestep, dFloat* const rightSide, dFloat* const low, dFloat* const high);
		};

		public:
		CUSTOM_JOINTS_API EngineController(CustomVehicleController* const controller, const Info& info, const Differential& differential);
		CUSTOM_JOINTS_API ~EngineController();

		CUSTOM_JOINTS_API void ApplyTorque(dFloat torque, dFloat timestep);

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
		void InitEngineTorqueCurve();
		void CalculateCrownGear();
		dFloat GetGearRatio () const;
		virtual void Update(dFloat timestep);
		void UpdateAutomaticGearBox(dFloat timestep);

		Info m_info;
		Info m_infoCopy;
		DriveTrainEngine* m_engine;
		dInterpolationCurve m_torqueRPMCurve;

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

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);
	CUSTOM_JOINTS_API BodyPartTire* AddTire (const BodyPartTire::Info& tireInfo);

	const CUSTOM_JOINTS_API BodyPart* GetChassis() const;
	const CUSTOM_JOINTS_API dMatrix& GetLocalFrame() const;

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
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient(dFloat maxDownforceInGravity, dFloat downWeightRatioAtSpeedFactor, dFloat speedFactor, dFloat maxWeightAtTopSpeed);

	CUSTOM_JOINTS_API dFloat GetWeightDistribution() const;
	CUSTOM_JOINTS_API void SetWeightDistribution(dFloat weightDistribution);

	CUSTOM_JOINTS_API void DrawSchematic (dFloat scale) const;	

	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

	bool ControlStateChanged() const;
	void Init (NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	void Init (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	

	void ApplyLateralStabilityForces(dFloat timestep);
	dVector GetLastLateralForce(BodyPartTire* const tire) const;
	void Cleanup();
	
	dMatrix m_localFrame;
	dVector m_lastAngularMomentum;
	BodyPartChassis m_chassis;
	dList<BodyPartTire> m_tireList;
	dList<BodyPart*> m_bodyPartsList;
	
	NewtonSkeletonContainer* m_skeleton;
	void* m_collisionAggregate;
	
	BrakeController* m_brakesControl;
	EngineController* m_engineControl;
	BrakeController* m_handBrakesControl;
	SteeringController* m_steeringControl; 
	BodyPartTire::FrictionModel* m_contactFilter;
	NewtonApplyForceAndTorque m_forceAndTorque;
	dFloat m_sideSlipAngle;
	dFloat m_weightDistribution;
	int m_tiresInContacts;
	bool m_finalized;
	bool m_isAirborned;
	bool m_hasNewContact;

	friend class CustomVehicleControllerManager;
};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
	class TireFilter;
	public:
	CUSTOM_JOINTS_API CustomVehicleControllerManager(NewtonWorld* const world, int materialCount, int* const otherMaterials);
	CUSTOM_JOINTS_API virtual ~CustomVehicleControllerManager();

	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonBody* const body, const dMatrix& vehicleFrame, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
	CUSTOM_JOINTS_API virtual CustomVehicleController* CreateVehicle (NewtonCollision* const chassisShape, const dMatrix& vehicleFrame, dFloat mass, NewtonApplyForceAndTorque forceAndTorque, void* const userData);
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

