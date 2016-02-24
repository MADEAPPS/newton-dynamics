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


class CustomVehicleController;



class CustomJoint;
class CustomVehicleController: public CustomControllerBase
{
	public:
	class WheelJoint;
	class EngineJoint;
	class dWeightDistibutionSolver;
	class DifferentialSpiderGearJoint;

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
			,m_aerodynamicsDownForceCoefficient(1.e-4f)
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

		dFloat m_aerodynamicsDownForceCoefficient;
	};
	

	class BodyPartTire: public BodyPart
	{
		public:
		class Info
		{
			public:
			dVector m_location;
			dFloat m_mass;
			dFloat m_radio;
			dFloat m_width;
			dFloat m_dampingRatio;
			dFloat m_springStrength;
			dFloat m_suspesionlenght;
			dFloat m_lateralStiffness;
			dFloat m_longitudialStiffness;
			dFloat m_aligningMomentTrail;
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
				//return 1.5f;
				return 1.0f;
			}

			// Using brush tire model explained on this paper
			// https://ddl.stanford.edu/sites/default/files/2013_Thesis_Hindiyeh_Dynamics_and_Control_of_Drifting_in_Automobiles.pdf
			// 
			virtual void GetForces(const BodyPartTire* const tire, const NewtonBody* const otherBody, const NewtonMaterial* const material, dFloat tireLoad, dFloat longitudinalSlip, dFloat lateralSlip, dFloat& longitudinalForce, dFloat& lateralForce, dFloat& aligningTorque) const
			{
				tireLoad *= GetFrictionCoefficient (material, tire->GetBody(), otherBody);
				dFloat phy_z = lateralSlip * tire->m_data.m_lateralStiffness;
				dFloat phy_x = longitudinalSlip * tire->m_data.m_longitudialStiffness;
				dFloat gamma = dSqrt(phy_x * phy_x + phy_z * phy_z);
				dFloat phyMax = 3.0f * tireLoad + 1.0f;

				dFloat F = (gamma <= phyMax) ? (gamma * (1.0f - gamma / phyMax  + gamma * gamma / (3.0f * phyMax * phyMax))) : tireLoad;

				dFloat fraction = F / gamma;
				dFloat F_x = phy_x * fraction;
				dFloat F_z = phy_z * fraction;

				lateralForce = - F_z;
				longitudinalForce = F_x;

				aligningTorque = 0.0f;
			}

			const CustomVehicleController* m_controller;
		};

		CUSTOM_JOINTS_API BodyPartTire();
		CUSTOM_JOINTS_API ~BodyPartTire();

		CUSTOM_JOINTS_API void Init (BodyPart* const parentPart, const dMatrix& locationInGlobalSpase, const Info& info);

		CUSTOM_JOINTS_API void SetSteerAngle (dFloat angle);
		CUSTOM_JOINTS_API void SetBrakeTorque (dFloat torque);

		CUSTOM_JOINTS_API dFloat GetRPM() const; 

		// TODO
		CUSTOM_JOINTS_API Info GetInfo() const {return m_data;}
		CUSTOM_JOINTS_API void SetInfo(const Info& info) {};


		protected:
		Info m_data;
		dFloat m_lateralSlip;
		dFloat m_longitudinalSlip;
		dFloat m_aligningTorque;
		int m_index;
		friend class WheelJoint;
		friend class CustomVehicleController;
		friend class CustomVehicleControllerManager;
	};

	class BodyPartEngine: public BodyPart
	{
		public:
		class DifferentialAxel
		{
			public:
			DifferentialAxel ()
				:m_leftTire (NULL)
				,m_rightTire (NULL)
			{
			}

			BodyPartTire* m_leftTire;
			BodyPartTire* m_rightTire;
		};

		class DifferentialGear: public DifferentialAxel
		{
			public:
			DifferentialGear(const DifferentialAxel& axel)
				:DifferentialAxel (axel)
				,m_leftGear(NULL)
				,m_rightGear(NULL)
			{
			}
			DifferentialSpiderGearJoint* m_leftGear;
			DifferentialSpiderGearJoint* m_rightGear;
		};

		class Info
		{
			public:
			Info()
			{
				memset (this, 0, sizeof (Info));
			}

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
			dFloat m_reverseGearRatio;
			dFloat m_gearRatios[10];
			int m_gearsCount;
			void* m_userData;

			private:
			void ConvertToMetricSystem();

			dFloat m_crownGearRatio;
			dFloat m_peakPowerTorque;
			dFloat m_engineIdleDryDrag;
			dFloat m_engineIdleViscousDrag;
			friend class BodyPartEngine;
		};

		CUSTOM_JOINTS_API BodyPartEngine(CustomVehicleController* const controller, const Info& info, const DifferentialAxel& axel0, const DifferentialAxel& axel1);
		CUSTOM_JOINTS_API virtual ~BodyPartEngine();

		CUSTOM_JOINTS_API void ApplyTorque(dFloat torque);
		CUSTOM_JOINTS_API virtual void Update (dFloat timestep, dFloat gasVal);

		CUSTOM_JOINTS_API dFloat GetRPM() const;
		CUSTOM_JOINTS_API dFloat GetSpeed() const;
		CUSTOM_JOINTS_API dFloat GetRedLineRPM() const;

		CUSTOM_JOINTS_API int GetGear() const;
		CUSTOM_JOINTS_API void SetGear(int gear);

		CUSTOM_JOINTS_API int GetNeutralGear() const;
		CUSTOM_JOINTS_API int GetReverseGear() const;

		CUSTOM_JOINTS_API void UpdateAutomaticGearBox(dFloat timestep);

		CUSTOM_JOINTS_API Info GetInfo () const;
		CUSTOM_JOINTS_API void SetInfo (const Info& info);

		protected:
		dFloat GetTopGear() const;
		void SetTopSpeed();
		void InitEngineTorqueCurve();

		Info m_info;
		Info m_infoCopy;
		DifferentialGear m_differential0;
		DifferentialGear m_differential1;
		dInterpolationCurve m_torqueRPMCurve;
		dFloat m_norminalTorque;
		int m_currentGear;
		int m_gearTimer;

		friend class ClutchController;
		friend class CustomVehicleController;
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
		dFloat m_maxTorque;
		friend class CustomVehicleController;
	};

	class ClutchController: public Controller
	{
		public:
		CUSTOM_JOINTS_API ClutchController(CustomVehicleController* const controller, BodyPartEngine* const engine, dFloat maxClutchTorque);
		
		protected:
		virtual void Update(dFloat timestep);

		BodyPartEngine* m_engine;
		dFloat m_maxTorque;
		friend class CustomVehicleController;
	};

	class EngineController: public Controller
	{
		public:
		CUSTOM_JOINTS_API EngineController (CustomVehicleController* const controller, BodyPartEngine* const engine);

		CUSTOM_JOINTS_API dFloat GetRPM() const;
		CUSTOM_JOINTS_API dFloat GetRedLineRPM() const;
		CUSTOM_JOINTS_API dFloat GetSpeed() const;

		CUSTOM_JOINTS_API int GetGear() const;
		CUSTOM_JOINTS_API void SetGear(int gear);

		CUSTOM_JOINTS_API int GetNeutralGear() const;
		CUSTOM_JOINTS_API int GetReverseGear() const;

		CUSTOM_JOINTS_API bool GetTransmissionMode() const;
		CUSTOM_JOINTS_API void SetTransmissionMode(bool mode);

		protected:
		virtual void Update(dFloat timestep);

		BodyPartEngine* m_engine;
		bool m_automaticTransmissionMode;
		friend class CustomVehicleController;
	};

#if 0
	CUSTOM_JOINTS_API void SetDryRollingFrictionTorque (dFloat torque);
	CUSTOM_JOINTS_API dFloat GetDryRollingFrictionTorque () const;
	
#endif

	CUSTOM_JOINTS_API void Finalize();

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);
	CUSTOM_JOINTS_API BodyPartTire* AddTire (const BodyPartTire::Info& tireInfo);

	CUSTOM_JOINTS_API BodyPartEngine* GetEngineBodyPart() const;
	CUSTOM_JOINTS_API void AddEngineBodyPart (BodyPartEngine* const engine);

	const CUSTOM_JOINTS_API BodyPart* GetChassis() const;
	const CUSTOM_JOINTS_API dMatrix& GetLocalFrame() const;

	CUSTOM_JOINTS_API dMatrix GetTransform() const;
	CUSTOM_JOINTS_API void SetTransform(const dMatrix& matrix);

	CUSTOM_JOINTS_API dList<BodyPartTire>::dListNode* GetFirstTire() const;
	CUSTOM_JOINTS_API dList<BodyPartTire>::dListNode* GetNextTire (dList<BodyPartTire>::dListNode* const tireNode) const;

	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetFirstBodyPart() const;
	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetNextBodyPart(dList<BodyPart*>::dListNode* const partNode) const;

	CUSTOM_JOINTS_API void LinksTiresKinematically (int count, BodyPartTire** const tires);

	CUSTOM_JOINTS_API dVector GetTireNormalForce(const BodyPartTire* const tire) const;
	CUSTOM_JOINTS_API dVector GetTireLateralForce(const BodyPartTire* const tire) const;
	CUSTOM_JOINTS_API dVector GetTireLongitudinalForce(const BodyPartTire* const tire) const;

	CUSTOM_JOINTS_API ClutchController* GetClutch() const;
	CUSTOM_JOINTS_API BrakeController* GetBrakes() const;
	CUSTOM_JOINTS_API EngineController* GetEngine() const;
	CUSTOM_JOINTS_API BrakeController* GetHandBrakes() const;
	CUSTOM_JOINTS_API SteeringController* GetSteering() const;
	
	CUSTOM_JOINTS_API void SetEngine(EngineController* const engine);
	CUSTOM_JOINTS_API void SetClutch(ClutchController* const cluth);
	CUSTOM_JOINTS_API void SetBrakes(BrakeController* const brakes);
	CUSTOM_JOINTS_API void SetHandBrakes(BrakeController* const brakes);
	CUSTOM_JOINTS_API void SetSteering(SteeringController* const steering);
	CUSTOM_JOINTS_API void SetContactFilter(BodyPartTire::FrictionModel* const filter);

	CUSTOM_JOINTS_API dFloat GetAerodynamicsDowforceCoeficient() const;
	CUSTOM_JOINTS_API void SetAerodynamicsDownforceCoefficient(dFloat maxDownforceInGravities, dFloat topSpeed);

	CUSTOM_JOINTS_API void DrawSchematic (dFloat scale) const;	

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
	
	BodyPartEngine* m_engine;
	ClutchController* m_cluthControl;
	BrakeController* m_brakesControl;
	EngineController* m_engineControl;
	BrakeController* m_handBrakesControl;
	SteeringController* m_steeringControl; 
	BodyPartTire::FrictionModel* m_contactFilter;
	NewtonApplyForceAndTorque m_forceAndTorque;
	bool m_finalized;

	friend class CustomVehicleControllerManager;
};


class CustomVehicleControllerManager: public CustomControllerManager<CustomVehicleController> 
{
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

	static void OnTireContactsProcess(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex);
	static int OnTireAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);
	static int OnContactGeneration (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex);

	const void* m_tireShapeTemplateData;
	NewtonCollision* m_tireShapeTemplate;
	int m_tireMaterial;

	friend class CustomVehicleController;
};


#endif 

