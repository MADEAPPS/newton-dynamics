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

class CustomVehicleController: public CustomControllerBase
{
	public:
//	class dTireForceSolverSolver;
	class dWeightDistibutionSolver;
	
	class BodyPart
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
		class CreationInfo
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
		void Init (BodyPart* const parentPart, const dMatrix& locationInGlobaSpase, const CreationInfo& info);

		CreationInfo m_data;
	};

	class BodyPartDifferential2WD: public BodyPart
	{
		public:
		class DifferentialJoint;
		BodyPartDifferential2WD (BodyPart* const parentPart, const BodyPartTire* const leftTire, const BodyPartTire* const rightTire);
		~BodyPartDifferential2WD();
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
	CUSTOM_JOINTS_API CustomVehicleControllerComponentSteering* GetSteering() const;

	
	

	CUSTOM_JOINTS_API void SetBrakes(CustomVehicleControllerComponentBrake* const brakes);
	CUSTOM_JOINTS_API void SetEngine(CustomVehicleControllerComponentEngine* const engine);
	CUSTOM_JOINTS_API void SetHandBrakes(CustomVehicleControllerComponentBrake* const brakes);
	CUSTOM_JOINTS_API void SetSteering(CustomVehicleControllerComponentSteering* const steering);
	CUSTOM_JOINTS_API void SetContactFilter(CustomVehicleControllerTireCollisionFilter* const filter);

	CUSTOM_JOINTS_API void LinksTiresKinematically (int count, CustomVehicleControllerBodyStateTire** const tires);
	

	protected:
	
	CUSTOM_JOINTS_API CustomVehicleControllerBodyStateContact* GetContactBody (const NewtonBody* const body);
	
	
	
	CUSTOM_JOINTS_API bool IsSleeping();

	CUSTOM_JOINTS_API void DrawSchematic (dFloat scale) const;	
	
	CustomVehicleControllerBodyStateChassis m_chassisState;

	dList<CustomVehicleControllerBodyState*> m_stateList;
	dList<CustomVehicleControllerBodyStateTire> m_tireList;
	dList<CustomVehicleControllerEngineDifferencialJoint> m_tankTireLinks;
	dList<CustomVehicleControllerBodyStateContact> m_externalContactStatesPool;
	dList<CustomVehicleControllerBodyStateContact>::dListNode* m_freeContactList;
	
	CustomVehicleControllerComponentBrake* m_brakes;
	CustomVehicleControllerComponentEngine* m_engine;
	CustomVehicleControllerComponentBrake* m_handBrakes;
	CustomVehicleControllerComponentSteering* m_steering; 
	CustomVehicleControllerTireCollisionFilter* m_contactFilter;
	CustomVehicleControllerBodyStateContact* m_externalContactStates[16];
	int m_sleepCounter;
	int m_externalContactStatesCount;
	

	
	friend class CustomVehicleControllerTireJoint;
	
	friend class CustomVehicleControllerTireContactJoint;
	friend class CustomVehicleControllerEngineIdleJoint;
	friend class CustomVehicleControllerEngineDifferencialJoint;
	
	friend class CustomVehicleControllerBodyStateTire;
	friend class CustomVehicleControllerBodyStateChassis;
	friend class CustomVehicleControllerComponentBrake;
	friend class CustomVehicleControllerComponentEngine;
	friend class CustomVehicleControllerComponentSteering;
	friend class CustomVehicleControllerComponentTrackSkidSteering;
#endif


	CUSTOM_JOINTS_API void Finalize();

	CUSTOM_JOINTS_API void SetCenterOfGravity(const dVector& comRelativeToGeomtriCenter);
	CUSTOM_JOINTS_API BodyPartTire* AddTire (const BodyPartTire::CreationInfo& tireInfo);
	CUSTOM_JOINTS_API BodyPartDifferential2WD* AddDifferential2WD (const BodyPartTire* const leftNode, const BodyPartTire* const rightire);

	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetFirstPart() const;
	CUSTOM_JOINTS_API dList<BodyPart*>::dListNode* GetNextPart(dList<BodyPart*>::dListNode* const part) const;

	protected:
	CUSTOM_JOINTS_API virtual void PreUpdate(dFloat timestep, int threadIndex);
	CUSTOM_JOINTS_API virtual void PostUpdate(dFloat timestep, int threadIndex);

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
	BodyPartDifferential2WD* m_differential;

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

