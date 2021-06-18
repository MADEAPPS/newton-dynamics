/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __D_MULTIBODY_VEHICLE_H__
#define __D_MULTIBODY_VEHICLE_H__

#include "ndNewtonStdafx.h"
#include "ndModel.h"

class ndWorld;
class ndWheelDescriptor;
class ndMultiBodyVehicleMotor;
class ndMultiBodyVehicleGearBox;
class ndMultiBodyVehicleTireJoint;
class ndMultiBodyVehicleTorsionBar;
class ndMultiBodyVehicleDifferential;

#define dRadPerSecToRpm dFloat32(9.55f)

class ndMultiBodyVehicle: public ndModel
{
	public:
	class ndDownForce
	{
		public:
		class ndSpeedForcePair
		{
			public:
			dFloat32 m_speed;
			dFloat32 m_forceFactor;
			private:
			dFloat32 m_aerodynamicsConst;
			friend class ndDownForce;
		};

		D_NEWTON_API ndDownForce();
		D_NEWTON_API dFloat32 GetDownforceFactor(dFloat32 speed) const;
		
		ndSpeedForcePair m_downForceTable[3];
	};

	D_NEWTON_API ndMultiBodyVehicle(const dVector& frontDir, const dVector& upDir);
	D_NEWTON_API ndMultiBodyVehicle(const nd::TiXmlNode* const xmlNode);
	D_NEWTON_API virtual ~ndMultiBodyVehicle ();

	D_CLASS_RELECTION(ndMultiBodyVehicle);
	ndMultiBodyVehicle* GetAsMultiBodyVehicle();
	virtual dFloat32 GetFrictionCoeficient(const ndMultiBodyVehicleTireJoint* const, const ndContactMaterial&) const;

	D_NEWTON_API dFloat32 GetSpeed() const;
	D_NEWTON_API void SetSteeringAngle(dFloat32 angleInRadians);

	D_NEWTON_API ndShapeInstance CreateTireShape(dFloat32 radius, dFloat32 width) const;

	D_NEWTON_API void AddChassis(ndBodyDynamic* const chassis, dFloat32 gravityMag);
	D_NEWTON_API ndMultiBodyVehicleMotor* AddMotor(ndWorld* const world, dFloat32 mass, dFloat32 radius);
	D_NEWTON_API ndMultiBodyVehicleTireJoint* AddTire(ndWorld* const world, const ndWheelDescriptor& desc, ndBodyDynamic* const tire);
	D_NEWTON_API ndMultiBodyVehicleTireJoint* AddAxleTire(ndWorld* const world, const ndWheelDescriptor& desc, ndBodyDynamic* const tire, ndBodyDynamic* const axleBody);
	D_NEWTON_API ndMultiBodyVehicleGearBox* AddGearBox(ndWorld* const world, ndMultiBodyVehicleMotor* const motor, ndMultiBodyVehicleDifferential* const differential);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndWorld* const world, dFloat32 mass, dFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, dFloat32 slipOmegaLock);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndWorld* const world, dFloat32 mass, dFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, dFloat32 slipOmegaLock);
	D_NEWTON_API ndMultiBodyVehicleTorsionBar* AddTorsionBar(ndWorld* const world);

	D_NEWTON_API void SetAsBrake(ndMultiBodyVehicleTireJoint* const tire);
	D_NEWTON_API void SetAsHandBrake(ndMultiBodyVehicleTireJoint* const tire);
	D_NEWTON_API void SetAsSteering(ndMultiBodyVehicleTireJoint* const tire);

	private:
	void ApplySteering();
	void ApplyTiremodel();
	void ApplyAligmentAndBalancing();
	void BrushTireModel(const ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint) const;
	ndBodyDynamic* CreateInternalBodyPart(ndWorld* const world, dFloat32 mass, dFloat32 radius) const;

	protected:
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;
	D_NEWTON_API virtual void Update(ndWorld* const world, dFloat32 timestep);
	virtual void PostUpdate(ndWorld* const, dFloat32) {}

	dMatrix m_localFrame;
	ndBodyDynamic* m_chassis;
	ndMultiBodyVehicleMotor* m_motor;
	ndShapeChamferCylinder* m_tireShape;
	ndMultiBodyVehicleGearBox* m_gearBox;
	ndMultiBodyVehicleTorsionBar* m_torsionBar;
	dList<ndMultiBodyVehicleTireJoint*> m_tireList;
	dList<ndMultiBodyVehicleTireJoint*> m_brakeTires;
	dList<ndMultiBodyVehicleTireJoint*> m_handBrakeTires;
	dList<ndMultiBodyVehicleTireJoint*> m_steeringTires;
	dList<ndMultiBodyVehicleDifferential*> m_differentials;
	ndDownForce m_downForce;
	dFloat32 m_gravityMag;
	dFloat32 m_steeringAngle;
	dFloat32 m_steeringAngleMemory;
	
	friend class ndMultiBodyVehicleMotor;
	friend class ndMultiBodyVehicleGearBox;
	friend class ndMultiBodyVehicleTorsionBar;
};

inline dFloat32 ndMultiBodyVehicle::GetFrictionCoeficient(const ndMultiBodyVehicleTireJoint* const, const ndContactMaterial&) const
{
	return dFloat32(2.0f);
}

inline ndMultiBodyVehicle* ndMultiBodyVehicle::GetAsMultiBodyVehicle() 
{ 
	return this; 
}

#endif