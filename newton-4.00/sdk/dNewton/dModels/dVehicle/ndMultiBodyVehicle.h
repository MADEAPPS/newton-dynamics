/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_MULTIBODY_VEHICLE_H__
#define __ND_MULTIBODY_VEHICLE_H__

#include "ndNewtonStdafx.h"
#include "dModels/ndModel.h"
#include "dIkSolver/ndIkSolver.h"
#include "dModels/ndModelArticulation.h"

class ndWorld;
class ndBodyDynamic;
class ndMultiBodyVehicleMotor;
class ndMultiBodyVehicleGearBox;
class ndMultiBodyVehicleTireJoint;
class ndMultiBodyVehicleTorsionBar;
class ndMultiBodyVehicleDifferential;
class ndMultiBodyVehicleTireJointInfo;
class ndMultiBodyVehicleDifferentialAxle;

#define dRadPerSecToRpm ndFloat32(9.55f)

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicle : public ndModelArticulation
{
	public:
	class ndTireContactPair
	{
		public:
		ndContact* m_contact;
		ndMultiBodyVehicleTireJoint* m_tireJoint;
	};
	
	class ndDownForce
	{
		public:
		class ndSpeedForcePair
		{
			public:
			ndFloat32 m_speed;
			ndFloat32 m_forceFactor;
			ndFloat32 m_aerodynamicDownforceConstant;
			friend class ndDownForce;
		};
	
		ndDownForce();
		ndFloat32 GetDownforceFactor(ndFloat32 speed) const;
	
		private:
		ndFloat32 CalculateFactor(const ndSpeedForcePair* const entry) const;
	
		ndFloat32 m_gravity;
		ndFloat32 m_suspensionStiffnessModifier;
		ndSpeedForcePair m_downForceTable[5];
		friend class ndMultiBodyVehicle;
		friend class ndMultiBodyVehicleTireJoint;
	};

	D_CLASS_REFLECTION(ndMultiBodyVehicle, ndModelArticulation)

	D_NEWTON_API ndMultiBodyVehicle();
	D_NEWTON_API virtual ~ndMultiBodyVehicle ();

	D_NEWTON_API const ndMatrix& GetLocalFrame() const;
	D_NEWTON_API void SetLocalFrame(const ndMatrix& localframe);

	D_NEWTON_API ndMultiBodyVehicle* GetAsMultiBodyVehicle();

	D_NEWTON_API ndBodyDynamic* GetChassis() const;
	D_NEWTON_API ndMultiBodyVehicleMotor* GetMotor() const;
	D_NEWTON_API ndMultiBodyVehicleGearBox* GetGearBox() const;
	
	D_NEWTON_API const ndList<ndMultiBodyVehicleTireJoint*>& GetTireList() const;

	D_NEWTON_API bool IsSleeping() const;
	D_NEWTON_API ndFloat32 GetSpeed() const;
	D_NEWTON_API void AddChassis(const ndSharedPtr<ndBody>& chassis);
	D_NEWTON_API ndMultiBodyVehicleMotor* AddMotor(ndFloat32 mass, ndFloat32 radius);
	D_NEWTON_API ndShapeInstance CreateTireShape(ndFloat32 radius, ndFloat32 width) const;
	D_NEWTON_API ndMultiBodyVehicleGearBox* AddGearBox(ndMultiBodyVehicleDifferential* const differential);

	D_NEWTON_API void AddTire(const ndSharedPtr<ndBody>& tireBody, const ndSharedPtr<ndJointBilateralConstraint>& tireJoint);
	D_NEWTON_API void AddMotor(const ndSharedPtr<ndBody>& motorBody, const ndSharedPtr<ndJointBilateralConstraint>& motorJoint);
	D_NEWTON_API void AddDifferential(const ndSharedPtr<ndBody>& differentialBody, const ndSharedPtr<ndJointBilateralConstraint>& differentialJoint);

	D_NEWTON_API void AddGearBox(const ndSharedPtr<ndJointBilateralConstraint>& gearBoxJoint);
	D_NEWTON_API void AddDifferentialAxle(const ndSharedPtr<ndJointBilateralConstraint>& differentialAxleJoint);

	D_NEWTON_API ndMultiBodyVehicleTireJoint* AddTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire);
	D_NEWTON_API ndMultiBodyVehicleTireJoint* AddAxleTire(const ndMultiBodyVehicleTireJointInfo& desc, const ndSharedPtr<ndBody>& tire, const ndSharedPtr<ndBody>& axleBody);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleTireJoint* const leftTire, ndMultiBodyVehicleTireJoint* const rightTire, ndFloat32 slipOmegaLock);
	D_NEWTON_API ndMultiBodyVehicleDifferential* AddDifferential(ndFloat32 mass, ndFloat32 radius, ndMultiBodyVehicleDifferential* const leftDifferential, ndMultiBodyVehicleDifferential* const rightDifferential, ndFloat32 slipOmegaLock);

	D_NEWTON_API virtual void Update(ndFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndFloat32 timestep);
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;

#if 0
	D_NEWTON_API ndMultiBodyVehicleTorsionBar* AddTorsionBar(ndBodyKinematic* const sentinel);
	D_NEWTON_API ndMultiBodyVehicle* GetAsMultiBodyVehicle();
#endif

	//D_NEWTON_API void SetVehicleSolverModel(bool hardJoint);

	private:
	void ApplyStabilityControl();
	void ApplyAlignmentAndBalancing();
	void ApplyTireModel(ndFloat32 timestep);
	void ApplyAerodynamics(ndFloat32 timestep);
	ndBodyKinematic* CreateInternalBodyPart(ndFloat32 mass, ndFloat32 radius) const;
	void ApplyTireModel(ndFloat32 timestep, ndFixSizeArray<ndTireContactPair, 128>& tireContacts);

	void CalculateNormalizedAlgniningTorque(ndMultiBodyVehicleTireJoint* const tire, ndFloat32 sideSlipTangent) const;
	void BrushTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const;
	void CoulombTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const;
	void PacejkaTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const;
	void CoulombFrictionCircleTireModel(ndMultiBodyVehicleTireJoint* const tire, ndContactMaterial& contactPoint, ndFloat32 timestep) const;

	ndMatrix m_localFrame;
	ndBodyDynamic* m_chassis;
	ndMultiBodyVehicleMotor* m_motor;
	ndShapeChamferCylinder* m_tireShape;
	ndMultiBodyVehicleGearBox* m_gearBox;
	ndMultiBodyVehicleTorsionBar* m_torsionBar;
	ndList<ndMultiBodyVehicleTireJoint*> m_tireList;
	ndList<ndMultiBodyVehicleDifferential*> m_differentialList;

	//ndIkSolver m_dynamicSolver;
	ndDownForce m_downForce;
	ndFloat32 m_steeringRate;
	ndFloat32 m_maxSideslipRate;
	ndFloat32 m_maxSideslipAngle;

	friend class ndMultiBodyVehicleMotor;
	friend class ndMultiBodyVehicleTireJoint;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif