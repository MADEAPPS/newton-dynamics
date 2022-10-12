/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __ND_MULTIBODY_VEHICLE_TIRE_JOINT_H__
#define __ND_MULTIBODY_VEHICLE_TIRE_JOINT_H__

#include "ndNewtonStdafx.h"
#include "ndJointWheel.h"

class ndTireFrictionModel
{
	public:
	enum ndFrictionModel
	{
		m_coulomb,
		m_brushModel,
		m_pacejka,
		m_coulombCicleOfFriction,
	};

	ndTireFrictionModel()
		:m_laterialStiffness(ndFloat32(1.0e3f))
		,m_longitudinalStiffness(ndFloat32(5.0e4f))
		,m_frictionModel(m_coulomb)
	{
	}
	ndFloat32 m_laterialStiffness;
	ndFloat32 m_longitudinalStiffness;
	ndFrictionModel m_frictionModel;
};

class ndMultiBodyVehicleTireJointInfo : public ndWheelDescriptor, public ndTireFrictionModel
{
	public:
	ndMultiBodyVehicleTireJointInfo()
		:ndWheelDescriptor()
		,ndTireFrictionModel()
	{
	}
};

class ndMultiBodyVehicleTireJoint: public ndJointWheel
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleTireJoint);
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndMultiBodyVehicleTireJointInfo& desc, ndMultiBodyVehicle* const vehicle);
	D_NEWTON_API virtual ~ndMultiBodyVehicleTireJoint();

	D_NEWTON_API ndFloat32 GetSideSlip() const;
	D_NEWTON_API ndFloat32 GetLongitudinalSlip() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndMultiBodyVehicle* m_vehicle;
	ndTireFrictionModel m_frictionModel;
	ndFloat32 m_lateralSlip;
	ndFloat32 m_longitudinalSlip;
	friend class ndMultiBodyVehicle;
};


#endif 

