/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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

class ndMultiBodyVehicleTireJoint: public ndJointWheel
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleTireJoint);
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndLoadSaveBase::ndLoadDescriptor& desc);
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndWheelDescriptor& desc, ndMultiBodyVehicle* const vehicle);
	D_NEWTON_API virtual ~ndMultiBodyVehicleTireJoint();

	D_NEWTON_API ndFloat32 GetSideSlip() const;
	D_NEWTON_API ndFloat32 GetLongitudinalSlip() const;

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	ndMultiBodyVehicle* m_vehicle;
	ndFloat32 m_lateralSlip;
	ndFloat32 m_longitudinalSlip;

	friend class ndMultiBodyVehicle;
};


#endif 

