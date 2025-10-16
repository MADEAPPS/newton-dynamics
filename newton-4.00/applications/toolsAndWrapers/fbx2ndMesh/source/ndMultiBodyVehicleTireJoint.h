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
	class ndPacejkaTireModel
	{
		public:
		D_NEWTON_API ndPacejkaTireModel();
		D_NEWTON_API ndPacejkaTireModel(ndFloat32 B, ndFloat32 C, ndFloat32 D, ndFloat32 E, ndFloat32 Sv, ndFloat32 Sh);

		private:
		void CalculateMaxPhi();
		ndFloat32 Evaluate(ndFloat32 phi, ndFloat32 frictionCoefficient) const;

		public:
		ndFloat32 m_b;
		ndFloat32 m_c;
		ndFloat32 m_d;
		ndFloat32 m_e;
		ndFloat32 m_sv;
		ndFloat32 m_sh;
		ndFloat32 m_normalizingPhi;
		ndFloat32 m_norminalNormalForce;

		friend class ndMultiBodyVehicle;
		friend class ndTireFrictionModel;
	};

	enum ndFrictionModel
	{
		m_coulomb,
		m_pacejkaSport,
		m_pacejkaTruck,
		m_pacejkaUtility,
		m_pacejkaCustom,
		m_coulombCicleOfFriction,
	};

	D_NEWTON_API ndTireFrictionModel();
	D_NEWTON_API void PlotPacejkaCurves(const char* const name) const;

	D_NEWTON_API void SetPacejkaCurves(ndFrictionModel pacejkaStockModel);
	D_NEWTON_API void SetPacejkaCurves(const ndPacejkaTireModel& longitudinal, const ndPacejkaTireModel& lateral);
	D_NEWTON_API void GetPacejkaCurves(ndFrictionModel pacejkaStockModel, ndPacejkaTireModel& longitudinal, ndPacejkaTireModel& lateral) const;

	ndPacejkaTireModel m_lateralPacejka;
	ndPacejkaTireModel m_longitudinalPacejka;
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

	ndMultiBodyVehicleTireJointInfo(const ndWheelDescriptor& info, const ndTireFrictionModel& frictionModel)
		:ndWheelDescriptor(info)
		,ndTireFrictionModel(frictionModel)
	{
	}
};

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMultiBodyVehicleTireJoint: public ndJointWheel
{
	public:
	D_CLASS_REFLECTION(ndMultiBodyVehicleTireJoint, ndJointWheel)

	D_NEWTON_API ndMultiBodyVehicleTireJoint();
	D_NEWTON_API ndMultiBodyVehicleTireJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndMultiBodyVehicleTireJointInfo& desc, ndMultiBodyVehicle* const vehicle);
	D_NEWTON_API virtual ~ndMultiBodyVehicleTireJoint();

	D_NEWTON_API ndFloat32 GetSideSlip() const;
	D_NEWTON_API ndFloat32 GetLongitudinalSlip() const;
	D_NEWTON_API ndMultiBodyVehicleTireJointInfo GetInfo() const;

	D_NEWTON_API const ndTireFrictionModel& GetFrictionModel() const;
	D_NEWTON_API void SetVehicleOwner(ndMultiBodyVehicle* const vehicle);

	protected:
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc) override;

	ndMultiBodyVehicle* m_vehicle;
	ndTireFrictionModel m_frictionModel;
	ndFloat32 m_lateralSlip;
	ndFloat32 m_longitudinalSlip;
	ndFloat32 m_normalizedAligningTorque;

	friend class ndMultiBodyVehicle;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

