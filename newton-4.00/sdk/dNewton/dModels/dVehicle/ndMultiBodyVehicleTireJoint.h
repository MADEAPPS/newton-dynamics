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
#include "dJoints/ndJointWheel.h"

class ndTireFrictionModel
{
	public:
	class ndBrushTireModel
	{
		public:
		D_NEWTON_API ndBrushTireModel();
		D_NEWTON_API ndBrushTireModel(ndFloat32 laterialStiffness, ndFloat32 longitudinalStiffness);

		ndFloat32 m_laterialStiffness;
		ndFloat32 m_longitudinalStiffness;
	};

	class ndPacejkaTireModel
	{
		public:
		D_NEWTON_API ndPacejkaTireModel();
		D_NEWTON_API ndPacejkaTireModel(ndFloat32 B, ndFloat32 C, ndFloat32 D, ndFloat32 E, ndFloat32 Sv, ndFloat32 Sh);

		private:
		void CalculateMaxPhi();
		ndFloat32 Evaluate(ndFloat32 phi, ndFloat32 f) const;

		ndFloat32 m_b;
		ndFloat32 m_c;
		ndFloat32 m_d;
		ndFloat32 m_e;
		ndFloat32 m_sv;
		ndFloat32 m_sh;
		ndFloat32 m_normalizingPhi;

		friend class ndMultiBodyVehicle;
	};

	enum ndFrictionModel
	{
		m_coulomb,
		m_brushModel,
		m_pacejka,
		m_coulombCicleOfFriction,
	};

	ndTireFrictionModel()
		:m_brush()
		//,m_lateralPacejka(ndFloat32(0.34f), ndFloat32(1.5f), ndFloat32(0.5f), ndFloat32(-0.74f), ndFloat32(0.0f), ndFloat32(0.01f))
		,m_lateralPacejka(ndFloat32(0.01f), ndFloat32(2.85f), ndFloat32(0.2f), ndFloat32(1.42f), ndFloat32(0.0f), ndFloat32(0.01f))
		,m_longitudinalPacejka(ndFloat32(0.5f), ndFloat32(1.65f), ndFloat32(1.0f), ndFloat32(0.8f), ndFloat32(0.0f), ndFloat32(0.0f))
		,m_frictionModel(m_brushModel)
	{
	}

	ndBrushTireModel m_brush;
	ndPacejkaTireModel m_lateralPacejka;
	ndPacejkaTireModel m_longitudinalPacejka;

	//ndFloat32 m_laterialStiffness;
	//ndFloat32 m_longitudinalStiffness;
	ndFrictionModel m_frictionModel;
};

class ndMultiBodyVehicleTireJointInfo : public ndWheelDescriptor, public ndTireFrictionModel
{
	public:
	ndMultiBodyVehicleTireJointInfo()
		:ndWheelDescriptor()
		,ndTireFrictionModel()
	{
		ndPacejkaTireModel pacejkaLongitudical(0.5f, 1.65f, 1.0f, 0.8f, 0.0f, 0.0f);
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
	D_NEWTON_API void JacobianDerivative(ndConstraintDescritor& desc);

	ndMultiBodyVehicle* m_vehicle;
	ndTireFrictionModel m_frictionModel;
	ndFloat32 m_lateralSlip;
	ndFloat32 m_longitudinalSlip;
	ndFloat32 m_normalizedAligningTorque;

	bool m_hasVSC;

	friend class ndMultiBodyVehicle;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif 

