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


#ifndef __D_VEHICLE_TIRE_H__
#define __D_VEHICLE_TIRE_H__

#include "dStdafxVehicle.h"
#include "dVehicleNode.h"


class dVehicleTireInterface: public dVehicleNode
{
	public:
	enum dSuspensionType
	{
		m_offroad,
		m_confort,
		m_race,
		m_roller,
	};

	class dTireInfo
	{
		public:
		dTireInfo()
		{
			memset(this, 0, sizeof(dTireInfo));
		}

		dFloat m_mass;
		dFloat m_radio;
		dFloat m_width;
		dFloat m_pivotOffset;
		dFloat m_maxSteeringAngle;
		dFloat m_dampingRatio;
		dFloat m_springStrength;
		dFloat m_suspensionLength;
		dFloat m_corneringStiffness;
		dFloat m_aligningMomentTrail;
		int m_hasFender;
		dSuspensionType m_suspentionType;
	};

	DVEHICLE_API dVehicleTireInterface(dVehicleNode* const parent, const dMatrix& location, const dTireInfo& info);
	DVEHICLE_API virtual ~dVehicleTireInterface();

	DVEHICLE_API virtual NewtonCollision* GetCollisionShape() const = 0;
};


#endif 

