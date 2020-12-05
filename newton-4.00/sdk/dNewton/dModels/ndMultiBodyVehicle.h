/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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
class ndJointWheel;
class ndMultiBodyVehicle: public ndModel
{
	public:
	D_NEWTON_API ndMultiBodyVehicle(const dVector& frontDir, const dVector& upDir);
	D_NEWTON_API ndMultiBodyVehicle(const nd::TiXmlNode* const xmlNode);
	D_NEWTON_API virtual ~ndMultiBodyVehicle ();

	virtual dFloat32 GetFrictionCoeficient(ndJointWheel* const tire, const ndContactMaterial& contactPoint) const
	{
		return dFloat32(1.4f);
	}

	D_NEWTON_API void SetBrakeTorque(dFloat32 brakeToqrue);
	D_NEWTON_API void SetHandBrakeTorque(dFloat32 brakeToqrue);
	D_NEWTON_API void SetSteeringAngle(dFloat32 angleInRadians);

	D_NEWTON_API ndShapeInstance CreateTireShape(dFloat32 radius, dFloat32 width) const;

	D_NEWTON_API void AddChassis(ndBodyDynamic* const chassis);
	D_NEWTON_API ndJointWheel* AddTire(ndWorld* const world, ndBodyDynamic* const tire);

	D_NEWTON_API void SetAsBrake(ndJointWheel* const tire);
	D_NEWTON_API void SetAsHandBrake(ndJointWheel* const tire);
	D_NEWTON_API void SetAsSteering(ndJointWheel* const tire);

	private:
	void ApplyBrakes();
	void ApplySteering();
	void ApplyTiremodel();
	void ApplyAligmentAndBalancing();

	protected:
	D_NEWTON_API virtual void Debug(ndConstraintDebugCallback& context) const;
	D_NEWTON_API virtual void Update(const ndWorld* const world, dFloat32 timestep);

	dMatrix m_localFrame;
	ndBodyDynamic* m_chassis;
	ndShapeChamferCylinder* m_tireShape;
	dList<ndJointWheel*> m_tiresList;
	dList<ndJointWheel*> m_brakeTires;
	dList<ndJointWheel*> m_handBrakeTires;
	dList<ndJointWheel*> m_steeringTires;
	dFloat32 m_brakeTorque;
	dFloat32 m_steeringAngle;
	dFloat32 m_handBrakeTorque;
	dFloat32 m_steeringAngleMemory;
};

#endif