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

#ifndef _D_DYNAMIC_BODY_H_
#define _D_DYNAMIC_BODY_H_

#include "dNewtonStdafx.h"
#include "dBody.h"

//#define DG_MAX_SPEED_ATT	dgFloat32(0.02f)
////#define DG_FREEZE_ACCEL	dgFloat32(0.1f)
//#define DG_FREEZE_ACCEL		dgFloat32(1.0f)
//#define DG_FREEZE_SPEED		dgFloat32(0.032f)
//
//#define DG_FREEZE_ACCEL2	(DG_FREEZE_ACCEL * DG_FREEZE_ACCEL)
//#define DG_FREEZE_SPEED2	(DG_FREEZE_SPEED * DG_FREEZE_SPEED)
//
//#define DG_FREEZE_MAG		DG_FREEZE_ACCEL
//#define DG_FREEZE_MAG2		(DG_FREEZE_MAG * DG_FREEZE_MAG)
//
//#define DG_ERR_TOLERANCE	dgFloat32(1.0e-2f)
//#define DG_ERR_TOLERANCE2	(DG_ERR_TOLERANCE * DG_ERR_TOLERANCE)
//
//class dgSkeletonContainer;

D_MSC_VECTOR_ALIGNMENT
class dDynamicBody: public dBody 
{
	public:
	D_NEWTON_API dDynamicBody();
	D_NEWTON_API virtual ~dDynamicBody ();
	D_NEWTON_API virtual dDynamicBody* GetAsDynamicBody() { return this; }
	D_NEWTON_API virtual void ApplyExternalForces(dInt32 threadIndex, dFloat32 tiemstep);

	dVector GetMassMatrix() const;
	void SetMassMatrix(const dVector& massMatrix);

	void SetMassMatrix(dFloat32 mass, const dShapeInstance& shapeInstance);
	void SetMassMatrix(dFloat32 Ixx, dFloat32 Iyy, dFloat32 Izz, dFloat32 mass);
	void GetMassMatrix(dFloat32& Ixx, dFloat32& Iyy, dFloat32& Izz, dFloat32& mass);

	dVector GetForce() const;
	void SetForce(const dVector& force);
	
	dVector GetToque() const;
	void SetTorque(const dVector& torque);

	private:
	void SetMassMatrix(dFloat32 mass, const dMatrix& inertia);

	protected:
	dVector m_mass;
	dVector m_invMass;
	dVector m_externalForce;
	dVector m_externalTorque;

} D_GCC_VECTOR_ALIGNMENT;

inline dVector dDynamicBody::GetForce() const
{
	return m_externalForce;
}

inline void dDynamicBody::SetForce(const dVector& force)
{
	m_externalForce = force;
}

inline dVector dDynamicBody::GetToque() const
{
	return m_externalTorque;
}

inline void dDynamicBody::SetTorque(const dVector& torque)
{
	m_externalTorque = torque;
}


inline dVector dDynamicBody::GetMassMatrix() const
{
	return m_mass;
}

inline void dDynamicBody::GetMassMatrix(dFloat32& Ixx, dFloat32& Iyy, dFloat32& Izz, dFloat32& mass)
{
	Ixx = m_mass.m_x;
	Iyy = m_mass.m_y;
	Izz = m_mass.m_z;
	mass = m_mass.m_w;
}

inline void dDynamicBody::SetMassMatrix(const dVector& massMatrix)
{
	dMatrix inertia(dGetZeroMatrix());
	inertia[0][0] = massMatrix.m_x;
	inertia[1][1] = massMatrix.m_y;
	inertia[2][2] = massMatrix.m_z;
	SetMassMatrix(massMatrix.m_w, inertia);
}

inline void dDynamicBody::SetMassMatrix(dFloat32 Ixx, dFloat32 Iyy, dFloat32 Izz, dFloat32 mass)
{
	SetMassMatrix(dVector(Ixx, Iyy, Izz, mass));
}

inline void dDynamicBody::SetMassMatrix(dFloat32 mass, const dShapeInstance& shapeInstance)
{
	dMatrix inertia(shapeInstance.CalculateInertia());

	dVector origin(inertia.m_posit);
	for (dInt32 i = 0; i < 3; i++) {
		inertia[i] = inertia[i].Scale(mass);
		//inertia[i][i] = (inertia[i][i] + origin[i] * origin[i]) * mass;
		//for (dInt32 j = i + 1; j < 3; j ++) {
		//	dgFloat32 crossIJ = origin[i] * origin[j];
		//	inertia[i][j] = (inertia[i][j] + crossIJ) * mass;
		//	inertia[j][i] = (inertia[j][i] + crossIJ) * mass;
		//}
	}

	// although the engine fully supports asymmetric inertia, I will ignore cross inertia for now
	SetCentreOfMass(origin);
	SetMassMatrix(mass, inertia);
}

#endif 


