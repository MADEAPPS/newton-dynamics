/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __DGBILATERALCONSTRAINT_H__
#define __DGBILATERALCONSTRAINT_H__

#include "dgConstraint.h"

#define DG_BILATERAL_CONTRAINT_DOF	8

class dgBilateralConstraint: public dgConstraint  
{
	public:
	virtual void SetDestructorCallback (OnConstraintDestroy destructor);
	virtual void Serialize (dgSerialize serializeCallback, void* const userData) = 0;

	protected:
	dgBilateralConstraint ();
    virtual ~dgBilateralConstraint ();

	virtual void Init (){dgAssert (0);}
	virtual void Remove (dgWorld* world) {dgAssert (0);}

	virtual dgFloat32 GetStiffness() const;
	virtual void SetStiffness(dgFloat32 stiffness);

	void SetPivotAndPinDir(const dgVector &pivot, const dgVector &pinDirection);
	void SetPivotAndPinDir (const dgVector& pivot, const dgVector& pinDirection0, const dgVector& pinDirection1);
	dgVector CalculateGlobalMatrixAndAngle (dgMatrix& globalMatrix0, dgMatrix& globalMatrix1) const;
	void CalculateMatrixOffset (const dgVector& pivot, const dgVector& dir, dgMatrix& matrix0, dgMatrix& matrix1);

	virtual void JointAccelerations(dgJointAccelerationDecriptor* const params); 

	dgFloat32 CalculateMotorAcceleration (dgInt32 index, dgContraintDescritor& desc) const;
	void SetMotorAcceleration (dgInt32 index, dgFloat32 acceleration, dgContraintDescritor& desc);
	void SetSpringDamperAcceleration (dgInt32 index, dgContraintDescritor& desc, dgFloat32 spring, dgFloat32 damper);
	void SetJacobianDerivative (dgInt32 index, dgContraintDescritor& desc, const dgFloat32* const jacobianA, const dgFloat32* const jacobianB, dgForceImpactPair* const jointForce);
	void CalculatePointDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& normalGlobal, const dgPointParam& param, dgForceImpactPair* const jointForce);
	void CalculateAngularDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& normalGlobal, dgFloat32 stiffness, dgFloat32 jointAngle, dgForceImpactPair* const jointForce);
	
	dgForceImpactPair m_jointForce[DG_BILATERAL_CONTRAINT_DOF];
	dgFloat32 m_motorAcceleration[DG_BILATERAL_CONTRAINT_DOF];
	dgInt32	  m_rowIsMotor[DG_BILATERAL_CONTRAINT_DOF];
	OnConstraintDestroy m_destructor;
	dgFloat32 m_stiffness;

	friend class dgWorldDynamicUpdate;
};

#endif

