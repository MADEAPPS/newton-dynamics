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

#if !defined(AFX_DGBILATERALCONSTRAINT_H__7C9E1F9A_5EC6_48BE_8C9F_FB90132C6619_H)
#define AFX_DGBILATERALCONSTRAINT_H__7C9E1F9A_5EC6_48BE_8C9F_FB90132C6619_H

#include "dgConstraint.h"


class dgBilateralConstraint: public dgConstraint  
{

	public:
	virtual void SetDestructorCallback (OnConstraintDestroy destructor);

	protected:
	dgBilateralConstraint ();
    virtual ~dgBilateralConstraint ();

	virtual void Init (){dgAssert (0);}
	virtual void Remove (dgWorld* world) {dgAssert (0);}
	virtual bool IsBilateral () const;

	virtual dgFloat32 GetStiffness() const;
	virtual void SetStiffness(dgFloat32 stiffness);

	void SetPivotAndPinDir(const dgVector &pivot, const dgVector &pinDirection);
	void SetPivotAndPinDir (const dgVector& pivot, const dgVector& pinDirection0, const dgVector& pinDirection1);
	dgVector CalculateGlobalMatrixAndAngle (dgMatrix& globalMatrix0, dgMatrix& globalMatrix1) const;
	void CalculateMatrixOffset (const dgVector& pivot, const dgVector& dir, dgMatrix& matrix0, dgMatrix& matrix1);


	virtual void JointAccelerations(dgJointAccelerationDecriptor* const params); 
	virtual void JointAccelerationsSimd(dgJointAccelerationDecriptor* const params);
	virtual void JointVelocityCorrection(dgJointAccelerationDecriptor* const params); 


	void SetMotorAcceleration (dgInt32 index, dgFloat32 accelration, dgContraintDescritor& desc);
	void SetJacobianDerivative (dgInt32 index, dgContraintDescritor& desc, const dgFloat32* jacobianA, const dgFloat32* jacobianB, dgFloat32* const jointForce);
	void CalculatePointDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& normalGlobal, const dgPointParam& param, dgFloat32* const jointForce);
	void CalculateAngularDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& normalGlobal, dgFloat32 stiffness, dgFloat32 jointAngle, dgFloat32* const jointForce);
	dgFloat32 CalculateSpringDamperAcceleration (dgInt32 index, const dgContraintDescritor& desc, dgFloat32 jointAngle, const dgVector& p0Global, const dgVector& p1Global, dgFloat32 springK, dgFloat32 springD);

	void SetMaxContactsForExactSolver (bool mode, dgInt32 MaxCount);

	dgMatrix m_localMatrix0;
	dgMatrix m_localMatrix1;
	dgFloat32 m_stiffness;
	dgFloat32 m_jointForce[32];
	dgInt32	  m_rowIsMotor[32];
	dgFloat32 m_motorAcceleration[32];
	dgInt32 m_useExactSolverContactLimit;
	
	OnConstraintDestroy m_destructor;

	friend class dgWorldDynamicUpdate;
};

#endif // !defined(AFX_DGBILATERALCONSTRAINT_H__7C9E1F9A_5EC6_48BE_8C9F_FB90132C6619_H)

