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

#ifndef __D_JOINT_BILATERAL_CONSTRAINT_H__
#define __D_JOINT_BILATERAL_CONSTRAINT_H__

#include "ndNewtonStdafx.h"

class ndJointBilateralConstraint;
#define DG_BILATERAL_CONTRAINT_DOF	8

//class dBilateralConstraintList: public dgList<ndJointBilateralConstraint*>
//{
//	public:
//	dBilateralConstraintList(dgMemoryAllocator* const allocator)
//		:dgList<ndJointBilateralConstraint*>(allocator)
//	{
//	}
//};

D_MSV_NEWTON_ALIGN_32
class ndJointBilateralConstraint: public ndConstraint
{
/*
	public:
	virtual void SetDestructorCallback (OnConstraintDestroy destructor);
	virtual void Serialize (dgSerialize serializeCallback, void* const userData) = 0;

	bool IsRowMotor(dgInt32 index) const {return m_rowIsMotor & (1 << index) ? true : false; }

	protected:
	virtual void Init (){dgAssert (0);}
	virtual void Remove (dgWorld* world) {dgAssert (0);}

	virtual dFloat32 GetStiffness() const;
	virtual void SetStiffness(dFloat32 stiffness);

	virtual dgInt32 GetSolverModel() const;
	virtual void SetSolverModel(dgInt32 model);

	virtual dFloat32 GetMassScaleBody0() const;
	virtual dFloat32 GetMassScaleBody1() const;

	void CalculateMatrixOffset (const dVector& pivot, const dVector& dir, dgMatrix& matrix0, dgMatrix& matrix1) const;
	void SetPivotAndPinDir(const dVector &pivot, const dVector &pinDirection, dgMatrix& matrix0, dgMatrix& matrix1) const;
	void SetPivotAndPinDir(const dVector& pivot, const dVector& pinDirection0, const dVector& pinDirection1, dgMatrix& matrix0, dgMatrix& matrix1) const;
	dVector CalculateGlobalMatrixAndAngle (const dgMatrix& localMatrix0, const dgMatrix& localMatrix1, dgMatrix& globalMatrix0, dgMatrix& globalMatrix1) const;

	virtual void JointAccelerations(dgJointAccelerationDecriptor* const params); 

	dFloat32 GetRowAcceleration (dgInt32 index, dgContraintDescritor& desc) const;
	dFloat32 CalculateMotorAcceleration (dgInt32 index, dgContraintDescritor& desc) const;
	void SetMotorAcceleration (dgInt32 index, dFloat32 acceleration, dgContraintDescritor& desc);
	void SetSpringDamperAcceleration (dgInt32 index, dgContraintDescritor& desc, dFloat32 rowStiffness, dFloat32 spring, dFloat32 damper);
	void SetJacobianDerivative (dgInt32 index, dgContraintDescritor& desc, const dFloat32* const jacobianA, const dFloat32* const jacobianB, dgForceImpactPair* const jointForce);
	void CalculatePointDerivative (dgInt32 index, dgContraintDescritor& desc, const dVector& normalGlobal, const dgPointParam& param, dgForceImpactPair* const jointForce);
	void CalculateAngularDerivative (dgInt32 index, dgContraintDescritor& desc, const dVector& normalGlobal, dFloat32 stiffness, dFloat32 jointAngle, dgForceImpactPair* const jointForce);

	void AppendToJointList();
	
	dVector m_r0[DG_BILATERAL_CONTRAINT_DOF];
	dVector m_r1[DG_BILATERAL_CONTRAINT_DOF];
	dgForceImpactPair m_jointForce[DG_BILATERAL_CONTRAINT_DOF];
	dFloat32 m_motorAcceleration[DG_BILATERAL_CONTRAINT_DOF];
	dFloat32 m_massScaleBody0;
	dFloat32 m_massScaleBody1;
	dFloat32 m_defualtDiagonalRegularizer;
	OnConstraintDestroy m_destructor;
	dBilateralConstraintList::dListNode* m_jointNode;
	dgInt8	  m_rowIsMotor;

	friend class dgBodyMasterList;
	friend class dgWorldDynamicUpdate;
*/
	public:
	D_NEWTON_API ndJointBilateralConstraint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const dMatrix& globalMatrix);
	D_NEWTON_API virtual ~ndJointBilateralConstraint();

	protected:
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
	ndBodyKinematic* m_body0;
	ndBodyKinematic* m_body1;
};

#endif

