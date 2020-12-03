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

#include "ndCollisionStdafx.h"
#include "ndJointList.h"
#include "ndConstraint.h"
#include "ndBodyKinematic.h"

#define DG_BILATERAL_CONTRAINT_DOF	8

D_MSV_NEWTON_ALIGN_32
class ndJointBilateralConstraint: public ndConstraint, public dClassAlloc
{
/*
	public:
	virtual void SetDestructorCallback (OnConstraintDestroy destructor);
	virtual void Serialize (dgSerialize serializeCallback, void* const userData) = 0;

	bool IsRowMotor(dInt32 index) const {return m_rowIsMotor & (1 << index) ? true : false; }

	protected:
	virtual void Init (){dgAssert (0);}
	virtual void Remove (dgWorld* world) {dgAssert (0);}

	virtual dFloat32 GetStiffness() const;
	virtual void SetStiffness(dFloat32 stiffness);

	virtual dFloat32 GetMassScaleBody0() const;
	virtual dFloat32 GetMassScaleBody1() const;

	void CalculateMatrixOffset (const dVector& pivot, const dVector& dir, dMatrix& matrix0, dMatrix& matrix1) const;
	void SetPivotAndPinDir(const dVector &pivot, const dVector &pinDirection, dMatrix& matrix0, dMatrix& matrix1) const;
	void SetPivotAndPinDir(const dVector& pivot, const dVector& pinDirection0, const dVector& pinDirection1, dMatrix& matrix0, dMatrix& matrix1) const;
	dVector CalculateGlobalMatrixAndAngle (const dMatrix& localMatrix0, const dMatrix& localMatrix1, dMatrix& globalMatrix0, dMatrix& globalMatrix1) const;

	virtual void JointAccelerations(dgJointAccelerationDecriptor* const params); 

	dFloat32 GetRowAcceleration (dInt32 index, dgContraintDescritor& desc) const;
	void SetMotorAcceleration (dInt32 index, dFloat32 acceleration, dgContraintDescritor& desc);
	void SetSpringDamperAcceleration (dInt32 index, dgContraintDescritor& desc, dFloat32 rowStiffness, dFloat32 spring, dFloat32 damper);
	void SetJacobianDerivative (dInt32 index, dgContraintDescritor& desc, const dFloat32* const jacobianA, const dFloat32* const jacobianB, dgForceImpactPair* const jointForce);
	void CalculatePointDerivative (dInt32 index, dgContraintDescritor& desc, const dVector& normalGlobal, const dgPointParam& param, dgForceImpactPair* const jointForce);
	void CalculateAngularDerivative (dInt32 index, dgContraintDescritor& desc, const dVector& normalGlobal, dFloat32 stiffness, dFloat32 jointAngle, dgForceImpactPair* const jointForce);

	void AppendToJointList();
	
	dFloat32 m_massScaleBody0;
	dFloat32 m_massScaleBody1;
	dFloat32 m_defualtDiagonalRegularizer;
	OnConstraintDestroy m_destructor;
	dBilateralConstraintList::dListNode* m_jointNode;
*/
	public:
	D_COLLISION_API ndJointBilateralConstraint(dInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const dMatrix& globalMatrix);
	D_COLLISION_API virtual ~ndJointBilateralConstraint();
	
	bool IsCollidable() const;
	virtual ndJointBilateralConstraint* GetAsBilateral() { return this; }

	virtual const dUnsigned32 GetRowsCount() const;
	virtual void JacobianDerivative(ndConstraintDescritor& desc);

	virtual ndBodyKinematic* GetBody0() const;
	virtual ndBodyKinematic* GetBody1() const;

	
	void CalculateGlobalMatrix(dMatrix& matrix0, dMatrix& matrix1) const;
	D_COLLISION_API dFloat32 CalculateAngle(const dVector& planeDir, const dVector& cosDir, const dVector& sinDir) const;

	D_COLLISION_API virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc);
	D_COLLISION_API void CalculateLocalMatrix(const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const;
	D_COLLISION_API void AddAngularRowJacobian(ndConstraintDescritor& desc, const dVector& dir, dFloat32 relAngle);
	D_COLLISION_API void AddLinearRowJacobian(ndConstraintDescritor& desc, const dVector& pivot0, const dVector& pivot1, const dVector& dir);

	D_COLLISION_API virtual void DebugJoint(ndConstraintDebugCallback& debugCallback) const;

	virtual dInt32 GetSolverModel() const;
	virtual void SetSolverModel(dInt32 model);

	dFloat32 GetMotorZeroAcceleration(ndConstraintDescritor& desc) const;
	void SetHighFriction(ndConstraintDescritor& desc, dFloat32 friction);
	void SetLowerFriction(ndConstraintDescritor& desc, dFloat32 friction);
	void SetMotorAcceleration(ndConstraintDescritor& desc, dFloat32 acceleration);
	D_COLLISION_API void SetMassSpringDamperAcceleration(ndConstraintDescritor& desc, dFloat32 spring, dFloat32 damper);

	void SetSkeletonFlag(bool flag);

	protected:
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	dVector m_r0[DG_BILATERAL_CONTRAINT_DOF];
	dVector m_r1[DG_BILATERAL_CONTRAINT_DOF];
	ndForceImpactPair m_jointForce[DG_BILATERAL_CONTRAINT_DOF];
	dFloat32 m_motorAcceleration[DG_BILATERAL_CONTRAINT_DOF];
	ndBodyKinematic* m_body0;
	ndBodyKinematic* m_body1;
	ndJointList::dListNode* m_worldNode;
	ndJointList::dListNode* m_body0Node;
	ndJointList::dListNode* m_body1Node;

	dFloat32 m_maxAngleError;
	dFloat32 m_defualtDiagonalRegularizer;
	dUnsigned32 m_maxDof			: 6;
	dUnsigned32 m_solverModel		: 2;
	dUnsigned32 m_isInSkeleton		: 1;
	dUnsigned32 m_mark				: 1;
	dUnsigned32 m_enableCollision	: 1;
	dInt8 m_rowIsMotor;

	friend class ndWorld;
	friend class ndDynamicsUpdate;
	friend class ndSkeletonContainer;
};

inline dInt32 ndJointBilateralConstraint::GetSolverModel() const
{
	return m_solverModel;
}

inline void ndJointBilateralConstraint::SetSolverModel(dInt32 model)
{
	m_solverModel = dInt8(dClamp(model, 0, 3));
}

inline const dUnsigned32 ndJointBilateralConstraint::GetRowsCount() const
{
	return m_maxDof;
}

inline void ndJointBilateralConstraint::CalculateGlobalMatrix(dMatrix& matrix0, dMatrix& matrix1) const
{
	matrix0 = m_localMatrix0 * m_body0->GetMatrix();
	matrix1 = m_localMatrix1 * m_body1->GetMatrix();
}

inline ndBodyKinematic* ndJointBilateralConstraint::GetBody0() const
{
	return m_body0;
}

inline ndBodyKinematic* ndJointBilateralConstraint::GetBody1() const
{
	return m_body1;
}

inline dFloat32 ndJointBilateralConstraint::GetMotorZeroAcceleration(ndConstraintDescritor& desc) const
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32(m_maxDof));
	return desc.m_zeroRowAcceleration[index];
}

inline void ndJointBilateralConstraint::SetMotorAcceleration(ndConstraintDescritor& desc, dFloat32 acceleration)
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32(m_maxDof));
	m_rowIsMotor |= (1 << index);
	desc.m_flags[index] = 0;
	m_motorAcceleration[index] = acceleration;
	desc.m_jointAccel[index] = acceleration;
	desc.m_penetrationStiffness[index] = acceleration;
}

inline void ndJointBilateralConstraint::SetLowerFriction(ndConstraintDescritor& desc, dFloat32 friction)
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32 (m_maxDof));
	desc.m_forceBounds[index].m_low = dClamp(friction, dFloat32(D_MIN_BOUND), dFloat32(-0.001f));
	dAssert(desc.m_forceBounds[index].m_normalIndex == D_INDEPENDENT_ROW);

	#ifdef _DEBUG
	dInt32 i0 = 0;
	dInt32 i1 = index - 1;
	while ((i0 <= i1) && (desc.m_forceBounds[i0].m_normalIndex == D_INDEPENDENT_ROW)) i0++;
	while ((i1 >= i0) && (desc.m_forceBounds[i1].m_normalIndex != D_INDEPENDENT_ROW)) i1--;
	dAssert((i0 - i1) == 1);
	if ((i0 - i1) != 1) 
	{
		dTrace(("make sure that friction joint are issue at last\n"));
	}
	#endif
}

inline void ndJointBilateralConstraint::SetHighFriction(ndConstraintDescritor& desc, dFloat32 friction)
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32(m_maxDof));
	
	desc.m_forceBounds[index].m_upper = dClamp(friction, dFloat32(0.001f), dFloat32(D_MAX_BOUND));
	dAssert(desc.m_forceBounds[index].m_normalIndex == D_INDEPENDENT_ROW);

	#ifdef _DEBUG
	dInt32 i0 = 0;
	dInt32 i1 = index - 1;
	while ((i0 <= i1) && (desc.m_forceBounds[i0].m_normalIndex == D_INDEPENDENT_ROW)) i0++;
	while ((i1 >= i0) && (desc.m_forceBounds[i1].m_normalIndex != D_INDEPENDENT_ROW)) i1--;
	dAssert((i0 - i1) == 1);
	if ((i0 - i1) != 1) 
	{
		dTrace(("make sure that friction joint are issue at last\n"));
	}
	#endif
}

inline void ndJointBilateralConstraint::JacobianDerivative(ndConstraintDescritor& desc)
{
	dAssert(0);
}

inline void ndJointBilateralConstraint::SetSkeletonFlag(bool flag)
{
	m_isInSkeleton = flag ? 1 : 0;
}

inline bool ndJointBilateralConstraint::IsCollidable() const
{
	return m_enableCollision ? true : false;
}

#endif

