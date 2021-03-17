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

enum ndJointBilateralSolverModel
{
	m_jointIterativeSoft,
	m_jointkinematicOpenLoop,
	m_jointkinematicCloseLoop,
	//m_jointkinematicHintOpenLoop,
	m_jointkinematicAttachment,
	m_jointModesCount
};

D_MSV_NEWTON_ALIGN_32
class ndJointBilateralConstraint: public ndConstraint, public dClassAlloc
{
	public:
	D_COLLISION_API ndJointBilateralConstraint(dInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const dMatrix& globalMatrix);
	D_COLLISION_API virtual ~ndJointBilateralConstraint();

	virtual ndBodyKinematic* GetBody0() const;
	virtual ndBodyKinematic* GetBody1() const;
	virtual const dUnsigned32 GetRowsCount() const;
	virtual ndJointBilateralConstraint* GetAsBilateral() { return this; }
	virtual void JacobianDerivative(ndConstraintDescritor& desc);
	virtual ndJointBilateralSolverModel GetSolverModel() const;
	virtual void SetSolverModel(ndJointBilateralSolverModel model);

	D_COLLISION_API dFloat32 CalculateAngle(const dVector& planeDir, const dVector& cosDir, const dVector& sinDir) const;
	D_COLLISION_API virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc);
	D_COLLISION_API void CalculateLocalMatrix(const dMatrix& pinsAndPivotFrame, dMatrix& localMatrix0, dMatrix& localMatrix1) const;
	D_COLLISION_API void AddAngularRowJacobian(ndConstraintDescritor& desc, const dVector& dir, dFloat32 relAngle);
	D_COLLISION_API void AddLinearRowJacobian(ndConstraintDescritor& desc, const dVector& pivot0, const dVector& pivot1, const dVector& dir);

	D_COLLISION_API virtual void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_COLLISION_API dFloat32 CalculateSpringDamperAcceleration(dFloat32 dt, dFloat32 ks, dFloat32 x, dFloat32 kd, dFloat32 v) const;
	D_COLLISION_API void SetMassSpringDamperAcceleration(ndConstraintDescritor& desc, dFloat32 regularizer, dFloat32 spring, dFloat32 damper);
	
	const dMatrix& GetLocalMatrix0() const;
	const dMatrix& GetLocalMatrix1() const;

	dVector GetForceBody0() const;
	dVector GetTorqueBody0() const;
	dVector GetForceBody1() const;
	dVector GetTorqueBody1() const;

	bool IsCollidable() const;
	void SetSkeletonFlag(bool flag);
	void CalculateGlobalMatrix(dMatrix& matrix0, dMatrix& matrix1) const;
	dFloat32 GetMotorZeroAcceleration(ndConstraintDescritor& desc) const;
	void SetHighFriction(ndConstraintDescritor& desc, dFloat32 friction);
	void SetLowerFriction(ndConstraintDescritor& desc, dFloat32 friction);
	void SetMotorAcceleration(ndConstraintDescritor& desc, dFloat32 acceleration);
	dFloat32 GetDiagonalRegularizer(const ndConstraintDescritor& desc) const;
	void SetDiagonalRegularizer(ndConstraintDescritor& desc, dFloat32 stiffness);

	bool IsSkeleton() const;

	protected:
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
	dVector m_forceBody0;
	dVector m_torqueBody0;
	dVector m_forceBody1;
	dVector m_torqueBody1;

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
	dUnsigned32 m_mark0				: 1;
	dUnsigned32 m_mark1				: 1;
	dUnsigned32 m_isInSkeleton		: 1;
	dUnsigned32 m_enableCollision	: 1;
	dInt8 m_rowIsMotor;
	ndJointBilateralSolverModel m_solverModel;

	friend class ndWorld;
	friend class ndDynamicsUpdate;
	friend class ndSkeletonContainer;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateOpencl;
};

inline ndJointBilateralSolverModel ndJointBilateralConstraint::GetSolverModel() const
{
	return m_solverModel;
}

inline void ndJointBilateralConstraint::SetSolverModel(ndJointBilateralSolverModel model)
{
	dAssert(model < m_jointModesCount);
	dAssert(model >= m_jointIterativeSoft);
	m_solverModel = dClamp(model, m_jointIterativeSoft, m_jointModesCount);
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

inline const dMatrix& ndJointBilateralConstraint::GetLocalMatrix0() const
{
	return m_localMatrix0;
}

inline const dMatrix& ndJointBilateralConstraint::GetLocalMatrix1() const
{
	return m_localMatrix1;
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

inline dVector ndJointBilateralConstraint::GetForceBody0() const
{
	return m_forceBody0;
}

inline dVector ndJointBilateralConstraint::GetTorqueBody0() const
{
	return m_torqueBody0;
}

inline dVector ndJointBilateralConstraint::GetForceBody1() const
{
	return m_forceBody1;
}

inline dVector ndJointBilateralConstraint::GetTorqueBody1() const
{
	return m_torqueBody1;
}

inline dFloat32 ndJointBilateralConstraint::GetDiagonalRegularizer(const ndConstraintDescritor& desc) const
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32(m_maxDof));
	return desc.m_diagonalRegularizer[index];
}

inline void ndJointBilateralConstraint::SetDiagonalRegularizer(ndConstraintDescritor& desc, dFloat32 stiffness)
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32(m_maxDof));
	desc.m_diagonalRegularizer[index] = dClamp(stiffness, dFloat32(0.0f), dFloat32(1.0f));
}

inline bool ndJointBilateralConstraint::IsSkeleton() const
{
	const ndJointBilateralSolverModel mode = GetSolverModel();
	bool test = false;
	test = test || (mode == m_jointkinematicOpenLoop);
	test = test || (mode == m_jointkinematicCloseLoop);
	//test = test || (mode == m_jointkinematicHintOpenLoop);
	return test;
}

#endif

