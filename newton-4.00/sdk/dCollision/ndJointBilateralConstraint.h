/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_JOINT_BILATERAL_CONSTRAINT_H__
#define __ND_JOINT_BILATERAL_CONSTRAINT_H__

#include "ndCollisionStdafx.h"
#include "ndJointList.h"
#include "ndConstraint.h"
#include "ndBodyKinematic.h"


#define ND_BILATERAL_CONTRAINT_DOF	12
#define ND_SPRING_DAMP_MIN_REG		ndFloat32 (1.0e-3f)

enum ndJointBilateralSolverModel
{
	m_jointIterativeSoft,
	m_jointkinematicOpenLoop,
	m_jointkinematicCloseLoop,
	m_jointkinematicAttachment,
	m_jointModesCount
};

#define D_ADD_IK_INTERFACE()															\
virtual void SetIkMode(bool mode)														\
{																						\
	m_ikMode = mode;																	\
}																						\
virtual void SetIkSetAccel(const ndJacobian& body0Accel, const ndJacobian& body1Accel)	\
{																						\
	m_accel0 = body0Accel;																\
	m_accel1 = body1Accel;																\
}

D_MSV_NEWTON_ALIGN_32
class ndJointBilateralConstraint : public ndConstraint
{
	public:
	D_BASE_CLASS_REFLECTION(ndJointBilateralConstraint);

	class ndIkInterface
	{
		public:
		ndIkInterface()
			:m_defualRegularizer(ndFloat32 (1.0e-3f))
			,m_ikMode(true)
		{}

		ndJacobian m_accel0;
		ndJacobian m_accel1;
		ndFloat32 m_defualRegularizer;
		bool m_ikMode;
	};

	class ndKinematicState
	{
		public:
		ndFloat32 m_posit;
		ndFloat32 m_velocity;
	};

	D_COLLISION_API ndJointBilateralConstraint();
	D_COLLISION_API ndJointBilateralConstraint(ndInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix);
	D_COLLISION_API ndJointBilateralConstraint(ndInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrixBody0,  const ndMatrix& globalMatrixBody1);
	D_COLLISION_API virtual ~ndJointBilateralConstraint();

	void ReplaceSentinel(ndBodyKinematic* const sentinel);

	virtual ndJointBilateralConstraint* GetAsBilateral();
	virtual void JacobianDerivative(ndConstraintDescritor& desc);
	virtual ndJointBilateralSolverModel GetSolverModel() const;
	virtual void SetSolverModel(ndJointBilateralSolverModel model);

	D_COLLISION_API ndFloat32 CalculateAngle(const ndVector& planeDir, const ndVector& cosDir, const ndVector& sinDir) const;
	D_COLLISION_API virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc);
	D_COLLISION_API void CalculateLocalMatrix(const ndMatrix& pinsAndPivotFrame, ndMatrix& localMatrix0, ndMatrix& localMatrix1) const;
	D_COLLISION_API void AddAngularRowJacobian(ndConstraintDescritor& desc, const ndVector& dir, ndFloat32 relAngle);
	D_COLLISION_API void AddLinearRowJacobian(ndConstraintDescritor& desc, const ndVector& pivot0, const ndVector& pivot1, const ndVector& dir);

	D_COLLISION_API virtual void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_COLLISION_API ndFloat32 CalculateSpringDamperAcceleration(ndFloat32 dt, ndFloat32 ks, ndFloat32 x, ndFloat32 kd, ndFloat32 v) const;
	D_COLLISION_API void SetMassSpringDamperAcceleration(ndConstraintDescritor& desc, ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);

	D_COLLISION_API virtual ndInt32 GetKinematicState(ndKinematicState* const state) const;

	
	const ndMatrix& GetLocalMatrix0() const;
	const ndMatrix& GetLocalMatrix1() const;

	bool IsInWorld() const;
	bool IsSkeleton() const;
	bool IsBilateral() const;
	bool IsCollidable() const;
	bool GetSkeletonFlag() const;

	void SetCollidable(bool state);
	void SetSkeletonFlag(bool flag);
	void CalculateGlobalMatrix(ndMatrix& matrix0, ndMatrix& matrix1) const;

	void SetHighFriction(ndConstraintDescritor& desc, ndFloat32 friction);
	void SetLowerFriction(ndConstraintDescritor& desc, ndFloat32 friction);
	void SetJointErrorPosit(ndConstraintDescritor& desc, ndFloat32 errorPosit);
	void SetMotorAcceleration(ndConstraintDescritor& desc, ndFloat32 acceleration);
	void SetDiagonalRegularizer(ndConstraintDescritor& desc, ndFloat32 regularizer);

	ndFloat32 GetJointErrorPosit(ndConstraintDescritor& desc) const;
	ndFloat32 GetJointErrorSpeed(ndConstraintDescritor& desc) const;
	ndFloat32 GetMotorAcceleration(ndConstraintDescritor& desc) const;
	ndFloat32 GetMotorZeroAcceleration(ndConstraintDescritor& desc) const;
	ndFloat32 GetDiagonalRegularizer(const ndConstraintDescritor& desc) const;

	protected:
	// inverse dynamics interface
	D_COLLISION_API virtual void SetIkMode(bool mode);
	D_COLLISION_API virtual void SetIkSetAccel(const ndJacobian& body0Accel, const ndJacobian& body1Accel);

	ndMatrix m_localMatrix0;
	ndMatrix m_localMatrix1;

	ndVector m_r0[ND_BILATERAL_CONTRAINT_DOF];
	ndVector m_r1[ND_BILATERAL_CONTRAINT_DOF];
	ndForceImpactPair m_jointForce[ND_BILATERAL_CONTRAINT_DOF];
	ndFloat32 m_motorAcceleration[ND_BILATERAL_CONTRAINT_DOF];
	ndJointList::ndNode* m_worldNode;
	ndBodyKinematic::ndJointList::ndNode* m_body0Node;
	ndBodyKinematic::ndJointList::ndNode* m_body1Node;
	ndSpecialList<ndJointBilateralConstraint>::ndNode* m_deletedNode;

	ndFloat32 m_defualtDiagonalRegularizer;
	ndUnsigned32 m_mark0			: 1;
	ndUnsigned32 m_mark1			: 1;
	ndUnsigned32 m_isInSkeleton		: 1;
	ndUnsigned32 m_enableCollision	: 1;
	ndInt8 m_rowIsMotor;
	ndJointBilateralSolverModel m_solverModel;

	friend class ndWorld;
	friend class ndIkSolver;
	friend class ndDynamicsUpdate;
	friend class ndFileFormatJoint;
	friend class ndModelArticulation;
	friend class ndSkeletonContainer;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateSycl;
	friend class ndDynamicsUpdateCuda;
};

inline ndJointBilateralSolverModel ndJointBilateralConstraint::GetSolverModel() const
{
	return m_solverModel;
}

inline void ndJointBilateralConstraint::SetSolverModel(ndJointBilateralSolverModel model)
{
	ndAssert(model < m_jointModesCount);
	ndAssert(model >= m_jointIterativeSoft);
	m_solverModel = ndClamp(model, m_jointIterativeSoft, m_jointModesCount);
}

//inline ndUnsigned32 ndJointBilateralConstraint::GetRowsCount() const
//{
//	return m_maxDof;
//}

inline void ndJointBilateralConstraint::CalculateGlobalMatrix(ndMatrix& matrix0, ndMatrix& matrix1) const
{
	matrix0 = m_localMatrix0 * m_body0->GetMatrix();
	matrix1 = m_localMatrix1 * m_body1->GetMatrix();
}

inline const ndMatrix& ndJointBilateralConstraint::GetLocalMatrix0() const
{
	return m_localMatrix0;
}

inline const ndMatrix& ndJointBilateralConstraint::GetLocalMatrix1() const
{
	return m_localMatrix1;
}

inline ndFloat32 ndJointBilateralConstraint::GetMotorZeroAcceleration(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_zeroRowAcceleration[index];
}

inline void ndJointBilateralConstraint::SetMotorAcceleration(ndConstraintDescritor& desc, ndFloat32 acceleration)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	m_rowIsMotor |= (1 << index);
	desc.m_flags[index] = 0;
	m_motorAcceleration[index] = acceleration;
	desc.m_jointAccel[index] = acceleration;
}

inline ndFloat32 ndJointBilateralConstraint::GetMotorAcceleration(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_jointAccel[index];
}

inline void ndJointBilateralConstraint::SetJointErrorPosit(ndConstraintDescritor& desc, ndFloat32 errorPosit)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	desc.m_penetration[index] = errorPosit;
}

inline void ndJointBilateralConstraint::SetLowerFriction(ndConstraintDescritor& desc, ndFloat32 friction)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32 (m_maxDof));
	desc.m_forceBounds[index].m_low = ndClamp(friction, ndFloat32(D_MIN_BOUND), ndFloat32(-0.001f));
	ndAssert(desc.m_forceBounds[index].m_normalIndex == D_INDEPENDENT_ROW);

	#ifdef _DEBUG
	ndInt32 i0 = 0;
	ndInt32 i1 = index - 1;
	while ((i0 <= i1) && (desc.m_forceBounds[i0].m_normalIndex == D_INDEPENDENT_ROW)) i0++;
	while ((i1 >= i0) && (desc.m_forceBounds[i1].m_normalIndex != D_INDEPENDENT_ROW)) i1--;
	ndAssert((i0 - i1) == 1);
	if ((i0 - i1) != 1) 
	{
		ndTrace(("make sure that friction joint are issue at last\n"));
	}
	#endif
}

inline void ndJointBilateralConstraint::SetHighFriction(ndConstraintDescritor& desc, ndFloat32 friction)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	
	desc.m_forceBounds[index].m_upper = ndClamp(friction, ndFloat32(0.001f), ndFloat32(D_MAX_BOUND));
	ndAssert(desc.m_forceBounds[index].m_normalIndex == D_INDEPENDENT_ROW);

	#ifdef _DEBUG
	ndInt32 i0 = 0;
	ndInt32 i1 = index - 1;
	while ((i0 <= i1) && (desc.m_forceBounds[i0].m_normalIndex == D_INDEPENDENT_ROW)) i0++;
	while ((i1 >= i0) && (desc.m_forceBounds[i1].m_normalIndex != D_INDEPENDENT_ROW)) i1--;
	ndAssert((i0 - i1) == 1);
	if ((i0 - i1) != 1) 
	{
		ndTrace(("make sure that friction joint are issue at last\n"));
	}
	#endif
}

inline void ndJointBilateralConstraint::JacobianDerivative(ndConstraintDescritor&)
{
	//ndAssert(0);
	ndTrace(("error: this joint is an interface\n"));
}

inline bool ndJointBilateralConstraint::GetSkeletonFlag() const
{
	return m_isInSkeleton ? true : false;
}

inline void ndJointBilateralConstraint::SetSkeletonFlag(bool flag)
{
	m_isInSkeleton = ndUnsigned32 (flag ? 1 : 0);
}

inline bool ndJointBilateralConstraint::IsCollidable() const
{
	return m_enableCollision ? true : false;
}

inline bool ndJointBilateralConstraint::IsBilateral() const
{
	return true;
}

inline void ndJointBilateralConstraint::SetCollidable(bool state)
{
	m_enableCollision = ndUnsigned32 (state ? 1 : 0);
}

inline ndFloat32 ndJointBilateralConstraint::GetDiagonalRegularizer(const ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_diagonalRegularizer[index];
}

inline void ndJointBilateralConstraint::SetDiagonalRegularizer(ndConstraintDescritor& desc, ndFloat32 regularizer)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	desc.m_diagonalRegularizer[index] = ndClamp(regularizer, ndFloat32(0.0f), ndFloat32(1.0f));
}

inline ndFloat32 ndJointBilateralConstraint::GetJointErrorPosit(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_penetration[index];
}

inline ndFloat32 ndJointBilateralConstraint::GetJointErrorSpeed(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_jointSpeed[index];
}

inline bool ndJointBilateralConstraint::IsInWorld() const
{
	return m_worldNode ? true : false;
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

inline void ndJointBilateralConstraint::ReplaceSentinel(ndBodyKinematic* const sentinel)
{
	m_body1 = sentinel;
}

inline ndJointBilateralConstraint* ndJointBilateralConstraint::GetAsBilateral()
{ 
	return this; 
}

#endif

