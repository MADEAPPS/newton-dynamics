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

D_MSV_NEWTON_CLASS_ALIGN_32
class ndJointBilateralConstraint : public ndConstraint
{
	public:
	D_BASE_CLASS_REFLECTION(ndJointBilateralConstraint);

	class ndIkInterface
	{
		public:
		ndIkInterface()
			:m_defualtRegularizer(ndFloat32 (1.0e-3f))
			,m_ikMode(true)
		{}

		ndJacobian m_accel0;
		ndJacobian m_accel1;
		ndFloat32 m_defualtRegularizer;
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

	D_COLLISION_API virtual ndJointBilateralConstraint* GetAsBilateral();
	D_COLLISION_API virtual void JacobianDerivative(ndConstraintDescritor& desc);
	D_COLLISION_API virtual ndJointBilateralSolverModel GetSolverModel() const;
	D_COLLISION_API virtual void SetSolverModel(ndJointBilateralSolverModel model);

	D_COLLISION_API ndFloat32 CalculateAngle(const ndVector& planeDir, const ndVector& cosDir, const ndVector& sinDir) const;
	D_COLLISION_API virtual void JointAccelerations(ndJointAccelerationDecriptor* const desc);
	D_COLLISION_API void AddAngularRowJacobian(ndConstraintDescritor& desc, const ndVector& dir, ndFloat32 relAngle);
	D_COLLISION_API void AddLinearRowJacobian(ndConstraintDescritor& desc, const ndVector& pivot0, const ndVector& pivot1, const ndVector& dir);

	D_COLLISION_API virtual void DebugJoint(ndConstraintDebugCallback& debugCallback) const;
	D_COLLISION_API ndFloat32 CalculateSpringDamperAcceleration(ndFloat32 dt, ndFloat32 ks, ndFloat32 x, ndFloat32 kd, ndFloat32 v) const;
	D_COLLISION_API void SetMassSpringDamperAcceleration(ndConstraintDescritor& desc, ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper);

	D_COLLISION_API virtual ndInt32 GetKinematicState(ndKinematicState* const state) const;

	D_COLLISION_API const ndMatrix& GetLocalMatrix0() const;
	D_COLLISION_API const ndMatrix& GetLocalMatrix1() const;

	D_COLLISION_API void SetLocalMatrix0(const ndMatrix& matrix);
	D_COLLISION_API void SetLocalMatrix1(const ndMatrix& matrix);
	D_COLLISION_API virtual ndMatrix CalculateGlobalMatrix0() const;
	D_COLLISION_API virtual ndMatrix CalculateGlobalMatrix1() const;

	D_COLLISION_API virtual void CalculateGlobalMatrix(ndMatrix& matrix0, ndMatrix& matrix1) const;
	D_COLLISION_API void CalculateLocalMatrix(const ndMatrix& pinsAndPivotFrame, ndMatrix& localMatrix0, ndMatrix& localMatrix1) const;

	D_COLLISION_API bool IsInWorld() const;
	D_COLLISION_API bool IsSkeleton() const;
	D_COLLISION_API bool IsBilateral() const;
	D_COLLISION_API bool IsCollidable() const;
	D_COLLISION_API bool GetSkeletonFlag() const;
	D_COLLISION_API void SetCollidable(bool state);
	D_COLLISION_API void SetSkeletonFlag(bool flag);

	D_COLLISION_API bool GetJointHitLimits() const;

	D_COLLISION_API void SetHighFriction(ndConstraintDescritor& desc, ndFloat32 friction);
	D_COLLISION_API void SetLowerFriction(ndConstraintDescritor& desc, ndFloat32 friction);
	D_COLLISION_API void SetJointErrorPosit(ndConstraintDescritor& desc, ndFloat32 errorPosit);
	D_COLLISION_API void SetMotorAcceleration(ndConstraintDescritor& desc, ndFloat32 acceleration);
	D_COLLISION_API void SetDiagonalRegularizer(ndConstraintDescritor& desc, ndFloat32 regularizer);

	D_COLLISION_API ndFloat32 GetJointErrorPosit(ndConstraintDescritor& desc) const;
	D_COLLISION_API ndFloat32 GetJointErrorSpeed(ndConstraintDescritor& desc) const;
	D_COLLISION_API ndFloat32 GetMotorAcceleration(ndConstraintDescritor& desc) const;
	D_COLLISION_API ndFloat32 GetMotorZeroAcceleration(ndConstraintDescritor& desc) const;
	D_COLLISION_API ndFloat32 GetDiagonalRegularizer(const ndConstraintDescritor& desc) const;

	D_COLLISION_API void ReplaceSentinel(ndBodyKinematic* const sentinel);

	// inverse dynamics interface
	D_COLLISION_API virtual void ClearMemory();
	D_COLLISION_API virtual void SetIkMode(bool mode);
	D_COLLISION_API virtual void SetIkSetAccel(const ndJacobian& body0Accel, const ndJacobian& body1Accel);

	protected:
	ndMatrix m_localMatrix0;
	ndMatrix m_localMatrix1;

	ndVector m_r0[ND_BILATERAL_CONTRAINT_DOF];
	ndVector m_r1[ND_BILATERAL_CONTRAINT_DOF];
	ndForceImpactPair m_jointForce[ND_BILATERAL_CONTRAINT_DOF];
	ndFloat32 m_motorAcceleration[ND_BILATERAL_CONTRAINT_DOF];
	ndJointList::ndNode* m_worldNode;
	ndBodyKinematic::ndJointList::ndNode* m_body0Node;
	ndBodyKinematic::ndJointList::ndNode* m_body1Node;
	//ndSpecialList<ndJointBilateralConstraint>::ndNode* m_deletedNode;

	ndFloat32 m_defualtDiagonalRegularizer;
	ndUnsigned32 m_mark0			: 1;
	ndUnsigned32 m_mark1			: 1;
	ndUnsigned32 m_isInSkeleton		: 1;
	ndUnsigned32 m_enableCollision	: 1;
	ndInt8 m_rowIsMotor;
	ndInt8 m_hitLimits;
	ndJointBilateralSolverModel m_solverModel;
	
	friend class ndWorld;
	friend class ndIkSolver;
	friend class ndDynamicsUpdate;
	friend class ndModelArticulation;
	friend class ndSkeletonContainer;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateSycl;
	friend class ndDynamicsUpdateCuda;
} D_GCC_NEWTON_CLASS_ALIGN_32;


#endif

