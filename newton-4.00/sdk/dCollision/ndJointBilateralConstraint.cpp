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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndBodyKinematic.h"
#include "ndJointBilateralConstraint.h"

#define D_VEL_DAMP			 ndFloat32(100.0f)
#define D_POS_DAMP			 ndFloat32(1500.0f)

ndJointBilateralConstraint::ndJointBilateralConstraint()
	:ndConstraint()
	,m_worldNode(nullptr)
	,m_body0Node(nullptr)
	,m_body1Node(nullptr)
	//,m_deletedNode(nullptr)
{
	m_mark0 = 0;
	m_mark1 = 0;
	m_maxDof = 0;
	m_hitLimits = 0;
	m_rowIsMotor = 0;
	m_isInSkeleton = 0;
	m_enableCollision = 0;
	m_solverModel = m_jointkinematicOpenLoop;
	m_defualtDiagonalRegularizer = ndFloat32(0.0f);

	ndMemSet(m_jointForce, ndForceImpactPair(), sizeof(m_jointForce) / sizeof(m_jointForce[0]));
	ndMemSet(m_motorAcceleration, ndFloat32 (0.0f), sizeof(m_motorAcceleration) / sizeof (m_motorAcceleration[0]));
}

ndJointBilateralConstraint::ndJointBilateralConstraint(ndInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const ndMatrix& globalMatrix)
	:ndConstraint()
	,m_worldNode(nullptr)
	,m_body0Node(nullptr)
	,m_body1Node(nullptr)
	//,m_deletedNode(nullptr)
{
	m_body0 = body0;
	m_body1 = body1;
	ndAssert(m_body0 && m_body1);
	ndAssert(m_body0 != m_body1);

	if (m_body0->GetInvMass() == ndFloat32(0.0f)) 
	{
		ndSwap(m_body0, m_body1);
	}
	ndAssert(m_body0->GetInvMass() > ndFloat32(0.0f));

	CalculateLocalMatrix(globalMatrix, m_localMatrix0, m_localMatrix1);

	m_mark0 = 0;
	m_mark1	= 0;
	m_hitLimits = 0;
	m_rowIsMotor = 0;
	m_isInSkeleton = 0;
	m_enableCollision = 0;
	m_maxDof = ndUnsigned8(maxDof);
	m_solverModel = m_jointkinematicOpenLoop;
	m_defualtDiagonalRegularizer = ndFloat32(0.0f);
	
	ndMemSet(m_jointForce, ndForceImpactPair(), sizeof(m_jointForce) / sizeof(m_jointForce[0]));
	ndMemSet(m_motorAcceleration, ndFloat32(0.0f), sizeof(m_motorAcceleration) / sizeof(m_motorAcceleration[0]));
}

ndJointBilateralConstraint::ndJointBilateralConstraint(ndInt32 maxDof, ndBodyKinematic* const body0,
	ndBodyKinematic* const body1, const ndMatrix& globalMatrixBody0, const ndMatrix& globalMatrixBody1)
	:ndConstraint()
	,m_worldNode(nullptr)
	,m_body0Node(nullptr)
	,m_body1Node(nullptr)
	//,m_deletedNode(nullptr)
{
	m_body0 = body0;
	m_body1 = body1;
	ndAssert(m_body0 && m_body1);
	ndAssert(m_body0 != m_body1);

	if (m_body0->GetInvMass() == ndFloat32(0.0f))
	{
		ndSwap(m_body0, m_body1);
	}
	ndAssert(m_body0->GetInvMass() > ndFloat32(0.0f));

	ndMatrix dummyMatrix;
	CalculateLocalMatrix(globalMatrixBody0, m_localMatrix0, dummyMatrix);
	CalculateLocalMatrix(globalMatrixBody1, dummyMatrix, m_localMatrix1);

	m_mark0 = 0;
	m_mark1 = 0;
	m_hitLimits = 0;
	m_rowIsMotor = 0;
	m_isInSkeleton = 0;
	m_enableCollision = 0;
	m_maxDof = ndUnsigned8(maxDof);
	m_solverModel = m_jointkinematicOpenLoop;
	m_defualtDiagonalRegularizer = ndFloat32(0.0f);

	ndMemSet(m_jointForce, ndForceImpactPair(), sizeof(m_jointForce) / sizeof(m_jointForce[0]));
	ndMemSet(m_motorAcceleration, ndFloat32(0.0f), sizeof(m_motorAcceleration) / sizeof(m_motorAcceleration[0]));
}

ndJointBilateralConstraint::~ndJointBilateralConstraint()
{
	//ndAssert(m_worldNode == nullptr);
	//ndAssert(m_body0Node == nullptr);
	//ndAssert(m_body1Node == nullptr);
	//ndAssert(m_deletedNode == nullptr);
}

ndJointBilateralSolverModel ndJointBilateralConstraint::GetSolverModel() const
{
	return m_solverModel;
}

void ndJointBilateralConstraint::SetSolverModel(ndJointBilateralSolverModel model)
{
	ndAssert(model < m_jointModesCount);
	ndAssert(model >= m_jointIterativeSoft);
	m_solverModel = ndClamp(model, m_jointIterativeSoft, m_jointModesCount);
}

//ndUnsigned32 ndJointBilateralConstraint::GetRowsCount() const
//{
//	return m_maxDof;
//}

const ndMatrix& ndJointBilateralConstraint::GetLocalMatrix0() const
{
	return m_localMatrix0;
}

const ndMatrix& ndJointBilateralConstraint::GetLocalMatrix1() const
{
	return m_localMatrix1;
}

void ndJointBilateralConstraint::SetLocalMatrix0(const ndMatrix& matrix)
{
	m_localMatrix0 = matrix;
}

void ndJointBilateralConstraint::SetLocalMatrix1(const ndMatrix& matrix)
{
	m_localMatrix1 = matrix;
}


ndFloat32 ndJointBilateralConstraint::GetMotorZeroAcceleration(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_zeroRowAcceleration[index];
}

void ndJointBilateralConstraint::SetMotorAcceleration(ndConstraintDescritor& desc, ndFloat32 acceleration)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	m_rowIsMotor |= (1 << index);
	desc.m_flags[index] = 0;
	m_motorAcceleration[index] = acceleration;
	desc.m_jointAccel[index] = acceleration;
}

ndFloat32 ndJointBilateralConstraint::GetMotorAcceleration(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_jointAccel[index];
}

void ndJointBilateralConstraint::SetJointErrorPosit(ndConstraintDescritor& desc, ndFloat32 errorPosit)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	desc.m_penetration[index] = errorPosit;
}

void ndJointBilateralConstraint::SetLowerFriction(ndConstraintDescritor& desc, ndFloat32 friction)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	desc.m_forceBounds[index].m_low = ndClamp(friction, ndFloat32(D_MIN_BOUND), ndFloat32(-0.001f));
	//ndAssert(desc.m_forceBounds[index].m_normalIndex == D_INDEPENDENT_ROW);
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

void ndJointBilateralConstraint::SetHighFriction(ndConstraintDescritor& desc, ndFloat32 friction)
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

void ndJointBilateralConstraint::JacobianDerivative(ndConstraintDescritor&)
{
	//ndAssert(0);
	ndTrace(("error: this joint is an interface\n"));
}

bool ndJointBilateralConstraint::GetSkeletonFlag() const
{
	return m_isInSkeleton ? true : false;
}

void ndJointBilateralConstraint::SetSkeletonFlag(bool flag)
{
	m_isInSkeleton = ndUnsigned32(flag ? 1 : 0);
}

bool ndJointBilateralConstraint::IsCollidable() const
{
	return m_enableCollision ? true : false;
}

bool ndJointBilateralConstraint::IsBilateral() const
{
	return true;
}

void ndJointBilateralConstraint::SetCollidable(bool state)
{
	m_enableCollision = ndUnsigned32(state ? 1 : 0);
}

ndFloat32 ndJointBilateralConstraint::GetDiagonalRegularizer(const ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_diagonalRegularizer[index];
}

void ndJointBilateralConstraint::SetDiagonalRegularizer(ndConstraintDescritor& desc, ndFloat32 regularizer)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	desc.m_diagonalRegularizer[index] = ndClamp(regularizer, ndFloat32(0.0f), ndFloat32(1.0f));
}

ndFloat32 ndJointBilateralConstraint::GetJointErrorPosit(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_penetration[index];
}

ndFloat32 ndJointBilateralConstraint::GetJointErrorSpeed(ndConstraintDescritor& desc) const
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	return desc.m_jointSpeed[index];
}

bool ndJointBilateralConstraint::IsInWorld() const
{
	return m_worldNode ? true : false;
}

bool ndJointBilateralConstraint::IsSkeleton() const
{
	const ndJointBilateralSolverModel mode = GetSolverModel();
	bool test = false;
	test = test || (mode == m_jointkinematicOpenLoop);
	test = test || (mode == m_jointkinematicCloseLoop);
	//test = test || (mode == m_jointkinematicHintOpenLoop);
	return test;
}

bool ndJointBilateralConstraint::GetJointHitLimits() const
{
	return m_hitLimits ? true : false;
}

void ndJointBilateralConstraint::ReplaceSentinel(ndBodyKinematic* const sentinel)
{
	m_body1 = sentinel;
}

ndJointBilateralConstraint* ndJointBilateralConstraint::GetAsBilateral()
{
	return this;
}


void ndJointBilateralConstraint::SetIkMode(bool)
{
}

void ndJointBilateralConstraint::SetIkSetAccel(const ndJacobian&, const ndJacobian&)
{
}

void ndJointBilateralConstraint::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);
}

ndFloat32 ndJointBilateralConstraint::CalculateAngle(const ndVector& pin, const ndVector& cosDir, const ndVector& sinDir) const
{
	const ndVector dir(pin & ndVector::m_triplexMask);
	ndVector projectDir(dir - sinDir * dir.DotProduct(sinDir));
	ndFloat32 cosAngle = projectDir.DotProduct(cosDir).GetScalar();
	ndFloat32 sinAngle = sinDir.DotProduct(projectDir.CrossProduct(cosDir)).GetScalar();
	return ndAtan2(sinAngle, cosAngle);
}

void ndJointBilateralConstraint::CalculateLocalMatrix(const ndMatrix& globalMatrix, ndMatrix& localMatrix0, ndMatrix& localMatrix1) const
{
	ndAssert(globalMatrix.TestOrthogonal());
	localMatrix0 = globalMatrix * m_body0->GetMatrix().OrthoInverse();
	localMatrix1 = globalMatrix * m_body1->GetMatrix().OrthoInverse();
}

void ndJointBilateralConstraint::CalculateGlobalMatrix(ndMatrix& matrix0, ndMatrix& matrix1) const
{
	matrix0 = m_localMatrix0 * m_body0->GetMatrix();
	matrix1 = m_localMatrix1 * m_body1->GetMatrix();
}

ndMatrix ndJointBilateralConstraint::CalculateGlobalMatrix0() const
{
	return m_localMatrix0 * m_body0->GetMatrix();
}

ndMatrix ndJointBilateralConstraint::CalculateGlobalMatrix1() const
{
	return m_localMatrix1 * m_body1->GetMatrix();
}

ndFloat32 ndJointBilateralConstraint::CalculateSpringDamperAcceleration(ndFloat32 dt, ndFloat32 ks, ndFloat32 x, ndFloat32 kd, ndFloat32 v) const
{
	//at = - (ks * x + kd * v);
	//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	ndFloat32 ksd = dt * ks;
	ndFloat32 num = ks * x + kd * v + ksd * v;
	ndFloat32 den = ndFloat32(1.0f) + dt * kd + dt * ksd;
	ndAssert(den > 0.0f);
	ndFloat32 accel = -num / den;
	return accel;
}

void ndJointBilateralConstraint::SetMassSpringDamperAcceleration(ndConstraintDescritor& desc, ndFloat32 regularizer, ndFloat32 spring, ndFloat32 damper)
{
	const ndInt32 index = desc.m_rowsCount - 1;
	ndAssert(index >= 0);
	ndAssert(index < ndInt32(m_maxDof));
	
	const ndVector& veloc0 = m_body0->m_veloc;
	const ndVector& omega0 = m_body0->m_omega;
	const ndVector& veloc1 = m_body1->m_veloc;
	const ndVector& omega1 = m_body1->m_omega;
	const ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	const ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;

	const ndFloat32 relPosit = desc.m_penetration[index];
	const ndFloat32 relVeloc = (jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	const ndFloat32 accel = CalculateSpringDamperAcceleration(desc.m_timestep, spring, -relPosit, damper, relVeloc);

	const ndFloat32 r = ndClamp(regularizer, ndFloat32(1.e-8f), ndFloat32(0.9f));
	desc.m_diagonalRegularizer[index] = r;
	SetMotorAcceleration(desc, accel);
}

void ndJointBilateralConstraint::JointAccelerations(ndJointAccelerationDecriptor* const desc)
{
	const ndVector& bodyVeloc0 = m_body0->m_veloc;
	const ndVector& bodyOmega0 = m_body0->m_omega;
	const ndVector& bodyVeloc1 = m_body1->m_veloc;
	const ndVector& bodyOmega1 = m_body1->m_omega;
	const ndVector gyroAlpha0(m_body0->GetGyroAlpha());
	const ndVector gyroAlpha1(m_body1->GetGyroAlpha());

	ndRightHandSide* const rhs = desc->m_rightHandSide;
	const ndLeftHandSide* const row = desc->m_leftHandSide;
	ndAssert(desc->m_timestep > ndFloat32(0.0f));
	ndAssert(desc->m_invTimestep > ndFloat32(0.0f));

	const ndFloat32 dt = desc->m_timestep;
	const ndFloat32 ks = D_POS_DAMP * ndFloat32(0.5f);
	const ndFloat32 kd = D_VEL_DAMP * ndFloat32(4.0f);
	for (ndInt32 k = 0; k < desc->m_rowsCount; ++k)
	{
		if (m_rowIsMotor & (1 << k))
		{
			const ndFloat32 accel = m_motorAcceleration[k] + rhs[k].m_deltaAccel;
			rhs[k].m_coordenateAccel = accel;
		}
		else
		{
			const ndJacobianPair& Jt = row[k].m_Jt;

			//calculate internal centripetal each sub step 
			const ndVector centripetal0(bodyOmega0.CrossProduct(bodyOmega0.CrossProduct(m_r0[k])));
			const ndVector centripetal1(bodyOmega1.CrossProduct(bodyOmega1.CrossProduct(m_r1[k])));

			const ndVector relVeloc(
				Jt.m_jacobianM0.m_linear * bodyVeloc0 + Jt.m_jacobianM0.m_angular * bodyOmega0 +
				Jt.m_jacobianM1.m_linear * bodyVeloc1 + Jt.m_jacobianM1.m_angular * bodyOmega1);
			const ndVector relGyro(Jt.m_jacobianM0.m_angular * gyroAlpha0 + Jt.m_jacobianM1.m_angular * gyroAlpha1);
			const ndVector relCentr(Jt.m_jacobianM0.m_linear * centripetal0 + Jt.m_jacobianM1.m_linear * centripetal1);
			const ndFloat32 vRel = relVeloc.AddHorizontal().GetScalar();

			const ndFloat32 relPosit = rhs[k].m_penetration - vRel * dt * desc->m_firstPassCoefFlag;
			const ndFloat32 ksd = dt * ks;
			const ndFloat32 num = ks * relPosit - kd * vRel - ksd * vRel;
			const ndFloat32 den = ndFloat32(1.0f) + dt * kd + dt * ksd;
			const ndFloat32 aRelErr = num / den;
			const ndFloat32 accel = rhs[k].m_deltaAccel + aRelErr + (relGyro - relCentr).AddHorizontal().GetScalar();

			rhs[k].m_penetration = relPosit;
			rhs[k].m_coordenateAccel = accel;
		}
	}
}

void ndJointBilateralConstraint::ClearMemory()
{
	for (ndInt32 i = 0; i < sizeof(m_jointForce) / sizeof(m_jointForce[0]); ++i)
	{
		m_jointForce[i].Clear();
		m_motorAcceleration[i] = ndFloat32 (0.0f);
	}
}

void ndJointBilateralConstraint::AddLinearRowJacobian(ndConstraintDescritor& desc, const ndVector& pivot0, const ndVector& pivot1, const ndVector& dir)
{
	ndPointParam param;
	InitPointParam(param, pivot0, pivot1);

	const ndInt32 index = desc.m_rowsCount;
	ndAssert(dir.m_w == ndFloat32(0.0f));

	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	ndVector r0CrossDir(param.m_r0.CrossProduct(dir));
	m_r0[index] = param.m_r0;
	jacobian0.m_linear = dir;
	jacobian0.m_angular = r0CrossDir;
	ndAssert(jacobian0.m_linear.m_w == ndFloat32(0.0f));
	ndAssert(jacobian0.m_angular.m_w == ndFloat32(0.0f));
	
	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;
	ndVector r1CrossDir(dir.CrossProduct(param.m_r1));
	m_r1[index] = param.m_r1;
	jacobian1.m_linear = dir * ndVector::m_negOne;
	jacobian1.m_angular = r1CrossDir;
	ndAssert(jacobian1.m_linear.m_w == ndFloat32(0.0f));
	ndAssert(jacobian1.m_angular.m_w == ndFloat32(0.0f));
	
	m_rowIsMotor &= ~(1 << index);
	m_motorAcceleration[index] = ndFloat32(0.0f);
	ndAssert(desc.m_timestep > ndFloat32(0.0f));
	ndForceImpactPair* const jointForce = &m_jointForce[index];

	const ndVector& omega0 = m_body0->m_omega;
	const ndVector& omega1 = m_body1->m_omega;
	const ndVector& veloc0 = m_body0->m_veloc;
	const ndVector& veloc1 = m_body1->m_veloc;

	const ndVector gyroAlpha0(m_body0->GetGyroAlpha());
	const ndVector gyroAlpha1(m_body1->GetGyroAlpha());
	const ndVector centripetal0(omega0.CrossProduct(omega0.CrossProduct(m_r0[index])));
	const ndVector centripetal1(omega1.CrossProduct(omega1.CrossProduct(m_r1[index])));

	const ndFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
	const ndFloat32 relCentr = -(jacobian0.m_linear * centripetal0 + jacobian1.m_linear * centripetal1).AddHorizontal().GetScalar();
	const ndFloat32 relPosit = -(jacobian0.m_linear * param.m_posit0 + jacobian1.m_linear * param.m_posit1).AddHorizontal().GetScalar();
	const ndFloat32 relVeloc = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();

	//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	const ndFloat32 dt = desc.m_timestep;
	const ndFloat32 ks = D_POS_DAMP;
	const ndFloat32 kd = D_VEL_DAMP;
	const ndFloat32 ksd = dt * ks;
	const ndFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
	const ndFloat32 den = ndFloat32(1.0f) + dt * kd + dt * ksd;
	const ndFloat32 accelError = num / den;

	const ndFloat32 relAccel = accelError + relCentr + relGyro;
	desc.m_flags[index] = 0;
	desc.m_jointAccel[index] = relAccel;
	desc.m_penetration[index] = relPosit;
	desc.m_jointSpeed[index] = relVeloc;
	desc.m_restitution[index] = ndFloat32(0.0f);
	desc.m_penetrationStiffness[index] = ndFloat32(0.0f);
	desc.m_forceBounds[index].m_jointForce = jointForce;
	desc.m_diagonalRegularizer[index] = m_defualtDiagonalRegularizer;
	desc.m_zeroRowAcceleration[index] = relVeloc * desc.m_invTimestep + relGyro;
	desc.m_rowsCount = index + 1;
}

void ndJointBilateralConstraint::AddAngularRowJacobian(ndConstraintDescritor& desc, const ndVector& dir, ndFloat32 relAngle)
{
	ndAssert(dir.m_w == ndFloat32(0.0f));
	const ndInt32 index = desc.m_rowsCount;
	ndForceImpactPair* const jointForce = &m_jointForce[index];
	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	m_r0[index] = ndVector::m_zero;
	jacobian0.m_linear = ndVector::m_zero;
	jacobian0.m_angular = dir;
	ndAssert(jacobian0.m_angular.m_w == ndFloat32(0.0f));

	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;
	ndAssert(m_body1);
	m_r1[index] = ndVector::m_zero;
	jacobian1.m_linear = ndVector::m_zero;
	jacobian1.m_angular = dir * ndVector::m_negOne;
	ndAssert(jacobian1.m_angular.m_w == ndFloat32(0.0f));

	const ndVector omega0 (m_body0->GetOmega());
	const ndVector omega1 (m_body1->GetOmega());
	const ndVector gyroAlpha0(m_body0->GetGyroAlpha());
	const ndVector gyroAlpha1(m_body1->GetGyroAlpha());

	const ndFloat32 relOmega = -(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular).AddHorizontal().GetScalar();

	ndAssert(desc.m_timestep > ndFloat32(0.0f));
	ndAssert(desc.m_invTimestep > ndFloat32(0.0f));
	
	m_rowIsMotor &= ~(1 << index);
	m_motorAcceleration[index] = ndFloat32(0.0f);
	ndAssert(desc.m_timestep);

	const ndFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();

	//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	ndFloat32 dt = desc.m_timestep;
	ndFloat32 ks = D_POS_DAMP;
	ndFloat32 kd = D_VEL_DAMP;
	ndFloat32 ksd = dt * ks;
	ndFloat32 num = ks * relAngle + kd * relOmega + ksd * relOmega;
	ndFloat32 den = ndFloat32(1.0f) + dt * kd + dt * ksd;
	ndFloat32 alphaError = num / den;

	desc.m_flags[index] = 0;
	desc.m_jointSpeed[index] = relOmega;
	desc.m_penetration[index] = relAngle;
	desc.m_jointAccel[index] = alphaError + relGyro;
	desc.m_restitution[index] = ndFloat32(0.0f);
	desc.m_penetrationStiffness[index] = ndFloat32(0.0f);
	desc.m_forceBounds[index].m_jointForce = jointForce;
	desc.m_diagonalRegularizer[index] = m_defualtDiagonalRegularizer;
	desc.m_zeroRowAcceleration[index] = relOmega * desc.m_invTimestep + relGyro;
	desc.m_rowsCount = index + 1;
}

ndInt32 ndJointBilateralConstraint::GetKinematicState(ndKinematicState* const) const
{
	ndAssert(0);
	return 0;
}
