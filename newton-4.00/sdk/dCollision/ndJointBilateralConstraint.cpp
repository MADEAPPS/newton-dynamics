/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndJointBilateralConstraint.h"

#define D_VEL_DAMP				 (dFloat32(100.0f))
#define D_POS_DAMP				 (dFloat32(1500.0f))

ndJointBilateralConstraint::ndJointBilateralConstraint(dInt32 maxDof, ndBodyKinematic* const body0, ndBodyKinematic* const body1, const dMatrix& globalMatrix)
	:ndConstraint()
	,dClassAlloc()
	,m_forceBody0(dVector::m_zero)
	,m_torqueBody0(dVector::m_zero)
	,m_forceBody1(dVector::m_zero)
	,m_torqueBody1(dVector::m_zero)
	,m_body0(body0)
	,m_body1(body1)
	,m_worldNode(nullptr)
	,m_body0Node(nullptr)
	,m_body1Node(nullptr)
{
	dAssert(m_body0 && m_body1);
	dAssert(m_body0 != m_body1);

	if (m_body0->GetInvMass() == dFloat32(0.0f)) 
	{
		dSwap(m_body0, m_body1);
	}
	dAssert(m_body0->GetInvMass() > dFloat32(0.0f));

	CalculateLocalMatrix(globalMatrix, m_localMatrix0, m_localMatrix1);

	m_mark0 = 0;
	m_mark1	= 0;
	m_maxDof = maxDof;
	m_rowIsMotor = 0;
	m_isInSkeleton = 0;
	m_enableCollision = 0;
	m_solverModel = m_jointkinematicOpenLoop;

	m_defualtDiagonalRegularizer = dFloat32(0.0f);
	m_maxAngleError = dFloat32(5.0f) * dDegreeToRad;
	
	memset(m_jointForce, 0, sizeof(m_jointForce));
	memset(m_motorAcceleration, 0, sizeof(m_motorAcceleration));
}

ndJointBilateralConstraint::~ndJointBilateralConstraint()
{
	dAssert(m_worldNode == nullptr);
	dAssert(m_body0Node == nullptr);
	dAssert(m_body1Node == nullptr);
}

void ndJointBilateralConstraint::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	dMatrix matrix0;
	dMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	debugCallback.DrawFrame(matrix0);
	debugCallback.DrawFrame(matrix1);
}

dFloat32 ndJointBilateralConstraint::CalculateAngle(const dVector& dir, const dVector& cosDir, const dVector& sinDir) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dVector projectDir(dir - sinDir * dir.DotProduct(sinDir));
	dFloat32 cosAngle = projectDir.DotProduct(cosDir).GetScalar();
	dFloat32 sinAngle = sinDir.DotProduct(projectDir.CrossProduct(cosDir)).GetScalar();
	return dAtan2(sinAngle, cosAngle);
}

void ndJointBilateralConstraint::CalculateLocalMatrix(const dMatrix& globalMatrix, dMatrix& localMatrix0, dMatrix& localMatrix1) const
{
	dAssert(globalMatrix.TestOrthogonal());
	localMatrix0 = globalMatrix * m_body0->GetMatrix().Inverse();
	localMatrix1 = globalMatrix * m_body1->GetMatrix().Inverse();
}


void ndJointBilateralConstraint::AddLinearRowJacobian(ndConstraintDescritor& desc, const dVector& pivot0, const dVector& pivot1, const dVector& dir)
{
	dgPointParam param;
	InitPointParam(param, m_defualtDiagonalRegularizer, pivot0, pivot1);

	const dInt32 index = desc.m_rowsCount;
	ndForceImpactPair* const jointForce = &m_jointForce[index];
	dAssert(dir.m_w == dFloat32(0.0f));

	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	dVector r0CrossDir(param.m_r0.CrossProduct(dir));
	m_r0[index] = param.m_r0;
	jacobian0.m_linear = dir;
	jacobian0.m_angular = r0CrossDir;
	dAssert(jacobian0.m_linear.m_w == dFloat32(0.0f));
	dAssert(jacobian0.m_angular.m_w == dFloat32(0.0f));
	
	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;
	dVector r1CrossDir(dir.CrossProduct(param.m_r1));
	m_r1[index] = param.m_r1;
	jacobian1.m_linear = dir * dVector::m_negOne;
	jacobian1.m_angular = r1CrossDir;
	dAssert(jacobian1.m_linear.m_w == dFloat32(0.0f));
	dAssert(jacobian1.m_angular.m_w == dFloat32(0.0f));
	
	m_rowIsMotor &= ~(1 << index);
	m_motorAcceleration[index] = dFloat32(0.0f);
	dAssert(desc.m_timestep > dFloat32(0.0f));
	if (desc.m_timestep > dFloat32(0.0f)) 
	{
		dVector positError(param.m_posit1 - param.m_posit0);
		dFloat32 relPosit = positError.DotProduct(dir).GetScalar();
	
		const dVector& veloc0 = m_body0->m_veloc;
		const dVector& veloc1 = m_body1->m_veloc;
		const dVector& omega0 = m_body0->m_omega;
		const dVector& omega1 = m_body1->m_omega;
		const dVector& gyroAlpha0 = m_body0->m_gyroAlpha;
		const dVector& gyroAlpha1 = m_body1->m_gyroAlpha;
		const dVector centripetal0(omega0.CrossProduct(omega0.CrossProduct(m_r0[index])));
		const dVector centripetal1(omega1.CrossProduct(omega1.CrossProduct(m_r1[index])));
	
		const dFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
		const dFloat32 relCentr = -(jacobian0.m_linear * centripetal0 + jacobian1.m_linear * centripetal1).AddHorizontal().GetScalar();
		const dFloat32 relVeloc = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	
		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		const dFloat32 dt = desc.m_timestep;
		const dFloat32 ks = D_POS_DAMP;
		const dFloat32 kd = D_VEL_DAMP;
		const dFloat32 ksd = dt * ks;
		const dFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
		const dFloat32 den = dFloat32(1.0f) + dt * kd + dt * ksd;
		const dFloat32 accelError = num / den;
	
		const dFloat32 relAccel = accelError + relCentr + relGyro;
		desc.m_flags[index] = 0;
		desc.m_penetration[index] = relPosit;
		desc.m_diagonalRegularizer[index] = param.m_defaultDiagonalRegularizer;
		desc.m_jointAccel[index] = relAccel;
		desc.m_penetrationStiffness[index] = relAccel;
		desc.m_restitution[index] = dFloat32(0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
		desc.m_zeroRowAcceleration[index] = relVeloc * desc.m_invTimestep + relGyro;
	}
	else 
	{
		dAssert(0);
	//	const dVector& veloc0 = m_body0->m_veloc;
	//	const dVector& veloc1 = m_body1->m_veloc;
	//	const dVector& omega0 = m_body0->m_omega;
	//	const dVector& omega1 = m_body1->m_omega;
	//	const dFloat32 relVeloc = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
	//
	//	desc.m_flags[index] = 0;
	//	desc.m_penetration[index] = dFloat32(0.0f);
	//	desc.m_diagonalRegularizer[index] = param.m_defualtDiagonalRegularizer;
	//	desc.m_jointAccel[index] = relVeloc;
	//	desc.m_penetrationStiffness[index] = relVeloc;
	//	desc.m_restitution[index] = dFloat32(0.0f);
	//	desc.m_zeroRowAcceleration[index] = dFloat32(0.0f);
	//	desc.m_forceBounds[index].m_jointForce = jointForce;
	}
	desc.m_rowsCount = index + 1;
}

void ndJointBilateralConstraint::AddAngularRowJacobian(ndConstraintDescritor& desc, const dVector& dir, dFloat32 relAngle)
{
	dAssert(dir.m_w == dFloat32(0.0f));
	const dInt32 index = desc.m_rowsCount;
	ndForceImpactPair* const jointForce = &m_jointForce[index];
	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	m_r0[index] = dVector::m_zero;
	jacobian0.m_linear = dVector::m_zero;
	jacobian0.m_angular = dir;
	dAssert(jacobian0.m_angular.m_w == dFloat32(0.0f));

	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;
	dAssert(m_body1);
	m_r1[index] = dVector::m_zero;
	jacobian1.m_linear = dVector::m_zero;
	jacobian1.m_angular = dir * dVector::m_negOne;
	dAssert(jacobian1.m_angular.m_w == dFloat32(0.0f));

	const dVector& omega0 = m_body0->GetOmega();
	const dVector& omega1 = m_body1->GetOmega();
	const dFloat32 relOmega = -(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular).AddHorizontal().GetScalar();

	dAssert(desc.m_timestep > dFloat32(0.0f));
	dAssert(desc.m_invTimestep > dFloat32(0.0f));
	
	m_rowIsMotor &= ~(1 << index);
	m_motorAcceleration[index] = dFloat32(0.0f);
	if (desc.m_timestep > dFloat32(0.0f)) {
		#ifdef _DEBUG
			const dFloat32 relCentr = -(omega0 * omega0.CrossProduct(jacobian0.m_angular) + omega1 * omega1.CrossProduct(jacobian1.m_angular)).AddHorizontal().GetScalar();
			// allow for some large error since this is affected bu numerical precision a lot
			dAssert(dAbs(relCentr) < dFloat32(4.0f));
		#endif

		const dVector& gyroAlpha0 = m_body0->m_gyroAlpha;
		const dVector& gyroAlpha1 = m_body1->m_gyroAlpha;
		const dFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();

		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		dFloat32 dt = desc.m_timestep;
		dFloat32 ks = D_POS_DAMP;
		dFloat32 kd = D_VEL_DAMP;
		dFloat32 ksd = dt * ks;
		dFloat32 num = ks * relAngle + kd * relOmega + ksd * relOmega;
		dFloat32 den = dFloat32(1.0f) + dt * kd + dt * ksd;
		dFloat32 alphaError = num / den;

		desc.m_flags[index] = 0;
		desc.m_penetration[index] = relAngle;
		desc.m_diagonalRegularizer[index] = m_defualtDiagonalRegularizer;
		desc.m_jointAccel[index] = alphaError + relGyro;
		desc.m_penetrationStiffness[index] = alphaError + relGyro;
		desc.m_restitution[index] = dFloat32(0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
		desc.m_zeroRowAcceleration[index] = relOmega * desc.m_invTimestep + relGyro;
	}
	else 
	{
		dAssert(0);
		//desc.m_flags[index] = 0;
		//desc.m_penetration[index] = dFloat32(0.0f);
		//desc.m_restitution[index] = dFloat32(0.0f);
		//desc.m_diagonalRegularizer[index] = stiffness;
		//desc.m_jointAccel[index] = relOmega;
		//desc.m_penetrationStiffness[index] = relOmega;;
		//desc.m_zeroRowAcceleration[index] = dFloat32(0.0f);
		//desc.m_forceBounds[index].m_jointForce = jointForce;
	}
	desc.m_rowsCount = index + 1;
}


void ndJointBilateralConstraint::JointAccelerations(ndJointAccelerationDecriptor* const desc)
{
	const dVector& bodyVeloc0 = m_body0->m_veloc;
	const dVector& bodyOmega0 = m_body0->m_omega;
	const dVector& bodyVeloc1 = m_body1->m_veloc;
	const dVector& bodyOmega1 = m_body1->m_omega;
	const dVector& gyroAlpha0 = m_body0->m_gyroAlpha;
	const dVector& gyroAlpha1 = m_body1->m_gyroAlpha;

	ndRightHandSide* const rhs = desc->m_rightHandSide;
	const ndLeftHandSide* const row = desc->m_leftHandSide;
	dAssert(desc->m_timestep > dFloat32(0.0f));
	dAssert(desc->m_invTimestep > dFloat32(0.0f));

	if (desc->m_timestep > dFloat32(0.0f)) 
	{
		const dFloat32 ks = D_POS_DAMP * dFloat32(0.5f);
		const dFloat32 kd = D_VEL_DAMP * dFloat32(4.0f);
		const dFloat32 dt = desc->m_timestep;
		for (dInt32 k = 0; k < desc->m_rowsCount; k++) 
		{
			if (m_rowIsMotor & (1 << k)) 
			{
				rhs[k].m_coordenateAccel = m_motorAcceleration[k] + rhs[k].m_deltaAccel;
			}
			else 
			{
				const ndJacobianPair& Jt = row[k].m_Jt;

				//calculate internal centripetal each sub step 
				const dVector centripetal0(bodyOmega0.CrossProduct(bodyOmega0.CrossProduct(m_r0[k])));
				const dVector centripetal1(bodyOmega1.CrossProduct(bodyOmega1.CrossProduct(m_r1[k])));

				const dVector relVeloc(
					Jt.m_jacobianM0.m_linear * bodyVeloc0 + Jt.m_jacobianM0.m_angular * bodyOmega0 +
					Jt.m_jacobianM1.m_linear * bodyVeloc1 + Jt.m_jacobianM1.m_angular * bodyOmega1);
				const dVector relGyro (Jt.m_jacobianM0.m_angular * gyroAlpha0 + Jt.m_jacobianM1.m_angular * gyroAlpha1);
				const dVector relCentr (Jt.m_jacobianM0.m_linear * centripetal0 + Jt.m_jacobianM1.m_linear * centripetal1);
				const dFloat32 vRel = relVeloc.AddHorizontal().GetScalar();

				//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
				//alphaError = num / den;
				//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
				//dFloat32 dt = desc.m_timestep;
				//dFloat32 ks = D_POS_DAMP;
				//dFloat32 kd = D_VEL_DAMP;
				//dFloat32 ksd = dt * ks;
				//dFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
				//dFloat32 den = dFloat32 (1.0f) + dt * kd + dt * ksd;
				//accelError = num / den;

				const dFloat32 relPosit = rhs[k].m_penetration - vRel * dt * desc->m_firstPassCoefFlag;
				rhs[k].m_penetration = relPosit;

				const dFloat32 ksd = dt * ks;
				const dFloat32 num = ks * relPosit - kd * vRel - ksd * vRel;
				const dFloat32 den = dFloat32(1.0f) + dt * kd + dt * ksd;
				const dFloat32 aRelErr = num / den;
				rhs[k].m_coordenateAccel = rhs[k].m_deltaAccel + aRelErr + (relGyro - relCentr).AddHorizontal().GetScalar();
			}
		}
	}
	else 
	{
		dAssert(0);
		for (dInt32 k = 0; k < desc->m_rowsCount; k++) 
		{
			if (m_rowIsMotor & (1 << k)) 
			{
				rhs[k].m_coordenateAccel = m_motorAcceleration[k] + rhs[k].m_deltaAccel;
			}
			else 
			{
				const ndJacobianPair& Jt = row[k].m_Jt;
				dVector relVeloc(
					Jt.m_jacobianM0.m_linear * bodyVeloc0 + Jt.m_jacobianM0.m_angular * bodyOmega0 +
					Jt.m_jacobianM1.m_linear * bodyVeloc1 + Jt.m_jacobianM1.m_angular * bodyOmega1);

				dFloat32 vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
				rhs[k].m_coordenateAccel = rhs[k].m_deltaAccel - vRel;
			}
		}
	}
}

dFloat32 ndJointBilateralConstraint::CalculateSpringDamperAcceleration(dFloat32 dt, dFloat32 ks, dFloat32 x, dFloat32 kd, dFloat32 v) const
{
	//at = - (ks * x + kd * v);
	//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
	dFloat32 ksd = dt * ks;
	dFloat32 num = ks * x + kd * v + ksd * v;
	dFloat32 den = dFloat32(1.0f) + dt * kd + dt * ksd;
	dAssert(den > 0.0f);
	dFloat32 accel = -num / den;
	return accel;
}

void ndJointBilateralConstraint::SetMassSpringDamperAcceleration(ndConstraintDescritor& desc, dFloat32 regularizer, dFloat32 spring, dFloat32 damper)
{
	const dInt32 index = desc.m_rowsCount - 1;
	dAssert(index >= 0);
	dAssert(index < dInt32(m_maxDof));
	
	const dVector& veloc0 = m_body0->m_veloc;
	const dVector& omega0 = m_body0->m_omega;
	const dVector& veloc1 = m_body1->m_veloc;
	const dVector& omega1 = m_body1->m_omega;
	const ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0;
	const ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1;

	const dFloat32 relPosit = desc.m_penetration[index];
	const dFloat32 relVeloc = -(veloc0.DotProduct(jacobian0.m_linear) + veloc1.DotProduct(jacobian1.m_linear) + omega0.DotProduct(jacobian0.m_angular) + omega1.DotProduct(jacobian1.m_angular)).GetScalar();

	const dFloat32 r = dClamp(regularizer, dFloat32(1.e-3f), dFloat32(0.99f));
	const dFloat32 accel = -CalculateSpringDamperAcceleration(desc.m_timestep, spring, relPosit, damper, relVeloc);
	
	desc.m_diagonalRegularizer[index] = r;
	SetMotorAcceleration(desc, accel);
}
