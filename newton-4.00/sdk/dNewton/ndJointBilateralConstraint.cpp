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


#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointBilateralConstraint.h"


//#include "dgBody.h"
//#include "dgWorld.h"
//#include "dgConstraint.h"
//#include "dgWorldDynamicUpdate.h"
//#include "ndJointBilateralConstraint.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_VEL_DAMP				 (dFloat32(100.0f))
#define DG_POS_DAMP				 (dFloat32(1500.0f))

#if 0

void dBilateralConstraint::AppendToJointList()
{
	dAssert(m_body0);
	dAssert(!m_jointNode);
	
	dgBilateralConstraintList* const jointList = m_body0->m_world;
	m_jointNode = jointList->Addtop(this);
}

dInt32 dBilateralConstraint::GetSolverModel() const
{
	return m_solverModel;
}

dFloat32 dBilateralConstraint::GetMassScaleBody0() const
{
	return m_massScaleBody0;
}

dFloat32 dBilateralConstraint::GetMassScaleBody1() const
{
	return m_massScaleBody1;
}

void dBilateralConstraint::SetSolverModel(dInt32 model)
{
	m_solverModel = dgClamp(model, 0, 3);
}

dFloat32 dBilateralConstraint::GetStiffness() const
{
	return m_defualtDiagonalRegularizer;
}

void dBilateralConstraint::SetStiffness(dFloat32 stiffness)
{
	m_defualtDiagonalRegularizer = dgClamp (stiffness, dFloat32(0.0f), dFloat32(1.0f));
}

void dBilateralConstraint::SetDestructorCallback (OnConstraintDestroy destructor)
{
	m_destructor = destructor;
}

void dBilateralConstraint::CalculateMatrixOffset (const dgVector& pivot, const dgVector& dir, dgMatrix& matrix0, dgMatrix& matrix1) const
{
	dFloat32 length; 
	dAssert (m_body0);
	dAssert (m_body1);
	dAssert (dir.m_w == dFloat32 (0.0f));
	const dgMatrix& body0_Matrix = m_body0->GetMatrix();

	length = dir.DotProduct(dir).GetScalar();
	length = dgSqrt (length);
	dAssert (length > dFloat32 (0.0f));
	matrix0 = dgMatrix (body0_Matrix.UnrotateVector (dir.Scale (dFloat32 (1.0f) / length)));
	matrix0.m_posit = body0_Matrix.UntransformVector (pivot);

	matrix0.m_front.m_w = dFloat32 (0.0f);
	matrix0.m_up.m_w    = dFloat32 (0.0f);
	matrix0.m_right.m_w = dFloat32 (0.0f);
	matrix0.m_posit.m_w = dFloat32 (1.0f);

	const dgMatrix& body1_Matrix = m_body1->GetMatrix();
	matrix1 = matrix0 * body0_Matrix * body1_Matrix.Inverse(); 
}


void dBilateralConstraint::SetPivotAndPinDir(const dgVector &pivot, const dgVector &pinDirection, dgMatrix& matrix0, dgMatrix& matrix1) const
{
	CalculateMatrixOffset (pivot, pinDirection, matrix0, matrix1);
}

void dBilateralConstraint::SetPivotAndPinDir (const dgVector& pivot, const dgVector& pinDirection0, const dgVector& pinDirection1, dgMatrix& matrix0, dgMatrix& matrix1) const
{
	dAssert (m_body0);
	dAssert (m_body1);

	const dgMatrix& body0_Matrix = m_body0->GetMatrix();
	dAssert (pinDirection0.m_w == dFloat32 (0.0f));
	dAssert ((pinDirection0.DotProduct(pinDirection0).GetScalar()) > dFloat32 (0.0f));

	matrix0.m_front = pinDirection0.Scale (dgRsqrt (pinDirection0.DotProduct(pinDirection0).GetScalar()));
	matrix0.m_right = matrix0.m_front.CrossProduct(pinDirection1);
	matrix0.m_right = matrix0.m_right.Scale (dgRsqrt (matrix0.m_right.DotProduct(matrix0.m_right).GetScalar()));
	matrix0.m_up = matrix0.m_right.CrossProduct(matrix0.m_front); 
	matrix0.m_posit = pivot;
	
	matrix0.m_front.m_w = dFloat32 (0.0f);
	matrix0.m_up.m_w    = dFloat32 (0.0f);
	matrix0.m_right.m_w = dFloat32 (0.0f);
	matrix0.m_posit.m_w = dFloat32 (1.0f);
	 
	const dgMatrix& body1_Matrix = m_body1->GetMatrix();

	matrix1 = matrix0 * body1_Matrix.Inverse(); 
	matrix0 = matrix0 * body0_Matrix.Inverse();
}

dgVector dBilateralConstraint::CalculateGlobalMatrixAndAngle (const dgMatrix& localMatrix0, const dgMatrix& localMatrix1, dgMatrix& globalMatrix0, dgMatrix& globalMatrix1) const
{
	dAssert (m_body0);
	dAssert (m_body1);
	const dgMatrix& body0Matrix = m_body0->GetMatrix();
	const dgMatrix& body1Matrix = m_body1->GetMatrix();

	globalMatrix0 = localMatrix0 * body0Matrix;
	globalMatrix1 = localMatrix1 * body1Matrix;

	dgMatrix relMatrix (globalMatrix1 * globalMatrix0.Inverse());

	dAssert (dgAbs (dFloat32 (1.0f) - (relMatrix.m_front.DotProduct(relMatrix.m_front).GetScalar())) < 1.0e-5f); 
	dAssert (dgAbs (dFloat32 (1.0f) - (relMatrix.m_up.DotProduct(relMatrix.m_up).GetScalar())) < 1.0e-5f); 
	dAssert (dgAbs (dFloat32 (1.0f) - (relMatrix.m_right.DotProduct(relMatrix.m_right).GetScalar())) < 1.0e-5f); 

	dgVector euler0;
	dgVector euler1;
	relMatrix.CalcPitchYawRoll (euler0, euler1);
	return euler0;
}

dFloat32 dBilateralConstraint::GetRowAcceleration (dInt32 index, dgContraintDescritor& desc) const
{
	//return m_motorAcceleration[index];
	return desc.m_penetrationStiffness[index];
}

void dBilateralConstraint::SetMotorAcceleration (dInt32 index, dFloat32 acceleration, dgContraintDescritor& desc)
{
	m_rowIsMotor |= (1 << index);
	desc.m_flags[index] = 0;
	m_motorAcceleration[index] = acceleration;
	desc.m_jointAccel[index] = acceleration;
	desc.m_penetrationStiffness[index] = acceleration; 
}

void dBilateralConstraint::SetJacobianDerivative (dInt32 index, dgContraintDescritor& desc, const dFloat32* const jacobianA, const dFloat32* const jacobianB, dgForceImpactPair* const jointForce)
{
	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 

	m_r0[index] = dgVector::m_zero;
	jacobian0.m_linear[0] = jacobianA[0];
	jacobian0.m_linear[1] = jacobianA[1];
	jacobian0.m_linear[2] = jacobianA[2];
	jacobian0.m_linear[3] = dFloat32 (0.0f);
	jacobian0.m_angular[0] = jacobianA[3];
	jacobian0.m_angular[1] = jacobianA[4];
	jacobian0.m_angular[2] = jacobianA[5];
	jacobian0.m_angular[3] = dFloat32 (0.0f);

	m_r1[index] = dgVector::m_zero;
	jacobian1.m_linear[0] = jacobianB[0];
	jacobian1.m_linear[1] = jacobianB[1];
	jacobian1.m_linear[2] = jacobianB[2];
	jacobian1.m_linear[3] = dFloat32 (0.0f);
	jacobian1.m_angular[0] = jacobianB[3];
	jacobian1.m_angular[1] = jacobianB[4];
	jacobian1.m_angular[2] = jacobianB[5];
	jacobian1.m_angular[3] = dFloat32 (0.0f);

	m_rowIsMotor |= (1 << index);
	m_motorAcceleration[index] = dFloat32 (0.0f);

	desc.m_flags[index] = 0;
	desc.m_restitution[index] = dFloat32 (0.0f);
	desc.m_jointAccel[index] = dFloat32 (0.0f);
	desc.m_penetration[index] = dFloat32 (0.0f);
	desc.m_penetrationStiffness[index] = dFloat32 (0.0f);
	desc.m_diagonalRegularizer[index] = m_defualtDiagonalRegularizer;
	desc.m_forceBounds[index].m_jointForce = jointForce;
}

void dBilateralConstraint::SetSpringDamperAcceleration (dInt32 index, dgContraintDescritor& desc, dFloat32 rowStiffness, dFloat32 spring, dFloat32 damper)
{
	if (desc.m_timestep > dFloat32 (0.0f)) {

		dAssert (m_body1);
		const ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
		const ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 

		const dgVector& veloc0 = m_body0->m_veloc;
		const dgVector& omega0 = m_body0->m_omega;
		const dgVector& veloc1 = m_body1->m_veloc;
		const dgVector& omega1 = m_body1->m_omega;

		//dFloat32 relPosit = (p1Global - p0Global) % jacobian0.m_linear + jointAngle;
		dFloat32 relPosit = desc.m_penetration[index];
		dFloat32 relVeloc = - (veloc0.DotProduct(jacobian0.m_linear) + veloc1.DotProduct(jacobian1.m_linear) + omega0.DotProduct(jacobian0.m_angular) + omega1.DotProduct(jacobian1.m_angular)).GetScalar();

		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		dFloat32 dt = desc.m_timestep;
		dFloat32 ks = dgAbs (spring);
		dFloat32 kd = dgAbs (damper);
		dFloat32 ksd = dt * ks;
		dFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
		dFloat32 den = dt * kd + dt * ksd;
		dFloat32 accel = num / (dFloat32 (1.0f) + den);
		desc.m_diagonalRegularizer[index] = rowStiffness;
		SetMotorAcceleration (index, accel, desc);
	}
}

dFloat32 dBilateralConstraint::CalculateMotorAcceleration (dInt32 index, dgContraintDescritor& desc) const
{
	return desc.m_zeroRowAcceleration[index];
}

void dBilateralConstraint::CalculateAngularDerivative (dInt32 index, dgContraintDescritor& desc, const dgVector& dir,	dFloat32 stiffness, dFloat32 jointAngle, dgForceImpactPair* const jointForce)
{
	dAssert (jointForce);
	dAssert (m_body0);
	dAssert (dir.m_w == dFloat32 (0.0f));

	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	m_r0[index] = dgVector::m_zero;
	jacobian0.m_linear = dgVector::m_zero;
	jacobian0.m_angular = dir;
	dAssert(jacobian0.m_angular.m_w == dFloat32(0.0f));

	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 
	dAssert (m_body1);
	m_r1[index] = dgVector::m_zero;
	jacobian1.m_linear = dgVector::m_zero;
	jacobian1.m_angular = dir * dgVector::m_negOne;
	dAssert(jacobian1.m_angular.m_w == dFloat32(0.0f));

	const dgVector& omega0 = m_body0->GetOmega();
	const dgVector& omega1 = m_body1->GetOmega();
	const dFloat32 relOmega = -(omega0 * jacobian0.m_angular + omega1 * jacobian1.m_angular).AddHorizontal().GetScalar();

	m_rowIsMotor &= ~(1 << index);
	m_motorAcceleration[index] = dFloat32 (0.0f);
	if (desc.m_timestep > dFloat32 (0.0f)) {
		#ifdef _DEBUG
			const dFloat32 relCentr = -(omega0 * omega0.CrossProduct(jacobian0.m_angular) + omega1 * omega1.CrossProduct(jacobian1.m_angular)).AddHorizontal().GetScalar();
			// allow for some large error since this is affected bu numerical precision a lot
			dAssert (dgAbs(relCentr) < dFloat32 (4.0f)); 
		#endif

		const dgVector& gyroAlpha0 = m_body0->m_gyroAlpha;
		const dgVector& gyroAlpha1 = m_body1->m_gyroAlpha;
		const dFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();

		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		dFloat32 dt = desc.m_timestep;
		dFloat32 ks = DG_POS_DAMP;
		dFloat32 kd = DG_VEL_DAMP;
		dFloat32 ksd = dt * ks;
		dFloat32 num = ks * jointAngle + kd * relOmega + ksd * relOmega;
		dFloat32 den = dFloat32 (1.0f) + dt * kd + dt * ksd;
		dFloat32 alphaError = num / den;
		
		desc.m_flags[index] = 0;
		desc.m_penetration[index] = jointAngle;
		desc.m_diagonalRegularizer[index] = stiffness;
		desc.m_jointAccel[index] = alphaError + relGyro;
		desc.m_penetrationStiffness[index] = alphaError + relGyro;
		desc.m_restitution[index] = dFloat32(0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
		desc.m_zeroRowAcceleration[index] = relOmega * desc.m_invTimestep + relGyro;

	} else {
		desc.m_flags[index] = 0;
		desc.m_penetration[index] = dFloat32 (0.0f);
		desc.m_restitution[index] = dFloat32 (0.0f);
		desc.m_diagonalRegularizer[index] = stiffness;
		desc.m_jointAccel[index] = relOmega;
		desc.m_penetrationStiffness[index] = relOmega;;
		desc.m_zeroRowAcceleration[index]  = dFloat32 (0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
	}
}

void ndJointBilateralConstraint::CalculatePointDerivative (dInt32 index, dgContraintDescritor& desc, const dgVector& dir, const dgPointParam& param, dgForceImpactPair* const jointForce)
{
	dAssert (jointForce);
	dAssert (m_body0);
	dAssert (m_body1);
	dAssert (dir.m_w == dFloat32 (0.0f));

	ndJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	dgVector r0CrossDir (param.m_r0.CrossProduct(dir));
	m_r0[index] = param.m_r0;
	jacobian0.m_linear = dir;
	jacobian0.m_angular = r0CrossDir;
	dAssert(jacobian0.m_linear.m_w == dFloat32(0.0f));
	dAssert(jacobian0.m_angular.m_w == dFloat32(0.0f));

	ndJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 
	dgVector r1CrossDir (dir.CrossProduct(param.m_r1));
	m_r1[index] = param.m_r1;
	jacobian1.m_linear = dir * dgVector::m_negOne;
	jacobian1.m_angular = r1CrossDir;
	dAssert(jacobian1.m_linear.m_w == dFloat32(0.0f));
	dAssert(jacobian1.m_angular.m_w == dFloat32(0.0f));

	m_rowIsMotor &= ~(1 << index);
	m_motorAcceleration[index] = dFloat32 (0.0f);
	if (desc.m_timestep > dFloat32 (0.0f)) {
		dgVector positError (param.m_posit1 - param.m_posit0);
		dFloat32 relPosit = positError.DotProduct(dir).GetScalar();

		const dgVector& veloc0 = m_body0->m_veloc;
		const dgVector& veloc1 = m_body1->m_veloc;
		const dgVector& omega0 = m_body0->m_omega;
		const dgVector& omega1 = m_body1->m_omega;
		const dgVector& gyroAlpha0 = m_body0->m_gyroAlpha;
		const dgVector& gyroAlpha1 = m_body1->m_gyroAlpha;
		const dgVector& centripetal0 (omega0.CrossProduct(omega0.CrossProduct(m_r0[index])));
		const dgVector& centripetal1 (omega1.CrossProduct(omega1.CrossProduct(m_r1[index])));
				
		const dFloat32 relGyro = (jacobian0.m_angular * gyroAlpha0 + jacobian1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
		const dFloat32 relCentr = -(jacobian0.m_linear * centripetal0 + jacobian1.m_linear * centripetal1).AddHorizontal().GetScalar();
		const dFloat32 relVeloc = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();
		
		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		const dFloat32 dt = desc.m_timestep;
		const dFloat32 ks = DG_POS_DAMP;
		const dFloat32 kd = DG_VEL_DAMP;
		const dFloat32 ksd = dt * ks;
		const dFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
		const dFloat32 den = dFloat32 (1.0f) + dt * kd + dt * ksd;
		const dFloat32 accelError = num / den;

		const dFloat32 relAccel = accelError + relCentr + relGyro;
		desc.m_flags[index] = 0;
		desc.m_penetration[index] = relPosit;
		desc.m_diagonalRegularizer[index] = param.m_defualtDiagonalRegularizer;
		desc.m_jointAccel[index] = relAccel;
		desc.m_penetrationStiffness[index] = relAccel;
		desc.m_restitution[index] = dFloat32 (0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
		desc.m_zeroRowAcceleration[index] = relVeloc * desc.m_invTimestep + relGyro;

	} else {
		const dgVector& veloc0 = m_body0->m_veloc;
		const dgVector& veloc1 = m_body1->m_veloc;
		const dgVector& omega0 = m_body0->m_omega;
		const dgVector& omega1 = m_body1->m_omega;
		const dFloat32 relVeloc = -(jacobian0.m_linear * veloc0 + jacobian0.m_angular * omega0 + jacobian1.m_linear * veloc1 + jacobian1.m_angular * omega1).AddHorizontal().GetScalar();

		desc.m_flags[index] = 0;
		desc.m_penetration[index] = dFloat32 (0.0f);
		desc.m_diagonalRegularizer[index] = param.m_defualtDiagonalRegularizer;
		desc.m_jointAccel[index] = relVeloc;
		desc.m_penetrationStiffness[index] = relVeloc;
		desc.m_restitution[index] = dFloat32 (0.0f);
		desc.m_zeroRowAcceleration[index]  = dFloat32 (0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
	}
}

void ndJointBilateralConstraint::JointAccelerations(dgJointAccelerationDecriptor* const params)
{
	const dgVector& bodyVeloc0 = m_body0->m_veloc;
	const dgVector& bodyOmega0 = m_body0->m_omega;
	const dgVector& bodyVeloc1 = m_body1->m_veloc;
	const dgVector& bodyOmega1 = m_body1->m_omega;
	const dgVector& gyroAlpha0 = m_body0->m_gyroAlpha;
	const dgVector& gyroAlpha1 = m_body1->m_gyroAlpha;

	dRightHandSide* const rhs = params->m_rightHandSide;
	const dgLeftHandSide* const row = params->m_leftHandSide;
	if (params->m_timeStep > dFloat32 (0.0f)) {
		const dFloat32 ks = DG_POS_DAMP * dFloat32 (0.5f);
		const dFloat32 kd = DG_VEL_DAMP * dFloat32 (4.0f);
		const dFloat32 dt = params->m_timeStep;
		for (dInt32 k = 0; k < params->m_rowsCount; k ++) {
			if (m_rowIsMotor & (1 << k)) {
   				rhs[k].m_coordenateAccel = m_motorAcceleration[k] + rhs[k].m_deltaAccel;
			} else {
				const ndJacobianPair& Jt = row[k].m_Jt;

				//calculate internal centripetal each sub step 
				const dgVector& centripetal0(bodyOmega0.CrossProduct(bodyOmega0.CrossProduct(m_r0[k])));
				const dgVector& centripetal1(bodyOmega1.CrossProduct(bodyOmega1.CrossProduct(m_r1[k])));

				const dgVector relVeloc(Jt.m_jacobianM0.m_linear * bodyVeloc0 + Jt.m_jacobianM0.m_angular * bodyOmega0 +
										Jt.m_jacobianM1.m_linear * bodyVeloc1 + Jt.m_jacobianM1.m_angular * bodyOmega1);
				const dFloat32 relGyro = (Jt.m_jacobianM0.m_angular * gyroAlpha0 + Jt.m_jacobianM1.m_angular * gyroAlpha1).AddHorizontal().GetScalar();
				const dFloat32 relCentr = -(Jt.m_jacobianM0.m_linear * centripetal0 + Jt.m_jacobianM1.m_linear * centripetal1).AddHorizontal().GetScalar();

				dFloat32 vRel = relVeloc.AddHorizontal().GetScalar();

				//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
				//alphaError = num / den;
				//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
				//dFloat32 dt = desc.m_timestep;
				//dFloat32 ks = DG_POS_DAMP;
				//dFloat32 kd = DG_VEL_DAMP;
				//dFloat32 ksd = dt * ks;
				//dFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
				//dFloat32 den = dFloat32 (1.0f) + dt * kd + dt * ksd;
				//accelError = num / den;

				dFloat32 relPosit = rhs[k].m_penetration - vRel * dt * params->m_firstPassCoefFlag;
				rhs[k].m_penetration = relPosit;

				dFloat32 ksd = dt * ks;
				dFloat32 num = ks * relPosit - kd * vRel - ksd * vRel;
				dFloat32 den = dFloat32(1.0f) + dt * kd + dt * ksd;
				dFloat32 aRelErr = num / den;
				rhs[k].m_coordenateAccel = rhs[k].m_deltaAccel + aRelErr + relCentr + relGyro;
			}
		}
	} else {

		for (dInt32 k = 0; k < params->m_rowsCount; k ++) {
			if (m_rowIsMotor & (1 << k)) {
				rhs[k].m_coordenateAccel = m_motorAcceleration[k] + rhs[k].m_deltaAccel;
			} else {
				const ndJacobianPair& Jt = row[k].m_Jt;
				dgVector relVeloc (Jt.m_jacobianM0.m_linear * bodyVeloc0 + Jt.m_jacobianM0.m_angular * bodyOmega0 +
								   Jt.m_jacobianM1.m_linear * bodyVeloc1 + Jt.m_jacobianM1.m_angular * bodyOmega1);

				dFloat32 vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
				rhs[k].m_coordenateAccel = rhs[k].m_deltaAccel - vRel;
			}
		}
	}
}
#endif


ndJointBilateralConstraint::ndJointBilateralConstraint(ndBodyKinematic* const body0, ndBodyKinematic* const body1, const dMatrix& globalMatrix)
	:ndConstraint()
	,dClassAlloc()
	,m_body0(body0)
	,m_body1(body1)
	,m_worldNode(nullptr)
	//,m_destructor(nullptr)
	//,m_jointNode(nullptr)
{
	dAssert(m_body0 && m_body1);

	if (m_body1->GetInvMass() == dFloat32(0.0f)) 
	{
		dSwap(m_body0, m_body1);
	}
	dAssert(m_body1->GetInvMass() > dFloat32(0.0f));

	CalculateLocalMatrix(globalMatrix, m_localMatrix0, m_localMatrix1);

	//m_maxDOF = 6;
	//m_isBilateral = true;
	//m_isActive = true;
	//m_solverModel = 0;
	//m_rowIsMotor = 0;
	//m_massScaleBody0 = dFloat32(1.0f);
	//m_massScaleBody1 = dFloat32(1.0f);
	//m_defualtDiagonalRegularizer = dFloat32(0.0f);
	//SetStiffness(dFloat32(0.0f));
	//
	//memset(m_jointForce, 0, sizeof(m_jointForce));
	//memset(m_motorAcceleration, 0, sizeof(m_motorAcceleration));
}

ndJointBilateralConstraint::~ndJointBilateralConstraint()
{
	dAssert(m_worldNode == nullptr);

	//if (m_destructor) 
	//{
	//	m_destructor(*this);
	//}
	//
	//if (m_jointNode) 
	//{
	//	dAssert(m_body0);
	//	dgBilateralConstraintList* const jointList = m_body0->m_world;
	//	jointList->Remove(m_jointNode);
	//}
}


void ndJointBilateralConstraint::CalculateLocalMatrix(const dMatrix& globalMatrix, dMatrix& localMatrix0, dMatrix& localMatrix1) const
{
	dAssert(globalMatrix.TestOrthogonal());
	localMatrix0 = globalMatrix * m_body0->GetMatrix().Inverse();
	localMatrix1 = globalMatrix * m_body1->GetMatrix().Inverse();
}
