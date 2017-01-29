/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"

#include "dgConstraint.h"
#include "dgWorldDynamicUpdate.h"
#include "dgBilateralConstraint.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_VEL_DAMP				 (dgFloat32(100.0f))
#define DG_POS_DAMP				 (dgFloat32(1500.0f))


dgBilateralConstraint::dgBilateralConstraint ()
	:dgConstraint () 
	,m_destructor(NULL)
{
	m_maxDOF = 6;
	m_isBilateral = true;
	m_contactActive = true;
m_canBeSkeleton = true;

	SetStiffness (dgFloat32 (1.0f));

	memset (m_jointForce, 0, sizeof (m_jointForce));
	memset (m_rowIsMotor, 0, sizeof (m_rowIsMotor));
	memset (m_motorAcceleration, 0, sizeof (m_motorAcceleration));
}

dgBilateralConstraint::~dgBilateralConstraint ()
{
	if (m_destructor) {
		m_destructor(*this);
	}
}


dgFloat32 dgBilateralConstraint::GetStiffness() const
{
	return m_stiffness;
}


void dgBilateralConstraint::SetStiffness(dgFloat32 stiffness)
{
	m_stiffness = dgClamp (stiffness, dgFloat32(0.0f), dgFloat32(1.0f));
}


void dgBilateralConstraint::SetDestructorCallback (OnConstraintDestroy destructor)
{
	m_destructor = destructor;
}

void dgBilateralConstraint::CalculateMatrixOffset (const dgVector& pivot, const dgVector& dir, dgMatrix& matrix0, dgMatrix& matrix1)
{
	dgFloat32 length; 
	dgAssert (m_body0);
	dgAssert (m_body1);

	const dgMatrix& body0_Matrix = m_body0->GetMatrix();

	length = dir.DotProduct3(dir);
	length = dgSqrt (length);
	dgAssert (length > dgFloat32 (0.0f));
	matrix0 = dgMatrix (body0_Matrix.UnrotateVector (dir.Scale3 (dgFloat32 (1.0f) / length)));
	matrix0.m_posit = body0_Matrix.UntransformVector (pivot);

	matrix0.m_front.m_w = dgFloat32 (0.0f);
	matrix0.m_up.m_w    = dgFloat32 (0.0f);
	matrix0.m_right.m_w = dgFloat32 (0.0f);
	matrix0.m_posit.m_w = dgFloat32 (1.0f);

	const dgMatrix& body1_Matrix = m_body1->GetMatrix();
	matrix1 = matrix0 * body0_Matrix * body1_Matrix.Inverse(); 
}


void dgBilateralConstraint::SetPivotAndPinDir(const dgVector &pivot, const dgVector &pinDirection)
{
	dgAssert (0);
//	CalculateMatrixOffset (pivot, pinDirection, m_localMatrix0, m_localMatrix1);
}

void dgBilateralConstraint::SetPivotAndPinDir (const dgVector& pivot, const dgVector& pinDirection0, const dgVector& pinDirection1)
{
	dgAssert (0);
/*
	dgAssert (m_body0);
	dgAssert (m_body1);

	const dgMatrix& body0_Matrix = m_body0->GetMatrix();

	
	dgAssert ((pinDirection0 % pinDirection0) > dgFloat32 (0.0f));
	m_localMatrix0.m_front = pinDirection0.Scale3 (dgRsqrt (pinDirection0 % pinDirection0));
	m_localMatrix0.m_right = m_localMatrix0.m_front * pinDirection1;
	m_localMatrix0.m_right = m_localMatrix0.m_right.Scale3 (dgRsqrt (m_localMatrix0.m_right % m_localMatrix0.m_right));
	m_localMatrix0.m_up = m_localMatrix0.m_right * m_localMatrix0.m_front; 
	m_localMatrix0.m_posit = pivot;

	m_localMatrix0.m_front.m_w = dgFloat32 (0.0f);
	m_localMatrix0.m_up.m_w    = dgFloat32 (0.0f);
	m_localMatrix0.m_right.m_w = dgFloat32 (0.0f);
	m_localMatrix0.m_posit.m_w = dgFloat32 (1.0f);
	 
	const dgMatrix& body1_Matrix = m_body1->GetMatrix();

	m_localMatrix1 = m_localMatrix0 * body1_Matrix.Inverse(); 
	m_localMatrix0 = m_localMatrix0 * body0_Matrix.Inverse();
*/
}

dgVector dgBilateralConstraint::CalculateGlobalMatrixAndAngle (dgMatrix& globalMatrix0, dgMatrix& globalMatrix1) const
{
	dgAssert (0);
	return dgVector (0.0f);
/*
	dgAssert (m_body0);
	dgAssert (m_body1);
	const dgMatrix& body0Matrix = m_body0->GetMatrix();
	const dgMatrix& body1Matrix = m_body1->GetMatrix();

	globalMatrix0 = m_localMatrix0 * body0Matrix;
	globalMatrix1 = m_localMatrix1 * body1Matrix;

	dgMatrix relMatrix (globalMatrix1 * globalMatrix0.Inverse());

	dgAssert (dgAbsf (dgFloat32 (1.0f) - (relMatrix.m_front % relMatrix.m_front)) < 1.0e-5f); 
	dgAssert (dgAbsf (dgFloat32 (1.0f) - (relMatrix.m_up % relMatrix.m_up)) < 1.0e-5f); 
	dgAssert (dgAbsf (dgFloat32 (1.0f) - (relMatrix.m_right % relMatrix.m_right)) < 1.0e-5f); 

	dgVector euler0;
	dgVector euler1;
	relMatrix.CalcPitchYawRoll (euler0, euler1);
	return euler0;
*/
}

dgFloat32 dgBilateralConstraint::GetRowAcceleration (dgInt32 index, dgContraintDescritor& desc) const
{
	return m_motorAcceleration[index];
}

void dgBilateralConstraint::SetMotorAcceleration (dgInt32 index, dgFloat32 acceleration, dgContraintDescritor& desc)
{
	m_rowIsMotor[index] = 1;
	m_motorAcceleration[index] = acceleration;
	desc.m_jointAccel[index] = acceleration;
}


void dgBilateralConstraint::SetJacobianDerivative (dgInt32 index, dgContraintDescritor& desc, const dgFloat32* const jacobianA, const dgFloat32* const jacobianB, dgForceImpactPair* const jointForce)
{
	dgJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	dgJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 

	jacobian0.m_linear[0] = jacobianA[0];
	jacobian0.m_linear[1] = jacobianA[1];
	jacobian0.m_linear[2] = jacobianA[2];
	jacobian0.m_linear[3] = dgFloat32 (0.0f);
	jacobian0.m_angular[0] = jacobianA[3];
	jacobian0.m_angular[1] = jacobianA[4];
	jacobian0.m_angular[2] = jacobianA[5];
	jacobian0.m_angular[3] = dgFloat32 (0.0f);


	jacobian1.m_linear[0] = jacobianB[0];
	jacobian1.m_linear[1] = jacobianB[1];
	jacobian1.m_linear[2] = jacobianB[2];
	jacobian1.m_linear[3] = dgFloat32 (0.0f);
	jacobian1.m_angular[0] = jacobianB[3];
	jacobian1.m_angular[1] = jacobianB[4];
	jacobian1.m_angular[2] = jacobianB[5];
	jacobian1.m_angular[3] = dgFloat32 (0.0f);

	m_rowIsMotor[index] = 1;
	m_motorAcceleration[index] = dgFloat32 (0.0f);

	desc.m_restitution[index] = dgFloat32 (0.0f);
	desc.m_jointAccel[index] = dgFloat32 (0.0f);
	desc.m_penetration[index] = dgFloat32 (0.0f);
	desc.m_penetrationStiffness[index] = dgFloat32 (0.0f);
	desc.m_jointStiffness[index] = dgFloat32 (1.0f);
	desc.m_forceBounds[index].m_jointForce = jointForce;
}


void dgBilateralConstraint::SetSpringDamperAcceleration (dgInt32 index, dgContraintDescritor& desc, dgFloat32 spring, dgFloat32 damper)
{
	if (desc.m_timestep > dgFloat32 (0.0f)) {

		dgAssert (m_body1);
		const dgJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
		const dgJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 

		const dgVector& veloc0 = m_body0->m_veloc;
		const dgVector& omega0 = m_body0->m_omega;
		const dgVector& veloc1 = m_body1->m_veloc;
		const dgVector& omega1 = m_body1->m_omega;

		//dgFloat32 relPosit = (p1Global - p0Global) % jacobian0.m_linear + jointAngle;
		dgFloat32 relPosit = desc.m_penetration[index];
		dgFloat32 relVeloc = - (veloc0.DotProduct3(jacobian0.m_linear) + veloc1.DotProduct3(jacobian1.m_linear) + omega0.DotProduct3(jacobian0.m_angular) + omega1.DotProduct3(jacobian1.m_angular));

		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		dgFloat32 dt = desc.m_timestep;
		dgFloat32 ks = dgAbsf (spring);
		dgFloat32 kd = dgAbsf (damper);
		dgFloat32 ksd = dt * ks;
		dgFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
		dgFloat32 den = dt * kd + dt * ksd;
		dgFloat32 accel = num / (dgFloat32 (1.0f) + den);
//		desc.m_jointStiffness[index] = - den / DG_PSD_DAMP_TOL ;
		desc.m_jointStiffness[index] = - dgFloat32 (1.0f) - den / DG_PSD_DAMP_TOL;
		SetMotorAcceleration (index, accel, desc);
	}
}


void dgBilateralConstraint::CalculateAngularDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& dir,	dgFloat32 stiffness, dgFloat32 jointAngle, dgForceImpactPair* const jointForce)
{
	dgAssert (jointForce);
	dgAssert (m_body0);
	dgJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	jacobian0.m_linear[0] = dgFloat32 (0.0f);
	jacobian0.m_linear[1] = dgFloat32 (0.0f);
	jacobian0.m_linear[2] = dgFloat32 (0.0f);
	jacobian0.m_linear[3] = dgFloat32 (0.0f);
	jacobian0.m_angular[0] = dir.m_x;
	jacobian0.m_angular[1] = dir.m_y;
	jacobian0.m_angular[2] = dir.m_z;
	jacobian0.m_angular[3] = dgFloat32 (0.0f);

	dgJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 
	dgAssert (m_body1);
	jacobian1.m_linear[0] = dgFloat32 (0.0f);
	jacobian1.m_linear[1] = dgFloat32 (0.0f);
	jacobian1.m_linear[2] = dgFloat32 (0.0f);
	jacobian1.m_linear[3] = dgFloat32 (0.0f);
	jacobian1.m_angular[0] = -dir.m_x;
	jacobian1.m_angular[1] = -dir.m_y;
	jacobian1.m_angular[2] = -dir.m_z;
	jacobian1.m_angular[3] = dgFloat32 (0.0f);

	const dgVector& omega0 = m_body0->GetOmega();
	const dgVector& omega1 = m_body1->GetOmega();
	dgFloat32 omegaError = dir.DotProduct3(omega1 - omega0);

	m_rowIsMotor[index] = 0;
	m_motorAcceleration[index] = dgFloat32 (0.0f);

	if (desc.m_timestep > dgFloat32 (0.0f)) {

		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		dgFloat32 dt = desc.m_timestep;
		dgFloat32 ks = DG_POS_DAMP;
		dgFloat32 kd = DG_VEL_DAMP;
		dgFloat32 ksd = dt * ks;
		dgFloat32 num = ks * jointAngle + kd * omegaError + ksd * omegaError;
		dgFloat32 den = dgFloat32 (1.0f) + dt * kd + dt * ksd;
		dgFloat32 alphaError = num / den;

		desc.m_zeroRowAcceleration[index] = (jointAngle + omegaError * desc.m_timestep) * desc.m_invTimestep * desc.m_invTimestep;

		desc.m_penetration[index] = jointAngle;
		desc.m_jointAccel[index] = alphaError;
		desc.m_restitution[index] = dgFloat32 (0.0f);
		desc.m_jointStiffness[index] = stiffness;
		desc.m_penetrationStiffness[index] = dgFloat32 (0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
	} else {
		desc.m_penetration[index] = dgFloat32 (0.0f);
		desc.m_jointAccel[index] = omegaError;
		desc.m_restitution[index] = dgFloat32 (0.0f);
		desc.m_jointStiffness[index] = stiffness;
		desc.m_penetrationStiffness[index] = dgFloat32 (0.0f);
		desc.m_zeroRowAcceleration[index]  = dgFloat32 (0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
	}
}

void dgBilateralConstraint::CalculatePointDerivative (dgInt32 index, dgContraintDescritor& desc, const dgVector& dir, const dgPointParam& param, dgForceImpactPair* const jointForce)
{
	dgAssert (jointForce);
	dgAssert (m_body0);
	dgAssert (m_body1);

	dgJacobian &jacobian0 = desc.m_jacobian[index].m_jacobianM0; 
	dgVector r0CrossDir (param.m_r0.CrossProduct3(dir));
	jacobian0.m_linear[0] = dir.m_x;
	jacobian0.m_linear[1] = dir.m_y;
	jacobian0.m_linear[2] = dir.m_z;
	jacobian0.m_linear[3] = dgFloat32 (0.0f);
	jacobian0.m_angular[0] = r0CrossDir.m_x;
	jacobian0.m_angular[1] = r0CrossDir.m_y;
	jacobian0.m_angular[2] = r0CrossDir.m_z;
	jacobian0.m_angular[3] = dgFloat32 (0.0f);

	dgJacobian &jacobian1 = desc.m_jacobian[index].m_jacobianM1; 
	dgVector r1CrossDir (dir.CrossProduct3(param.m_r1));
	jacobian1.m_linear[0] = -dir.m_x;
	jacobian1.m_linear[1] = -dir.m_y;
	jacobian1.m_linear[2] = -dir.m_z;
	jacobian1.m_linear[3] = dgFloat32 (0.0f);
	jacobian1.m_angular[0] = r1CrossDir.m_x;
	jacobian1.m_angular[1] = r1CrossDir.m_y;
	jacobian1.m_angular[2] = r1CrossDir.m_z;
	jacobian1.m_angular[3] = dgFloat32 (0.0f);

	m_rowIsMotor[index] = 0;
	m_motorAcceleration[index] = dgFloat32 (0.0f);

	dgVector velocError (param.m_veloc1 - param.m_veloc0);
	dgFloat32 relVeloc = velocError.DotProduct3(dir);
	if (desc.m_timestep > dgFloat32 (0.0f)) {

		dgVector positError (param.m_posit1 - param.m_posit0);
		dgVector centrError (param.m_centripetal1 - param.m_centripetal0);

		dgFloat32 relPosit = positError.DotProduct3(dir);
		dgFloat32 relCentr = centrError.DotProduct3(dir); 
		relCentr = dgClamp (relCentr, dgFloat32(-100000.0f), dgFloat32(100000.0f));

		desc.m_zeroRowAcceleration[index] = (relPosit + relVeloc * desc.m_timestep) * desc.m_invTimestep * desc.m_invTimestep;

		//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
		dgFloat32 dt = desc.m_timestep;
		dgFloat32 ks = DG_POS_DAMP;
		dgFloat32 kd = DG_VEL_DAMP;
		dgFloat32 ksd = dt * ks;
		dgFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
		dgFloat32 den = dgFloat32 (1.0f) + dt * kd + dt * ksd;
		dgFloat32 accelError = num / den;

		desc.m_penetration[index] = relPosit;
		desc.m_penetrationStiffness[index] = dgFloat32 (0.01f/4.0f);
		desc.m_jointStiffness[index] = param.m_stiffness;
		desc.m_jointAccel[index] = accelError + relCentr;
		// save centripetal acceleration in the restitution member
		desc.m_restitution[index] = relCentr;
		desc.m_forceBounds[index].m_jointForce = jointForce;
	} else {
		desc.m_penetration[index] = dgFloat32 (0.0f);
		desc.m_penetrationStiffness[index] = dgFloat32 (0.0f);
		desc.m_jointStiffness[index] = param.m_stiffness;
		desc.m_jointAccel[index] = relVeloc;
		desc.m_restitution[index] = dgFloat32 (0.0f);
		desc.m_zeroRowAcceleration[index]  = dgFloat32 (0.0f);
		desc.m_forceBounds[index].m_jointForce = jointForce;
	}
}

dgFloat32 dgBilateralConstraint::CalculateMotorAcceleration (dgInt32 index, dgContraintDescritor& desc) const
{
//static int xxx;
//dgTrace (("%d %f %f\n", xxx, desc.m_zeroRowAcceleration[index], desc.m_jointAccel[index]));
//xxx ++;
	return desc.m_zeroRowAcceleration[index];
}

void dgBilateralConstraint::JointAccelerations(dgJointAccelerationDecriptor* const params)
{
	dgJacobianMatrixElement* const jacobianMatrixElements = params->m_rowMatrix;
	const dgVector& bodyVeloc0 = m_body0->m_veloc;
	const dgVector& bodyOmega0 = m_body0->m_omega;
	const dgVector& bodyVeloc1 = m_body1->m_veloc;
	const dgVector& bodyOmega1 = m_body1->m_omega;

// remember the impulse branch 
//dgAssert (params->m_timeStep > dgFloat32 (0.0f));
	if (params->m_timeStep > dgFloat32 (0.0f)) {
		dgFloat32 kd = DG_VEL_DAMP * dgFloat32 (4.0f);
		dgFloat32 ks = DG_POS_DAMP * dgFloat32 (0.25f);
		dgFloat32 dt = params->m_timeStep;
		for (dgInt32 k = 0; k < params->m_rowsCount; k ++) {
			if (m_rowIsMotor[k]) {
				//params.m_coordenateAccel[k] = m_motorAcceleration[k] + params.m_externAccelaration[k];
   				jacobianMatrixElements[k].m_coordenateAccel = m_motorAcceleration[k] + jacobianMatrixElements[k].m_deltaAccel;
			} else {
				const dgJacobianPair& Jt = jacobianMatrixElements[k].m_Jt;
				dgVector relVeloc (Jt.m_jacobianM0.m_linear.CompProduct3(bodyVeloc0) + Jt.m_jacobianM0.m_angular.CompProduct3(bodyOmega0) +
								   Jt.m_jacobianM1.m_linear.CompProduct3(bodyVeloc1) + Jt.m_jacobianM1.m_angular.CompProduct3(bodyOmega1));

				dgFloat32 vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;

				//dgFloat32 aRel = jacobianMatrixElements[k].m_deltaAccel;
				//dgFloat32 aRel = jacobianMatrixElements[k].m_coordenateAccel;
				dgFloat32 aRel = params->m_firstPassCoefFlag ? jacobianMatrixElements[k].m_deltaAccel : jacobianMatrixElements[k].m_coordenateAccel;

				//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
				//alphaError = num / den;

				//at =  [- ks (x2 - x1) - kd * (v2 - v1) - dt * ks * (v2 - v1)] / [1 + dt * kd + dt * dt * ks] 
				//dgFloat32 dt = desc.m_timestep;
				//dgFloat32 ks = DG_POS_DAMP;
				//dgFloat32 kd = DG_VEL_DAMP;
				//dgFloat32 ksd = dt * ks;
				//dgFloat32 num = ks * relPosit + kd * relVeloc + ksd * relVeloc;
				//dgFloat32 den = dgFloat32 (1.0f) + dt * kd + dt * ksd;
				//accelError = num / den;

				dgFloat32 ksd = dt * ks;
				dgFloat32 relPosit = jacobianMatrixElements[k].m_penetration - vRel * dt * params->m_firstPassCoefFlag;
				jacobianMatrixElements[k].m_penetration = relPosit;


				dgFloat32 num = ks * relPosit - kd * vRel - ksd * vRel;
				dgFloat32 den = dgFloat32 (1.0f) + dt * kd + dt * ksd;
				dgFloat32 aRelErr = num / den;

				//centripetal acceleration is stored in restitution member
				jacobianMatrixElements[k].m_coordenateAccel = aRelErr + jacobianMatrixElements[k].m_restitution + aRel;
			}
		}
	} else {
		for (dgInt32 k = 0; k < params->m_rowsCount; k ++) {
			if (m_rowIsMotor[k]) {
				jacobianMatrixElements[k].m_coordenateAccel = m_motorAcceleration[k] + jacobianMatrixElements[k].m_deltaAccel;
			} else {
				const dgJacobianPair& Jt = jacobianMatrixElements[k].m_Jt;
				dgVector relVeloc (Jt.m_jacobianM0.m_linear.CompProduct3(bodyVeloc0) +
								   Jt.m_jacobianM0.m_angular.CompProduct3(bodyOmega0) +
								   Jt.m_jacobianM1.m_linear.CompProduct3(bodyVeloc1) +
								   Jt.m_jacobianM1.m_angular.CompProduct3(bodyOmega1));

				dgFloat32 vRel = relVeloc.m_x + relVeloc.m_y + relVeloc.m_z;
				jacobianMatrixElements[k].m_coordenateAccel = jacobianMatrixElements[k].m_deltaAccel - vRel;
			}
		}
	}
}

