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

#ifndef __ND_BODY_BUFFER_H__
#define __ND_BODY_BUFFER_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include "ndCudaQuat.h"
#include "ndCudaVector.h"
#include "ndCudaMatrix3x3.h"
#include "ndCudaDeviceBuffer.h"

class ndCudaBodyProxy
{
	public:
	inline ndCudaMatrix3x3 __device__ CalculateInvInertiaMatrix(const ndCudaMatrix3x3& matrix) const
	{
		const ndCudaVector invIxx(m_invIntertia.GetElement(0));
		const ndCudaVector invIyy(m_invIntertia.GetElement(1));
		const ndCudaVector invIzz(m_invIntertia.GetElement(2));
		return ndCudaMatrix3x3(
			matrix.m_front.Scale(matrix.m_front.GetElement(0)) * invIxx +
			matrix.m_up.Scale(matrix.m_up.GetElement(0))	* invIyy +
			matrix.m_right.Scale(matrix.m_right.GetElement(0)) * invIzz,

			matrix.m_front.Scale(matrix.m_front.GetElement(1)) * invIxx +
			matrix.m_up.Scale(matrix.m_up.GetElement(1))	* invIyy +
			matrix.m_right.Scale(matrix.m_right.GetElement(1)) * invIzz,

			matrix.m_front.Scale(matrix.m_front.GetElement(2)) * invIxx +
			matrix.m_up.Scale(matrix.m_up.GetElement(2))	* invIyy +
			matrix.m_right.Scale(matrix.m_right.GetElement(2)) * invIzz);
	}

	inline void __device__ AddDampingAcceleration(const ndCudaMatrix3x3& matrix)
	{
		const ndCudaVector omega(matrix.UnrotateVector(m_omega) * m_dampCoef);
		m_omega = matrix.RotateVector(omega);
		m_veloc = m_veloc.Scale(m_dampCoef.w);
	}

	inline void __device__ IntegrateExternalForce(const ndCudaMatrix3x3& matrix, float timestep)
	{
		//if (!m_equilibrium && (m_invMass.m_w > float(0.0f)))
		{
			//const ndVector accel(GetForce().Scale(m_invMass.m_w));
			//const ndVector torque(GetTorque());
			const ndCudaVector accel(0.0);
			const ndCudaVector torque(0.0);
			
			ndCudaVector localOmega(matrix.UnrotateVector(m_omega));
			const ndCudaVector localAngularMomentum(localOmega * m_mass);
			const ndCudaVector angularMomentum(matrix.RotateVector(localAngularMomentum));
			const ndCudaVector gyroTorque(m_omega.CrossProduct(angularMomentum));
			const ndCudaVector localTorque(matrix.UnrotateVector(torque - gyroTorque));
			
			// and solving for alpha we get the angular acceleration at t + dt
			// calculate gradient at a full time step
			// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
			const ndCudaVector dw(localOmega.Scale(0.5f * timestep));
			
			const ndCudaMatrix3x3 jacobianMatrix(
				ndCudaVector(m_mass.x, (m_mass.z - m_mass.y) * dw.z, (m_mass.z - m_mass.y) * dw.y, 0.0f),
				ndCudaVector((m_mass.x - m_mass.z) * dw.z, m_mass.y, (m_mass.x - m_mass.z) * dw.x, 0.0f),
				ndCudaVector((m_mass.y - m_mass.x) * dw.y, (m_mass.y - m_mass.x) * dw.x, m_mass.z, 0.0f));
	
			const ndCudaVector gradientStep (jacobianMatrix.SolveByGaussianElimination(localTorque.Scale(timestep)));
			localOmega = localOmega + gradientStep;
			const ndCudaVector alpha(matrix.RotateVector(localTorque * m_invIntertia));
			
			//SetAccel(accel);
			//SetAlpha(alpha);
			m_veloc = m_veloc + accel.Scale(timestep);
			m_omega = matrix.RotateVector(localOmega);
		}
		//else
		//{
		//	SetAccel(ndVector::m_zero);
		//	SetAlpha(ndVector::m_zero);
		//}
	}

	inline void __device__ IntegrateVelocity(float timestep)
	{
		m_posit = m_posit + m_veloc.Scale(timestep);
		const float omegaMag2 = m_omega.DotProduct(m_omega);
		
		const float tol = (float(0.0125f) * 3.141592f / 180.0f);
		const float tol2 = tol * tol;
		if (omegaMag2 > tol2)
		{
			// this is correct
			const float omegaAngle = sqrtf(omegaMag2);
			const ndCudaVector omegaAxis(m_omega.Scale(float(1.0f) / omegaAngle));
			const ndCudaQuat rotationStep(omegaAxis, omegaAngle * timestep);
			const ndCudaQuat rotation(m_rotation * rotationStep);
			m_rotation = rotation.Normalize();
		}
	}

	//void ProxyToBody(ndBodyKinematic* const body) const
	//void ProxyToBody(ndBodyKinematic* const) const
	void ProxyToBody(void* const) const
	{
		ndAssert(0);
		//const ndVector veloc(m_veloc.x, m_veloc.y, m_veloc.z, float(0.0f));
		//const ndVector omega(m_omega.x, m_omega.y, m_omega.z, float(0.0f));
		//const ndVector position(m_posit.x, m_posit.y, m_posit.z, float(1.0f));
		//const ndQuaternion rotation(ndVector(m_rotation.x, m_rotation.y, m_rotation.z, m_rotation.w));
		//
		//body->SetOmegaNoSleep(omega);
		//body->SetVelocityNoSleep(veloc);
		//body->SetMatrixAndCentreOfMass(rotation, position);
	}

	ndCudaQuat m_rotation;
	ndCudaVector m_posit;
	ndCudaVector m_veloc;
	ndCudaVector m_omega;

	// scene Management data
	ndCudaQuat m_globalSphapeRotation;
	ndCudaVector m_globalSphapePosition;
	ndCudaVector m_minAabb;
	ndCudaVector m_maxAabb;

	// constant Data
	ndCudaVector m_mass;
	ndCudaVector m_dampCoef;
	ndCudaVector m_invIntertia;
	ndCudaVector m_obbSize;
	ndCudaVector m_obbOrigin;
	ndCudaVector m_scale;
	ndCudaVector m_localPosition;
	ndCudaQuat m_localRotation;
	ndCudaQuat m_alignRotation;
};


#endif