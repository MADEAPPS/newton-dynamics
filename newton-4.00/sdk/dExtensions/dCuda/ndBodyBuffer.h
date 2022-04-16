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
#include <ndNewtonStdafx.h>

#include "cuQuat.h"
#include "cuVector.h"
#include "cuMatrix3x3.h"
#include "cuDeviceBuffer.h"

class cuBodyProxy
{
	public:
	inline cuMatrix3x3 __device__ CalculateInvInertiaMatrix(const cuMatrix3x3& matrix) const
	{
		const cuVector invIxx(m_invIntertia.GetElement(0));
		const cuVector invIyy(m_invIntertia.GetElement(1));
		const cuVector invIzz(m_invIntertia.GetElement(2));
		return cuMatrix3x3(
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

	inline void __device__ AddDampingAcceleration(const cuMatrix3x3& matrix)
	{
		const cuVector omega(matrix.UnrotateVector(m_omega) * m_dampCoef);
		m_omega = matrix.RotateVector(omega);
		m_veloc = m_veloc.Scale(m_dampCoef.w);
	}

	inline void __device__ IntegrateExternalForce(const cuMatrix3x3& matrix, float timestep)
	{
		//if (!m_equilibrium && (m_invMass.m_w > float(0.0f)))
		{
			//const ndVector accel(GetForce().Scale(m_invMass.m_w));
			//const ndVector torque(GetTorque());
			const cuVector accel(0.0);
			const cuVector torque(0.0);
			
			cuVector localOmega(matrix.UnrotateVector(m_omega));
			const cuVector localAngularMomentum(localOmega * m_mass);
			const cuVector angularMomentum(matrix.RotateVector(localAngularMomentum));
			const cuVector gyroTorque(m_omega.CrossProduct(angularMomentum));
			const cuVector localTorque(matrix.UnrotateVector(torque - gyroTorque));
			
			// and solving for alpha we get the angular acceleration at t + dt
			// calculate gradient at a full time step
			// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
			const cuVector dw(localOmega.Scale(0.5f * timestep));
			
			const cuMatrix3x3 jacobianMatrix(
				cuVector(m_mass.x, (m_mass.z - m_mass.y) * dw.z, (m_mass.z - m_mass.y) * dw.y, 0.0f),
				cuVector((m_mass.x - m_mass.z) * dw.z, m_mass.y, (m_mass.x - m_mass.z) * dw.x, 0.0f),
				cuVector((m_mass.y - m_mass.x) * dw.y, (m_mass.y - m_mass.x) * dw.x, m_mass.z, 0.0f));
	
			const cuVector gradientStep (jacobianMatrix.SolveByGaussianElimination(localTorque.Scale(timestep)));
			localOmega = localOmega + gradientStep;
			const cuVector alpha(matrix.RotateVector(localTorque * m_invIntertia));
			
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
			const float omegaAngle = ndSqrt(omegaMag2);
			const cuVector omegaAxis(m_omega.Scale(ndFloat32(1.0f) / omegaAngle));
			const cuQuat rotationStep(omegaAxis, omegaAngle * timestep);
			const cuQuat rotation(m_rotation * rotationStep);
			m_rotation = rotation.Normalize();
		}
	}

	void ProxyToBody(ndBodyKinematic* const body) const
	{
		dAssert(0);
		//const ndVector veloc(m_veloc.x, m_veloc.y, m_veloc.z, ndFloat32(0.0f));
		//const ndVector omega(m_omega.x, m_omega.y, m_omega.z, ndFloat32(0.0f));
		//const ndVector position(m_posit.x, m_posit.y, m_posit.z, ndFloat32(1.0f));
		//const ndQuaternion rotation(ndVector(m_rotation.x, m_rotation.y, m_rotation.z, m_rotation.w));
		//
		//body->SetOmegaNoSleep(omega);
		//body->SetVelocityNoSleep(veloc);
		//body->SetMatrixAndCentreOfMass(rotation, position);
	}

	cuQuat m_rotation;
	cuVector m_posit;
	cuVector m_veloc;
	cuVector m_omega;

	// scene Management data
	cuQuat m_globalSphapeRotation;
	cuVector m_globalSphapePosition;
	cuVector m_minAabb;
	cuVector m_maxAabb;

	// constant Data
	cuVector m_mass;
	cuVector m_dampCoef;
	cuVector m_invIntertia;
	cuVector m_obbSize;
	cuVector m_obbOrigin;
	cuVector m_scale;
	cuVector m_localPosition;
	cuQuat m_localRotation;
	cuQuat m_alignRotation;
};


#endif