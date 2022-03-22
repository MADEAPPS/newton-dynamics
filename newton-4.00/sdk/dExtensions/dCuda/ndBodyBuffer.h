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
#include "cuVector3.h"
#include "cuVector4.h"
#include "cuMatrix3x3.h"
#include "cuDeviceBuffer.h"

class ndBodyProxy
{
	public:
	inline cuMatrix3x3 __device__ CalculateInvInertiaMatrix(const cuMatrix3x3& matrix) const
	{
		const cuVector3 invIxx(m_invIntertia.GetElement(0));
		const cuVector3 invIyy(m_invIntertia.GetElement(1));
		const cuVector3 invIzz(m_invIntertia.GetElement(2));
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

	//inline void __device__ AddDampingAcceleration(const cuMatrix3x3& matrix, float timestep)
	inline void __device__ AddDampingAcceleration(const cuMatrix3x3& matrix, float)
	{
		const cuVector3 omega(matrix.UnrotateVector(m_omega) * m_dampCoef);
		m_omega = matrix.RotateVector(omega);
		m_veloc = m_veloc.Scale(m_dampCoef.m_w);
	}

	inline void __device__ IntegrateExternalForce(const cuMatrix3x3& matrix, float timestep)
	{
		//if (!m_equilibrium && (m_invMass.m_w > float(0.0f)))
		{
			//const ndVector accel(GetForce().Scale(m_invMass.m_w));
			//const ndVector torque(GetTorque());
			const cuVector3 accel(0.0);
			const cuVector3 torque(0.0);
			
			cuVector3 localOmega(matrix.UnrotateVector(m_omega));
			const cuVector3 localAngularMomentum(localOmega * m_mass);
			const cuVector3 angularMomentum(matrix.RotateVector(localAngularMomentum));
			const cuVector3 gyroTorque(m_omega.CrossProduct(angularMomentum));
			const cuVector3 localTorque(matrix.UnrotateVector(torque - gyroTorque));
			
			// and solving for alpha we get the angular acceleration at t + dt
			// calculate gradient at a full time step
			// derivative at half time step. (similar to midpoint Euler so that it does not loses too much energy)
			const cuVector3 dw(localOmega.Scale(0.5f * timestep));
			
			const cuMatrix3x3 jacobianMatrix(
				cuVector3(m_mass.m_x, (m_mass.m_z - m_mass.m_y) * dw.m_z, (m_mass.m_z - m_mass.m_y) * dw.m_y),
				cuVector3((m_mass.m_x - m_mass.m_z) * dw.m_z, m_mass.m_y, (m_mass.m_x - m_mass.m_z) * dw.m_x),
				cuVector3((m_mass.m_y - m_mass.m_x) * dw.m_y, (m_mass.m_y - m_mass.m_x) * dw.m_x, m_mass.m_z));
	
			const cuVector3 gradientStep (jacobianMatrix.SolveByGaussianElimination(localTorque.Scale(timestep)));
			localOmega = localOmega + gradientStep;
			const cuVector3 alpha(matrix.RotateVector(localTorque * m_invIntertia));
			
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
		float omegaMag2 = m_omega.DotProduct(m_omega);
		
		float tol = (float(0.0125f) * 3.141592f / 180.0f);
		float tol2 = tol * tol;
		if (omegaMag2 > tol2)
		{
			// this is correct
		//	//float invOmegaMag = ndRsqrt(omegaMag2);
			float invOmegaMag = 1.0f / sqrt(omegaMag2);
			const float omegaAngle = invOmegaMag * omegaMag2 * timestep;
			const cuVector3 omegaAxis(m_omega.Scale(invOmegaMag));
			const cuQuat rotationStep(omegaAxis, omegaAngle);
			const cuQuat rotation(m_rotation * rotationStep);
			m_rotation = rotation.Normalize();

			//m_matrix = ndMatrix(m_rotation, m_matrix.m_posit);
		}
		////m_matrix.m_posit = m_globalCentreOfMass - m_matrix.RotateVector(m_localCentreOfMass);
	}

	void BodyToProxy(ndBodyKinematic* const body)
	{
		m_mass = body->GetMassMatrix();
		m_rotation = cuQuat(body->GetRotation());
		m_posit = body->GetGlobalGetCentreOfMass();
		m_invIntertia = body->GetInvInertia();
		m_dampCoef = body->GetCachedDamping();
		m_veloc = body->GetVelocity();
		m_omega = body->GetOmega();
	}

	void ProxyToBody(ndBodyKinematic* const body) const
	{
		const ndVector veloc(m_veloc.m_x, m_veloc.m_y, m_veloc.m_z, ndFloat32(0.0f));
		const ndVector omega(m_omega.m_x, m_omega.m_y, m_omega.m_z, ndFloat32(0.0f));
		const ndVector position(m_posit.m_x, m_posit.m_y, m_posit.m_z, ndFloat32(1.0f));
		const ndQuaternion rotation(ndVector(m_rotation.m_x, m_rotation.m_y, m_rotation.m_z, m_rotation.m_w));

		body->SetOmegaNoSleep(omega);
		body->SetVelocityNoSleep(veloc);
		body->SetMatrixAndCentreOfMass(rotation, position);
	}

	cuVector4 m_mass;
	cuQuat m_rotation;
	cuVector4 m_dampCoef;
	cuVector4 m_invIntertia;
	cuVector3 m_posit;
	cuVector3 m_veloc;
	cuVector3 m_omega;
};

class ndBodyBuffer: public cuDeviceBuffer<ndBodyProxy>
{
	public:
	ndBodyBuffer();
	~ndBodyBuffer();
	ndArray<ndBodyProxy> m_dataView;
};


inline ndBodyBuffer::ndBodyBuffer()
	:cuDeviceBuffer<ndBodyProxy>()
{
}

inline ndBodyBuffer::~ndBodyBuffer()
{
}
#endif