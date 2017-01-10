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
#include "dgWorld.h"
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgDynamicBody.h"
#include "dgCollisionMassSpringDamperSystem.h"



dgCollisionMassSpringDamperSystem::dgCollisionMassSpringDamperSystem (dgWorld* const world, dgInt32 shapeID, dgInt32 pointCount, const dgFloat32* const points, dgInt32 srideInBytes, const dgFloat32* const pointsMasses, dgInt32 linksCount, const dgInt32* const links, const dgFloat32* const linksSpring, const dgFloat32* const LinksDamper)
	:dgCollisionDeformableMesh(world, m_deformableSolidMesh)
{
	m_rtti |= dgCollisionDeformableClothPatch_RTTI;

	m_particlesCount = pointCount;
	m_posit.Resize(m_particlesCount);
	m_invMass.Resize(m_particlesCount);
	const dgInt32 stride = srideInBytes / sizeof (dgFloat32);
	for (dgInt32 i = 0; i < pointCount; i++) {
		m_invMass[i] = dgFloat32(1.0f / pointsMasses[i]);
		m_posit[i] = dgVector (points[i * stride + 0], points[i * stride + 1], points[i * stride + 2], dgFloat32(0.0f));
	}

	m_linksCount = linksCount;
	m_linkList.Resize (linksCount);
	for (dgInt32 i = 0; i < linksCount; i++) {
		dgInt32 v0 = links[i * 2 + 0];
		dgInt32 v1 = links[i * 2 + 1];
		dgAssert (v0 != v1);
		dgAssert (v0 >= 0);
		dgAssert (v1 >= 0);
		dgAssert (v0 < pointCount);
		dgAssert (v1 < pointCount);
		m_linkList[i].m_m0 = dgInt16(dgMin(v0, v1));
		m_linkList[i].m_m1 = dgInt16(dgMax(v0, v1));
		m_linkList[i].m_spring = linksSpring[i];
		m_linkList[i].m_damper = LinksDamper[i];

		const dgVector& p0 = m_posit[v0];
		const dgVector& p1 = m_posit[v1];
		dgVector dp(p0 - p1);
		m_linkList[i].m_restlength = dgSqrt(dp.DotProduct3(dp));
		dgAssert(m_linkList[i].m_restlength > dgFloat32(1.0e-2f));
	}

	FinalizeBuild();
}


dgCollisionMassSpringDamperSystem::dgCollisionMassSpringDamperSystem(const dgCollisionMassSpringDamperSystem& source)
	:dgCollisionDeformableMesh(source)
{
	m_rtti |= source.m_rtti;
}

dgCollisionMassSpringDamperSystem::dgCollisionMassSpringDamperSystem(dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionDeformableMesh(world, deserialization, userData, revisionNumber)
{
}

dgCollisionMassSpringDamperSystem::~dgCollisionMassSpringDamperSystem(void)
{
}

dgInt32 dgCollisionMassSpringDamperSystem::GetMemoryBufferSizeInBytes() const
{
	dgInt32 sizeInByte = 0;
	sizeInByte += 2 * m_particlesCount * sizeof (dgVector);
	sizeInByte += 1 * m_particlesCount * sizeof (dgFloat32);
	return sizeInByte;
}


void dgCollisionMassSpringDamperSystem::CalculateAcceleration(dgFloat32 timestep)
{
	// Ks is in [sec^-2] a spring constant unit acceleration, not a spring force acceleration. 
	// Kc is in [sec^-1] a damper constant unit velocity, not a damper force acceleration. 

	dgAssert (0);
/*
	// for now make a share value for all springs. later this is a per material feature.
	dgFloat32 kSpring = dgFloat32(1000.0f);
	dgFloat32 kDamper = dgFloat32(30.0f);

	dgInt32 iter = 4;
	dgVector* const accel = &m_accel[0];
	dgVector* const veloc = &m_veloc[0];
	dgVector* const posit = &m_posit[0];
	const dgFloat32* const restLenght = &m_restlength[0];

	//dgVector* const normalAccel = dgAlloca(dgVector, m_particlesCount);
	//dgVector* const normalDir = dgAlloca(dgVector, m_particlesCount);
	//dgFloat32* const frictionCoeffecient = dgAlloca(dgFloat32, m_particlesCount);

	dgWorld* const world = m_body->GetWorld();
	world->m_solverJacobiansMemory.ResizeIfNecessary (GetMemoryBufferSizeInBytes() + 1024);
	dgVector* const normalAccel = (dgVector*)&world->m_solverJacobiansMemory[0];
	dgVector* const normalDir = &normalAccel[m_particlesCount];
	dgFloat32* const frictionCoeffecient = (dgFloat32*) &normalDir[m_particlesCount];

	//dgFloat32* const damper_A01 = dgAlloca(dgFloat32, m_linksCount);
	//dgFloat32* const damper_B01 = dgAlloca(dgFloat32, m_linksCount);
	//dgFloat32* const damper_C01 = dgAlloca(dgFloat32, m_linksCount);
	//dgVector* const collisionDir1 = dgAlloca(dgVector, m_particlesCount);
	//dgVector* const collisionDir2 = dgAlloca(dgVector, m_particlesCount);
	//dgVector* const tmp1 = dgAlloca(dgVector, m_particlesCount);
	//dgVector* const tmp2 = dgAlloca(dgVector, m_particlesCount);
	//dgVector* const diag = dgAlloca(dgVector, m_particlesCount);
	//dgVector* const offDiag = dgAlloca(dgVector, m_particlesCount);

	dgVector unitAccel(m_body->m_externalForce.CompProduct4(m_body->m_invMass.m_w));
	dgVector deltaOmega(m_body->m_invWorldInertiaMatrix.RotateVector(m_body->m_externalTorque.Scale4(timestep)));

	m_body->m_alpha = dgVector::m_zero;
	m_body->m_omega = dgVector::m_zero;
	m_body->m_externalForce = dgVector::m_zero;
	m_body->m_externalTorque = dgVector::m_zero;

	// here I need to add all other external acceleration like wind and pressure, friction and collision.
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_externalAccel[i] = unitAccel;
		veloc[i] += deltaOmega.CrossProduct3(m_posit[i]);
	}

	const dgSpringDamperLink* const links = &m_linkList[0];
	dgFloat32 ks_dt = -timestep * kSpring;
	//	dgFloat32 kd_dt0 = -timestep * kDamper;
	//	dgFloat32 kd_dt1 = -timestep * kDamper * dgFloat32(2.0f);

	dgVector dtRK4(timestep / iter);
	dgVector epsilon(dgFloat32(1.0e-14f));

	dtRK4 = dtRK4 & dgVector::m_triplexMask;
	HandleCollision(timestep, normalDir, normalAccel, frictionCoeffecient);
	for (dgInt32 k = 0; k < iter; k++) {

		for (dgInt32 i = 0; i < m_particlesCount; i++) {
			accel[i] = m_externalAccel[i];
		}

		for (dgInt32 i = 0; i < m_linksCount; i++) {
			const dgInt32 j0 = links[i].m_m0;
			const dgInt32 j1 = links[i].m_m1;

			const dgVector p0p1(posit[j0] - posit[j1]);
			const dgVector v0v1(veloc[j0] - veloc[j1]);
			const dgVector dpdv(p0p1.CompProduct4(v0v1));

			const dgVector mag2(p0p1.DotProduct4(p0p1));
			const dgVector mask(mag2 > m_smallestLenght2);

			const dgVector lenght2((mag2 & mask) | mag2.AndNot(mask));
			const dgFloat32 length = (lenght2.Sqrt()).GetScalar();
			const dgFloat32 invDen = dgFloat32(1.0f) / length;
			const dgFloat32 lenghtRatio = restLenght[i] * invDen;
			const dgFloat32 compression = dgFloat32(1.0f) - lenghtRatio;
			const dgVector fs(p0p1.Scale4(kSpring * compression));
			const dgVector fd(p0p1.Scale4(kDamper * invDen * invDen * (v0v1.DotProduct4(p0p1)).GetScalar()));
			const dgVector dfsdx(v0v1.Scale4 (ks_dt * compression) + (p0p1.CompProduct4(dpdv).Scale4 (ks_dt * lenghtRatio * invDen * invDen)));
			dgAssert(fs.m_w == dgFloat32(0.0f));
			dgAssert(fs.m_w == dgFloat32(0.0f));
			dgAssert(p0p1.m_w == dgFloat32(0.0f));
			dgAssert(dfsdx.m_w == dgFloat32(0.0f));

			const dgVector nextAccel(fs + fd - dfsdx);
			accel[j0] -= nextAccel;
			accel[j1] += nextAccel;
		}


		for (dgInt32 i = 0; i < m_particlesCount; i++) {
			//dgVector dirAccel1 (collisionDir1[i].CompProduct4(accel[i].DotProduct4(collisionDir1[i])));
			//dgVector dirAccel2 (collisionDir2[i].CompProduct4(accel[i].DotProduct4(collisionDir2[i])));
			//tmp0[i] = accel[i] + collidingAccel[i] - dirAccel0 - dirAccel1 - dirAccel2;
			
			dgVector tangentDir(veloc[i] - normalDir[i].CompProduct4(normalDir[i].DotProduct4(veloc[i])));
			dgVector mag(tangentDir.DotProduct4(tangentDir) + epsilon);
			
			dgFloat32 tangentFrictionAccel = dgAbsf(accel[i].DotProduct4(normalDir[i]).GetScalar());
			dgVector friction(tangentDir.Scale4(frictionCoeffecient[i] * tangentFrictionAccel / dgSqrt(mag.GetScalar())));

			//dgVector particleAccel (accel[i] + normalAccel[i] - normalDirAccel);
			dgVector normalDirAccel(normalDir[i].CompProduct4(accel[i].DotProduct4(normalDir[i])));
			accel[i] = accel[i] + normalAccel[i] - normalDirAccel - friction;
			veloc[i] += accel[i].CompProduct4(dtRK4);
			posit[i] += veloc[i].CompProduct4(dtRK4);
		}
	}
*/
}
