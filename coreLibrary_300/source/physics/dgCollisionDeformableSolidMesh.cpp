/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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
#include "dgCollisionBVH.h"
#include "dgDeformableBody.h"
#include "dgDeformableContact.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableSolidMesh.h"


#define DG_DEFORMABLE_DEFAULT_STIFFNESS		 (dgFloat32 (0.8f))


//#define DG_DEFORMABLE_STACK_DEPTH	256
//#define DG_DEFORMABLE_PADDING				 (dgFloat32 (4.0f))
//#define DG_DEFORMABLE_INV_PADDING			 (dgFloat32 (1.0f) / DG_DEFORMABLE_PADDING)
//#define DG_DEFORMABLE_DEFAULT_PLASTICITY	 (dgFloat32 (0.3f))
//#define DG_DEFORMABLE_PLANE_DISTANCE_TOL	 (dgFloat32 (1.0e-4f))
//#define DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS (dgFloat32 (5.0e-2f))


/*
class dgCollisionDeformableSolidMesh::dgDeformationRegion
{
	public:

	void Init (dgCollisionDeformableSolidMesh* const me, dgInt32 count)
	{
		m_count = count;
	}

	void PrecomputeValues (dgCollisionDeformableSolidMesh* const me)
	{
		m_com0 = dgVector (dgFloat32 (0.0f));
		m_AqqInv = dgGetIdentityMatrix();
		m_rotSeed = dgGetIdentityMatrix();
		
		const dgVector* const shapePosit = me->m_shapePosit;
		const dgFloat32* const unitMass = me->m_particles.m_unitMass;

		m_totalMass = dgFloat32 (0.0f);
		dgVector accSum (dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			dgFloat32 mass = unitMass[index];
			m_totalMass += mass;
			accSum += shapePosit[index].Scale4 (mass);
		}

		if (m_totalMass > dgFloat32 (0.0f)) {
			m_com0 = accSum.Scale4 (dgFloat32 (1.0f) / m_totalMass);

			dgMatrix sumQiQi (dgGetZeroMatrix());
			for (dgInt32 i = 0; i < m_count; i ++) {
				dgInt32 index = m_indices[i];
				dgVector qi (shapePosit[index] - m_com0);
				dgFloat32 mass = unitMass[index];
				for (dgInt32 j = 0; j < 3; j ++) {
					for (dgInt32 k = 0; k < 3; k ++) {
						sumQiQi[j][k] += mass * qi[j] * qi[k];
					}
				}
			}

			dgAssert (dgAbsf (sumQiQi[1][0] - sumQiQi[0][1]) < dgFloat32 (1.0e-6f));
			dgAssert (dgAbsf (sumQiQi[2][0] - sumQiQi[0][2]) < dgFloat32 (1.0e-6f));
			dgAssert (dgAbsf (sumQiQi[2][1] - sumQiQi[1][2]) < dgFloat32 (1.0e-6f));
			sumQiQi[3][3] = dgFloat32 (1.0f);
			m_AqqInv = sumQiQi.Symetric3by3Inverse();
		}
	}

	void Copy (const dgDeformationRegion& src)
	{
		dgAssert (0);

		dgAssert (!m_indices);
		m_count = src.m_count;
		m_totalMass = src.m_totalMass;
		m_com0 = src.m_com0;
		m_com = src.m_com;
		m_AqqInv = src.m_AqqInv; 
		m_rotSeed = src.m_rotSeed; 

		m_indices = (dgInt16*) dgMallocStack(sizeof (dgInt16) * m_count);
		memcpy (m_indices, src.m_indices, sizeof (dgInt16) * m_count);

	}


	void UpdateVelocities(dgCollisionDeformableSolidMesh* const me, dgFloat32 timestep, dgFloat32 stiffness)
	{
		dgAssert (0);

		const dgCollisionDeformableSolidMesh::dgParticle& particles = me->m_particles;
		const dgFloat32* const mass = particles.m_unitMass;
		const dgVector* const posit0 = me->m_shapePosit;
		const dgVector* const posit1 = particles.m_posit;

		dgVector com = dgVector (dgFloat32 (0.0f));
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			com += posit1[index].Scale4 (mass[index]);
		}
		m_com = com.Scale4 (dgFloat32 (1.0f) / m_totalMass);

		dgMatrix sumQiPi (dgGetZeroMatrix());
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			dgVector pi (posit1[index] - m_com);
			dgVector qi (posit0[index] - m_com0);
			dgFloat32 massI = mass[index];
			for (dgInt32 j = 0; j < 3; j ++) {
				for (dgInt32 k = 0; k < 3; k ++) {
					sumQiPi[j][k] += massI * pi[j] * qi[k];
				}
			}
		}

		dgAssert (dgAbsf (sumQiPi[1][0] - sumQiPi[0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (sumQiPi[2][0] - sumQiPi[0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (sumQiPi[2][1] - sumQiPi[1][2]) < dgFloat32 (1.0e-6f));
		sumQiPi[3][3] = dgFloat32 (1.0f);

		dgMatrix S (sumQiPi * sumQiPi.Transpose4X4());
		dgVector eigenValues;

		S.EigenVectors (eigenValues, m_rotSeed);
		m_rotSeed = S;


		#ifdef _DEBUG
			dgMatrix P0 (sumQiPi * sumQiPi.Transpose4X4());
			dgMatrix D (dgGetIdentityMatrix());
			D[0][0] = eigenValues[0];
			D[1][1] = eigenValues[1];
			D[2][2] = eigenValues[2];
			dgMatrix P1 (S.Transpose4X4() * D * S);

			dgMatrix xx (P1 * P0.Symetric3by3Inverse());
			dgAssert (dgAbsf (xx[0][0] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[1][1] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[2][2] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[0][1]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[0][2]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[1][0]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[1][2]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[2][0]) < dgFloat32 (1.0e-3f));
			dgAssert (dgAbsf (xx[2][1]) < dgFloat32 (1.0e-3f));
		#endif

		eigenValues = eigenValues.InvSqrt();

		dgMatrix m (S);
		m.m_front = m.m_front.Scale4 (eigenValues.m_x);
		m.m_up    = m.m_up.Scale4 (eigenValues.m_y);
		m.m_right = m.m_right.Scale4 (eigenValues.m_z);
		dgMatrix invS = S.Transpose4X4() * m;
		dgMatrix R (invS * sumQiPi);
		dgMatrix A (m_AqqInv * sumQiPi);


dgVector beta0 (dgFloat32 (0.99f));
dgVector beta1 (dgVector::m_one - beta0);
		dgMatrix deformationMatrix (dgGetIdentityMatrix());
		deformationMatrix.m_front = A.m_front.CompProduct4(beta0) + R.m_front.CompProduct4(beta1);
		deformationMatrix.m_up = A.m_up.CompProduct4(beta0) + R.m_up.CompProduct4(beta1);
		deformationMatrix.m_right = A.m_right.CompProduct4(beta0) + R.m_right.CompProduct4(beta1);


		dgVector invTimeScale (stiffness / timestep);
		dgVector* const veloc = particles.m_veloc;
		for (dgInt32 i = 0; i < m_count; i ++) {
			dgInt32 index = m_indices[i];
			dgVector gi (deformationMatrix.RotateVector(posit0[index] - m_com0) + m_com);
			veloc[index] += (gi - posit1[index]).CompProduct4 (invTimeScale);
//dgTrace (("%f %f %f\n", veloc[i][0], veloc[i][1], veloc[i][2] ));
		}

	}

	dgMatrix m_AqqInv; 
	dgMatrix m_rotSeed; 
	dgVector m_com0;
	dgFloat32 m_totalMass;
	dgInt32 m_count;
};
*/


						   
dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionDeformableMesh (world, deserialization, userData)
{
	dgAssert (0);
}


dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgMeshEffect* const mesh)
	:dgCollisionDeformableMesh (world, mesh, m_deformableSolidMesh)
	,m_posit(NULL)
	,m_shapePosit(NULL)
	,m_weight(NULL)
	,m_positRegions(NULL)
	,m_positRegionsStart(NULL)
	,m_regionAqqInv(NULL) 
	,m_regionRotSeed(NULL) 
	,m_regionCom0(NULL)
	,m_regionMass(NULL)
	
	,m_regionsCount(0)
	,m_stiffness(DG_DEFORMABLE_DEFAULT_STIFFNESS)
{
	m_rtti |= dgCollisionDeformableSolidMesh_RTTI;
	mesh->Triangulate();

	m_regionsCount = 1;
	m_shapePosit = (dgVector*) dgMallocStack (sizeof (dgVector) * m_particles.m_count);
	m_posit = (dgVector*) dgMallocStack (sizeof (dgVector) * m_particles.m_count);
	m_positRegionsStart = (dgUnsigned16*) dgMallocStack (sizeof (dgUnsigned16) * (m_particles.m_count + 1));

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_shapePosit[i] = m_particles.m_posit[i];
		m_posit[i] = m_particles.m_posit[i];
	}
	CreateRegions();
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh (const dgCollisionDeformableSolidMesh& source)
	:dgCollisionDeformableMesh (source)
	,m_regionsCount(source.m_regionsCount)
	,m_stiffness(source.m_stiffness)
{
	m_rtti |= dgCollisionCompoundBreakable_RTTI;
	m_shapePosit = (dgVector*) dgMallocStack (sizeof (dgVector) * source.m_particles.m_count);
	m_posit = (dgVector*) dgMallocStack (sizeof (dgVector) * source.m_particles.m_count);
	m_weight = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * (source.m_positRegionsStart[source.m_particles.m_count]));
	m_positRegions = (dgUnsigned16*) dgMallocStack (sizeof (dgUnsigned16) * (source.m_positRegionsStart[source.m_particles.m_count]));
	m_positRegionsStart = (dgUnsigned16*) dgMallocStack (sizeof (dgUnsigned16) * (source.m_particles.m_count + 1));
	
	m_regionAqqInv = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_regionsCount);
	m_regionRotSeed = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_regionsCount);
	m_regionCom0 = (dgVector*) dgMallocStack (sizeof (dgVector) * m_regionsCount);
	m_regionMass = (dgFloat32*) dgMallocStack (sizeof (dgVector) * m_regionsCount);

	memcpy (m_posit, source.m_posit, sizeof (dgVector) * source.m_particles.m_count);
	memcpy (m_shapePosit, source.m_shapePosit, sizeof (dgVector) * source.m_particles.m_count);

	memcpy (m_regionAqqInv, source.m_regionAqqInv, sizeof (dgMatrix) * m_regionsCount);
	memcpy (m_regionRotSeed, source.m_regionRotSeed, sizeof (dgMatrix) * m_regionsCount);
	memcpy (m_regionCom0, source.m_regionCom0, sizeof (dgVector) * m_regionsCount);
	memcpy (m_regionMass, source.m_regionMass, sizeof (dgFloat32) * m_regionsCount);

	memcpy (m_weight, source.m_weight, sizeof (dgFloat32) * (source.m_positRegionsStart[source.m_particles.m_count]));
	memcpy (m_positRegions, source.m_positRegions, sizeof (dgUnsigned16) * (source.m_positRegionsStart[source.m_particles.m_count]));
	memcpy (m_positRegionsStart, source.m_positRegionsStart, sizeof (dgUnsigned16) * (source.m_particles.m_count + 1));
}


dgCollisionDeformableSolidMesh::~dgCollisionDeformableSolidMesh(void)
{
	if (m_shapePosit) {
		dgFree (m_shapePosit);
		dgFree (m_posit);
		dgFree (m_weight);
		dgFree (m_positRegions);
		dgFree (m_positRegionsStart);

		dgFree (m_regionAqqInv); 
		dgFree (m_regionRotSeed); 
		dgFree (m_regionCom0);
		dgFree (m_regionMass);
		
	}
}


void dgCollisionDeformableSolidMesh::CreateRegions()
{
	if (m_positRegions) {
		dgFree (m_regionAqqInv); 
		dgFree (m_regionRotSeed); 
		dgFree (m_regionCom0);
		dgFree (m_regionMass);
	}


	// for now only region
	m_regionsCount = 1;
	m_regionAqqInv = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_regionsCount);
	m_regionRotSeed = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_regionsCount);
	m_regionCom0 = (dgVector*) dgMallocStack (sizeof (dgVector) * m_regionsCount);
	m_regionMass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_regionsCount);
	

m_weight = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * (m_particles.m_count));
m_positRegions = (dgUnsigned16*) dgMallocStack (sizeof (dgUnsigned16) * (m_particles.m_count));
for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
	m_positRegionsStart[i] = dgUnsigned16(i);
	m_positRegions[i] = 0;
	m_weight[i] = dgFloat32 (1.0f);
}
m_positRegionsStart[m_particles.m_count] = dgUnsigned16(m_particles.m_count);


}


void dgCollisionDeformableSolidMesh::InitRegions()
{

	dgStack<dgVector> covarianceMatrixPool(m_particles.m_count * 3);
	dgVector* const covarianceMatrix = &covarianceMatrixPool[0];

	const dgFloat32* const masses = m_particles.m_unitMass;
	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		m_regionCom0[i] = dgVector (dgFloat32 (0.0f));
		m_regionMass[i] = dgFloat32 (0.0f);
		m_regionRotSeed[i] = dgGetIdentityMatrix();
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgVector& r = m_shapePosit[i];
		dgVector mr (r.Scale4(masses[i]));
		covarianceMatrix[i * 3 + 0] = mr.CompProduct4(r.BroadcastX()); 
		covarianceMatrix[i * 3 + 1] = mr.CompProduct4(r.BroadcastY()); 
		covarianceMatrix[i * 3 + 2] = mr.CompProduct4(r.BroadcastZ()); 

		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			m_regionCom0[index] += mr;
			m_regionMass[index] += masses[i];
		}
	}

	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		dgVector mcr0 (m_regionCom0[i]);
		dgVector cr0 (mcr0.Scale4 (dgFloat32 (1.0f) / m_regionMass[i]));
		mcr0 = mcr0.CompProduct4(dgVector::m_negOne);

		m_regionCom0[i] = cr0;
		m_regionAqqInv[i].m_front = mcr0.CompProduct4 (cr0.BroadcastX());
		m_regionAqqInv[i].m_up = mcr0.CompProduct4 (cr0.BroadcastX());
		m_regionAqqInv[i].m_right = mcr0.CompProduct4 (cr0.BroadcastX());
		m_regionAqqInv[i].m_posit = dgVector::m_wOne;
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;

		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			m_regionAqqInv[index].m_front += covarianceMatrix[i * 3 + 0];
			m_regionAqqInv[index].m_up += covarianceMatrix[i * 3 + 1];
			m_regionAqqInv[index].m_right += covarianceMatrix[i * 3 + 2];
		}
	}

	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		dgAssert (dgAbsf (m_regionAqqInv[i][1][0] - m_regionAqqInv[i][0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (m_regionAqqInv[i][2][0] - m_regionAqqInv[i][0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (m_regionAqqInv[i][2][1] - m_regionAqqInv[i][1][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (m_regionAqqInv[i][3][3] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-6f));
		m_regionAqqInv[i] = m_regionAqqInv[i].Symetric3by3Inverse();
	}
}


void dgCollisionDeformableSolidMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
}

dgInt32 dgCollisionDeformableSolidMesh::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}

void dgCollisionDeformableSolidMesh::EndConfiguration ()
{
	dgAssert(0);
}

void dgCollisionDeformableSolidMesh::ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body)
{
	dgAssert(0);
}


void dgCollisionDeformableSolidMesh::SetMass (dgFloat32 mass)
{
	if ((mass < dgFloat32 (1.0e5f)) && (mass > dgFloat32 (0.0f))) {
		mass = mass / m_particles.m_count;
		dgFloat32* const unitMass = m_particles.m_unitMass;
		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			unitMass[i] = mass;
		}

		InitRegions();
	}
}


void dgCollisionDeformableSolidMesh::ApplyExternalForces (dgFloat32 timestep)
{
	dgAssert (m_myBody);

	dgBody* const body = GetBody();

	dgAssert (body->GetMass().m_w > dgFloat32 (0.0f));
	dgAssert (body->GetMass().m_w < dgFloat32 (1.0e5f));
	const dgMatrix& matrix = body->GetCollision()->GetGlobalMatrix();

	dgFloat32 invMass = body->GetInvMass().m_w;
	dgVector velocyStep (body->GetForce().Scale4(invMass * timestep));

	dgVector* const veloc = m_particles.m_veloc;
	const dgVector* const particlePosit = m_particles.m_posit;
	dgFloat32* const unitMass = m_particles.m_unitMass;

	dgVector com (dgFloat32 (0.0f));
	dgVector comVeloc (dgFloat32 (0.0f));

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector mass (unitMass[i]);
		dgVector p (matrix.TransformVector(particlePosit[i]));
		m_posit[i] = p;
		veloc[i] += velocyStep;
		com += p.CompProduct4(mass);
		comVeloc += veloc[i].CompProduct4(mass);
	}

	com = com.Scale4(invMass);
	comVeloc = comVeloc.Scale4(invMass);

	dgMatrix InertiaMatrix (dgGetZeroMatrix());
	dgVector angularMomentum (dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector mass (unitMass[i]);
		dgVector r (m_posit[i] - com);
		dgVector mr (r.CompProduct4(mass));
		angularMomentum += r * mr;
		InertiaMatrix.m_front += mr.CompProduct4(r.BroadcastX()); 
		InertiaMatrix.m_up += mr.CompProduct4(r.BroadcastY()); 
		InertiaMatrix.m_right += mr.CompProduct4(r.BroadcastZ()); 
	}

dgVector damp (0.1f); 
	dgVector omega (InertiaMatrix.Symetric3by3Inverse().RotateVector(angularMomentum));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector r (m_posit[i] - com);
		dgVector deltaVeloc (comVeloc + omega * r - veloc[i]);
		veloc[i] += deltaVeloc.CompProduct4(damp);
	}
}



void dgCollisionDeformableSolidMesh::ResolvePositionsConstraints (dgFloat32 timestep)
{
	dgAssert (m_myBody);
//	dgVector* const posit = m_particles.m_posit;
//	dgVector* const veloc = m_particles.m_veloc;
	const dgFloat32* const masses = m_particles.m_unitMass;

//	dgBody* const body = GetBody();
	// calculate velocity contribution by each particle regions
//	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
//		m_regions[i].UpdateVelocities(this, timestep, m_stiffness);
//	}

	dgInt32 strideInBytes = sizeof (dgVector) * m_regionsCount + sizeof (dgMatrix) * m_regionsCount + 3 * sizeof (dgVector) * m_particles.m_count;
	m_world->m_solverMatrixMemory.ExpandCapacityIfNeessesary (1, strideInBytes);
	dgMatrix* const sumQiPi = (dgMatrix*) &m_world->m_solverMatrixMemory[0];
	dgVector* const covarianceMatrix = (dgVector*) &sumQiPi[m_regionsCount];
	dgVector* const regionCom = &covarianceMatrix[m_particles.m_count * 3];

	memset (sumQiPi, 0, m_regionsCount * sizeof (dgMatrix));
	memset (regionCom, 0, m_regionsCount * sizeof (dgVector));
	memset (covarianceMatrix, 0, 3 * m_regionsCount * sizeof (dgVector));

//	const dgCollisionDeformableSolidMesh::dgParticle& particles = me->m_particles;
//	const dgFloat32* const mass = particles.m_unitMass;
//	const dgVector* const posit0 = me->m_shapePosit;
//	const dgVector* const posit1 = particles.m_posit;
//	dgVector com = dgVector (dgFloat32 (0.0f));
//	for (dgInt32 i = 0; i < m_count; i ++) {
//		dgInt32 index = m_indices[i];
//		com += posit1[index].Scale4 (mass[index]);
//	}
//	m_com = com.Scale4 (dgFloat32 (1.0f) / m_totalMass);
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgVector& r = m_posit[i];
		dgVector mr (r.Scale4(masses[i]));
		covarianceMatrix[i * 3 + 0] = mr.CompProduct4(r.BroadcastX()); 
		covarianceMatrix[i * 3 + 1] = mr.CompProduct4(r.BroadcastY()); 
		covarianceMatrix[i * 3 + 2] = mr.CompProduct4(r.BroadcastZ()); 

		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			regionCom[index] += mr;
		}
	}

	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		regionCom[i] = regionCom[i].Scale4 (dgFloat32 (1.0f) / m_regionMass[i]);
	}

//	dgMatrix sumQiPi (dgGetZeroMatrix());
//	for (dgInt32 i = 0; i < m_count; i ++) {
//		dgInt32 index = m_indices[i];
//		dgVector pi (posit1[index] - m_com);
//		dgVector qi (posit0[index] - m_com0);
//		dgFloat32 massI = mass[index];
//		for (dgInt32 j = 0; j < 3; j ++) {
//			for (dgInt32 k = 0; k < 3; k ++) {
//				sumQiPi[j][k] += massI * pi[j] * qi[k];
//			}
//		}
//	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;

		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			sumQiPi[index].m_front += covarianceMatrix[i * 3 + 0];
			sumQiPi[index].m_up += covarianceMatrix[i * 3 + 1];
			sumQiPi[index].m_right += covarianceMatrix[i * 3 + 2];
		}
	}

/*
	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		dgMatrix matrixsumQiPi;
		dgVector eigenValues;

		matrixsumQiPi.m_front = sumQiPi[i * 3 + 0];
		matrixsumQiPi.m_up = sumQiPi[i * 3 + 1];
		matrixsumQiPi.m_right = sumQiPi[i * 3 + 2];
		matrixsumQiPi.m_posit = dgVector::m_wOne;

		//dgAssert (dgAbsf (sumQiPi[1][0] - sumQiPi[0][1]) < dgFloat32 (1.0e-6f));
		//dgAssert (dgAbsf (sumQiPi[2][0] - sumQiPi[0][2]) < dgFloat32 (1.0e-6f));
		//dgAssert (dgAbsf (sumQiPi[2][1] - sumQiPi[1][2]) < dgFloat32 (1.0e-6f));
		//sumQiPi[3][3] = dgFloat32 (1.0f);
		dgAssert (dgAbsf (matrixsumQiPi[1][0] - matrixsumQiPi[0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (matrixsumQiPi[2][0] - matrixsumQiPi[0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (matrixsumQiPi[2][1] - matrixsumQiPi[1][2]) < dgFloat32 (1.0e-6f));

		//dgMatrix S (sumQiPi * sumQiPi.Transpose4X4());
		//S.EigenVectors (eigenValues, m_rotSeed);
		//m_rotSeed = S;

		dgMatrix S (matrixsumQiPi * matrixsumQiPi.Transpose4X4());
		S.EigenVectors (eigenValues, m_regionRotSeed[i]);
		m_regionRotSeed[i] = S;

#ifdef _DEBUG
//		dgMatrix P0 (sumQiPi * sumQiPi.Transpose4X4());
//		dgMatrix D (dgGetIdentityMatrix());
//		D[0][0] = eigenValues[0];
//		D[1][1] = eigenValues[1];
//		D[2][2] = eigenValues[2];
//		dgMatrix P1 (S.Transpose4X4() * D * S);

//		dgMatrix xx (P1 * P0.Symetric3by3Inverse());
//		dgAssert (dgAbsf (xx[0][0] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[1][1] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[2][2] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[0][1]) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[0][2]) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[1][0]) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[1][2]) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[2][0]) < dgFloat32 (1.0e-3f));
//		dgAssert (dgAbsf (xx[2][1]) < dgFloat32 (1.0e-3f));

#endif

		eigenValues = eigenValues.InvSqrt();

//		dgMatrix m (S);
//		m.m_front = m.m_front.Scale4 (eigenValues.m_x);
//		m.m_up    = m.m_up.Scale4 (eigenValues.m_y);
//		m.m_right = m.m_right.Scale4 (eigenValues.m_z);

		dgMatrix m;
		m.m_front = m.m_front.CompProduct4(eigenValues.BroadcastX());
		m.m_up = m.m_up.CompProduct4(eigenValues.BroadcastX());
		m.m_right = m.m_right.CompProduct4(eigenValues.BroadcastX());
		m.m_posit = dgVector::m_wOne;

		//dgMatrix invS = S.Transpose4X4() * m;
		dgMatrix invS (S.Transpose4X4() * m);
		dgMatrix R (invS * matrixsumQiPi[i]);
		dgMatrix A (m_regionAqqInv[i] * matrixsumQiPi[i]);

		dgVector beta0 (dgFloat32 (0.99f));
		dgVector beta1 (dgVector::m_one - beta0);
		sumQiPi[i].m_front = A.m_front.CompProduct4(beta0) + R.m_front.CompProduct4(beta1);
		sumQiPi[i].m_up = A.m_up.CompProduct4(beta0) + R.m_up.CompProduct4(beta1);
		sumQiPi[i].m_right = A.m_right.CompProduct4(beta0) + R.m_right.CompProduct4(beta1);
		sumQiPi[i].m_posit = dgVector::m_wOne;
	}



//	dgVector invTimeScale (stiffness / timestep);
//	dgVector* const veloc = particles.m_veloc;
//	for (dgInt32 i = 0; i < m_count; i ++) {
//		dgInt32 index = m_indices[i];
//		dgVector gi (deformationMatrix.RotateVector(posit0[index] - m_com0) + m_com);
//		veloc[index] += (gi - posit1[index]).CompProduct4 (invTimeScale);
//		//dgTrace (("%f %f %f\n", veloc[i][0], veloc[i][1], veloc[i][2] ));
//	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;

		const dgVector& p = m_shapePosit[i];
		dgVector averageVeloc (dgFloat32(0.0f));
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			const dgMatrix& matrix = sumQiPi[index];
			dgVector gi (matrix.RotateVector(p - m_regionCom0[index]) + regionCom[index]);
		}
	}
*/

/*
	// resolve collisions here
//for now just a hack a collision plane until I get the engine up an running
for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
	if (posit[i].m_y < dgFloat32 (0.0f)) {
		posit[i].m_y = dgFloat32 (0.0f);
		veloc[i].m_y = dgFloat32 (0.0f);
	}
}



	// integrate particle positions
	dgVector invTimeStep (timestep);
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		posit[i] += veloc[i].CompProduct4 (invTimeStep);
//dgTrace (("%f %f %f\n", posit[i][0], posit[i][1], posit[i][2] ));
	}
//dgTrace(("\n"));

*/


/*
	// integrate each particle by the deformation velocity, also calculate the new com
	dgFloat32 dampCoef = 0.0f;
	dgVector com (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		instantVelocity[i] += internalVelocity[i].Scale3 (dampCoef);
		dgVector step (internalVelocity[i].Scale3 (timestep));
		deltaPositions[i] += step;
		positions[i] += step;
		com += positions[i];
	}

	// center the particles around the new geometrics center of mass
	dgVector oldCom (m_particles.m_com);
	m_particles.m_com = com.Scale (dgFloat32 (1.0f) / m_particles.m_count); 

	// calculate the new body average velocity
	myBody->m_veloc = (m_particles.m_com - oldCom).Scale (dgFloat32 (1.0f) / timestep);
	myBody->m_globalCentreOfMass = m_particles.m_com; 
	myBody->m_matrix.m_posit = m_particles.m_com; 

	//myBody->UpdateMatrix (timestep, threadIndex);
	if (myBody->m_matrixUpdate) {
		myBody->m_matrixUpdate (*myBody, myBody->m_matrix, threadIndex);
	}

	// the collision changed shape, need to update spatial structure 
	UpdateCollision ();

	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
*/
}
