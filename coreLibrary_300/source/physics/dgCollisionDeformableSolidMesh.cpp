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
		m_regionAqqInv[i].m_up = mcr0.CompProduct4 (cr0.BroadcastY());
		m_regionAqqInv[i].m_right = mcr0.CompProduct4 (cr0.BroadcastZ());
		m_regionAqqInv[i].m_posit = dgVector::m_wOne;

		dgAssert (dgAbsf (m_regionAqqInv[i][1][0] - m_regionAqqInv[i][0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (m_regionAqqInv[i][2][0] - m_regionAqqInv[i][0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (m_regionAqqInv[i][2][1] - m_regionAqqInv[i][1][2]) < dgFloat32 (1.0e-6f));
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;

		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			dgMatrix& covariance = m_regionAqqInv[index];
			covariance.m_front += covarianceMatrix[i * 3 + 0];
			covariance.m_up += covarianceMatrix[i * 3 + 1];
			covariance.m_right += covarianceMatrix[i * 3 + 2];
			dgAssert (dgAbsf (covariance[1][0] - covariance[0][1]) < dgFloat32 (1.0e-6f));
			dgAssert (dgAbsf (covariance[2][0] - covariance[0][2]) < dgFloat32 (1.0e-6f));
			dgAssert (dgAbsf (covariance[2][1] - covariance[1][2]) < dgFloat32 (1.0e-6f));
		}
	}

	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		dgMatrix& covariance = m_regionAqqInv[i];
		dgAssert (dgAbsf (covariance[1][0] - covariance[0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (covariance[2][0] - covariance[0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (covariance[2][1] - covariance[1][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (covariance[3][3] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-6f));
		covariance = covariance.Symetric3by3Inverse();
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

void dgCollisionDeformableSolidMesh::SetMatrix(const dgMatrix& matrix)
{
   	dgAssert (m_myBody);
   	dgBody* const body = GetBody();
    dgMatrix globalMatrix (body->GetCollision()->GetLocalMatrix() * matrix);
    for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
        m_particles.m_posit[i] = m_shapePosit[i];
        m_posit[i] = matrix.TransformVector(m_shapePosit[i]);
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
	dgFloat32* const unitMass = m_particles.m_unitMass;

	dgVector com (dgFloat32 (0.0f));
	dgVector comVeloc (dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector mass (unitMass[i]);
        const dgVector& p = m_posit[i]; 
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

    InertiaMatrix.EigenVectors (matrix);
    InertiaMatrix.m_posit = matrix.m_posit;

    body->GetCollision()->SetGlobalMatrix(InertiaMatrix);
    body->SetMatrixOriginAndRotation(body->GetCollision()->GetLocalMatrix().Inverse() * InertiaMatrix);
}

void dgCollisionDeformableSolidMesh::ResolvePositionsConstraints (dgFloat32 timestep)
{
	dgAssert (m_myBody);

	dgInt32 strideInBytes = sizeof (dgVector) * m_regionsCount + sizeof (dgMatrix) * m_regionsCount + 3 * sizeof (dgVector) * m_particles.m_count;
	m_world->m_solverMatrixMemory.ExpandCapacityIfNeessesary (1, strideInBytes);
	dgVector* const regionCom = (dgVector*)&m_world->m_solverMatrixMemory[0];
	dgMatrix* const sumQiPi = (dgMatrix*) &regionCom[m_regionsCount];
	dgVector* const covarianceMatrix = (dgVector*) &sumQiPi[3 * m_regionsCount];

	const dgFloat32* const masses = m_particles.m_unitMass;
	dgVector zero (dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		regionCom[i] = zero;
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgVector& r = m_posit[i];
		dgVector mr (r.Scale4(masses[i]) & dgVector::m_triplexMask);
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
		dgVector mcr (regionCom[i]);
		dgVector cr (mcr.Scale4 (dgFloat32 (1.0f) / m_regionMass[i]));
		mcr = mcr.CompProduct4(dgVector::m_negOne);
		regionCom[i] = cr;

		dgMatrix& QiPi = sumQiPi[i];
		QiPi.m_front = mcr.CompProduct4 (cr.BroadcastX());
		QiPi.m_up = mcr.CompProduct4 (cr.BroadcastY());
		QiPi.m_right = mcr.CompProduct4 (cr.BroadcastZ());
		QiPi.m_posit = dgVector::m_wOne;
		dgAssert (dgAbsf (QiPi[1][0] - QiPi[0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (QiPi[2][0] - QiPi[0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (QiPi[2][1] - QiPi[1][2]) < dgFloat32 (1.0e-6f));
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;

		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			dgMatrix& covariance = sumQiPi[index];
			covariance.m_front += covarianceMatrix[i * 3 + 0];
			covariance.m_up += covarianceMatrix[i * 3 + 1];
			covariance.m_right += covarianceMatrix[i * 3 + 2];
			dgAssert (dgAbsf (covariance[1][0] - covariance[0][1]) < dgFloat32 (1.0e-6f));
			dgAssert (dgAbsf (covariance[2][0] - covariance[0][2]) < dgFloat32 (1.0e-6f));
			dgAssert (dgAbsf (covariance[2][1] - covariance[1][2]) < dgFloat32 (1.0e-6f));
		}
	}


	dgVector beta0 (dgFloat32 (0.99f));
	dgVector beta1 (dgVector::m_one - beta0);
	dgFloat32 stiffness = dgFloat32 (0.8f);

	for (dgInt32 i = 0; i < m_regionsCount; i ++) {
		dgMatrix& QiPi = sumQiPi[i];
		dgAssert (dgAbsf (QiPi[1][0] - QiPi[0][1]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (QiPi[2][0] - QiPi[0][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (QiPi[2][1] - QiPi[1][2]) < dgFloat32 (1.0e-6f));
		dgAssert (dgAbsf (QiPi[3][3] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-6f));

		dgMatrix S (QiPi * QiPi.Transpose4X4());
		dgVector eigenValues;

		S.EigenVectors (eigenValues, m_regionRotSeed[i]);
		m_regionRotSeed[i] = S;

#if 0
		dgMatrix P0 (QiPi * QiPi.Transpose4X4());
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

		dgMatrix m;
		m.m_front = S.m_front.Scale4 (eigenValues.m_x);
		m.m_up    = S.m_up.Scale4 (eigenValues.m_y);
		m.m_right = S.m_right.Scale4 (eigenValues.m_z);
		m.m_posit = dgVector::m_wOne;
		dgMatrix invS (S.Transpose4X4() * m);
		dgMatrix R (invS * QiPi);
		dgMatrix A (m_regionAqqInv[i] * QiPi);

		QiPi.m_front = A.m_front.CompProduct4(beta0) + R.m_front.CompProduct4(beta1);
		QiPi.m_up = A.m_up.CompProduct4(beta0) + R.m_up.CompProduct4(beta1);
		QiPi.m_right = A.m_right.CompProduct4(beta0) + R.m_right.CompProduct4(beta1);
	}


	dgVector invTimeScale (stiffness / timestep);
	dgVector* const veloc = m_particles.m_veloc;

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_positRegionsStart[i];
		const dgInt32 count = m_positRegionsStart[i + 1] - start;

		const dgVector& p0 = m_posit[i];
		const dgVector& q0 = m_shapePosit[i];
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_positRegions[start + j];
			const dgMatrix& matrix = sumQiPi[index];
			dgVector gi (matrix.RotateVector(q0 - m_regionCom0[index]) + regionCom[index]);
			dgVector v ((gi - p0).CompProduct4(invTimeScale).Scale4 (m_weight[index]));
			veloc[index] += v;
		}
	}


// resolve collisions here
//for now just a hack a collision plane until I get the engine up an running
for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
	if (m_posit[i].m_y < dgFloat32 (0.0f)) {
		m_posit[i].m_y = dgFloat32 (0.0f);
		veloc[i].m_y = dgFloat32 (0.0f);
	}
}

	dgVector time (timestep);
    dgVector minBox (dgFloat32 (1.0e10f));
    dgVector maxBox (dgFloat32 (-1.0e10f));
    dgMatrix matrix (m_myBody->GetCollision()->GetGlobalMatrix().Inverse());
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_posit[i] += veloc[i].CompProduct4 (time);
		minBox = minBox.GetMin(m_posit[i]);
        maxBox = maxBox.GetMax(m_posit[i]);
        m_particles.m_posit[i] = matrix.TransformVector(m_posit[i]);
	}

	// integrate each particle by the deformation velocity, also calculate the new com
	// calculate the new body average velocity
//	myBody->m_veloc = (m_particles.m_com - oldCom).Scale (dgFloat32 (1.0f) / timestep);
//	if (myBody->m_matrixUpdate) {
//		myBody->m_matrixUpdate (*myBody, myBody->m_matrix, threadIndex);
//	}
	// the collision changed shape, need to update spatial structure 
//	UpdateCollision ();
//	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
}
