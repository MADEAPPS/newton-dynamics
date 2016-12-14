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
#include "dgCollisionBVH.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableSolidMesh.h"


dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgMeshEffect* const mesh)
	:dgCollisionDeformableMesh(world, m_deformableSolidMesh)
{
	m_rtti |= dgCollisionDeformableSolidMesh_RTTI;

//	dgArray<dgVector> m_posit;
//	dgArray<dgVector> m_veloc;
//	dgArray<dgVector> m_accel;
//	dgArray<dgVector> m_externalforce;

	dgInt32 count = mesh->GetVertexCount();
	dgVector* const points = dgAlloca (dgVector, count);

	for (dgInt32 i = 0; i < count; i++) {
		dgBigVector p(mesh->GetVertex(i));
		points[i] = p;
	}
	m_indexToVertexMap.Resize(count);
	dgInt32* const indexToVertexMap = &m_indexToVertexMap[0];
	m_particlesCount = dgVertexListToIndexList (&points[0].m_x, sizeof (dgVector), 3 * sizeof (dgFloat32), 0, count, indexToVertexMap, dgFloat32 (1.0e-5f));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_posit[i] = points[i];
	}

	dgInt32 edgeCount = 0;
	dgSoftLink* const links = dgAlloca (dgSoftLink, mesh->GetCount() / 2);
	for (void* edgePtr = mesh->GetFirstEdge(); edgePtr; edgePtr = mesh->GetNextEdge(edgePtr)) {
		dgInt32 v0;
		dgInt32 v1;
		mesh->GetEdgeIndex(edgePtr, v0, v1);
		v0 = indexToVertexMap[v0];
		v1 = indexToVertexMap[v1];
		links[edgeCount].m_m0 = dgInt16(dgMin (v0, v1));
		links[edgeCount].m_m1 = dgInt16(dgMax (v0, v1));
		edgeCount ++;
	}
	dgAssert(edgeCount == mesh->GetCount() / 2);
	dgSort(links, edgeCount, CompareEdges);

	dgInt32 uniqueEdgeCount = 0;
	for (dgInt32 i = 1; i < edgeCount; i ++) {
		if (CompareEdges (&links[i], &links[uniqueEdgeCount], NULL) > 0) {
			uniqueEdgeCount++;
			links[uniqueEdgeCount] = links[i];
		}
	}
	uniqueEdgeCount++;
	m_linksCount = uniqueEdgeCount;
	m_indexToVertexMap.Resize(m_linksCount);
	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		m_linkList[i] = links[i];
	}
	FinalizeBuild();
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(const dgCollisionDeformableSolidMesh& source)
	:dgCollisionDeformableMesh(source)
{
	m_rtti |= source.m_rtti;
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionDeformableMesh(world, deserialization, userData, revisionNumber)
{
}


dgCollisionDeformableSolidMesh::~dgCollisionDeformableSolidMesh(void)
{
}


dgInt32 dgCollisionDeformableSolidMesh::CompareEdges (const dgSoftLink* const A, const dgSoftLink* const B, void* const context)
{
	dgInt64 m0 = (dgInt64(A->m_m0)<<32) + A->m_m1;
	dgInt64 m1 = (dgInt64(B->m_m0)<<32) + B->m_m1;
	if (m0 > m1) {
		return 1;
	} else if (m0 < m1) {
		return -1;
	} 
	return 0;
}

void dgCollisionDeformableSolidMesh::DebugCollision(const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	const dgVector* const posit = &m_posit[0];
	const dgSoftLink* const links = &m_linkList[0];
	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgInt32 j0 = links[i].m_m0;
		const dgInt32 j1 = links[i].m_m1;
		dgVector p0 (matrix.TransformVector(posit[j0]));
		dgVector p1 (matrix.TransformVector(posit[j1]));
		dgTriplex points[2];
		points[0].m_x = p0.m_x;
		points[0].m_y = p0.m_y;
		points[0].m_z = p0.m_z;
		points[1].m_x = p1.m_x;
		points[1].m_y = p1.m_y;
		points[1].m_z = p1.m_z;
		callback(userData, 2, &points[0].m_x, 0);
	}
}


void dgCollisionDeformableSolidMesh::CalculateAcceleration(dgFloat32 timestep)
{
// Ks is in [sec^-2] a spring constant unit acceleration, not a spring force acceleration. 
// Kc is in [sec^-1] a damper constant unit velocity, not a damper force acceleration. 

// for now make a share value for all springs. later this is a per material feature.
dgFloat32 kSpring = dgFloat32(1000.0f);
dgFloat32 kDamper = dgFloat32(15.0f);


	dgInt32 iter = 4;
	dgVector* const accel = &m_accel[0];
	dgVector* const veloc = &m_veloc[0];
	dgVector* const posit = &m_posit[0];
	const dgFloat32* const restLenght = &m_restlength[0];

	dgFloat32* const spring_A01 = dgAlloca(dgFloat32, m_linksCount);
	dgFloat32* const spring_B01 = dgAlloca(dgFloat32, m_linksCount);
//	dgFloat32* const damper_A01 = dgAlloca(dgFloat32, m_linksCount);
//	dgFloat32* const damper_B01 = dgAlloca(dgFloat32, m_linksCount);
//	dgFloat32* const damper_C01 = dgAlloca(dgFloat32, m_linksCount);
	dgVector* const dx = dgAlloca(dgVector, m_linksCount);
	dgVector* const dv = dgAlloca(dgVector, m_linksCount);

	dgVector* const collisionDir0 = dgAlloca(dgVector, m_particlesCount);
	dgVector* const collisionDir1 = dgAlloca(dgVector, m_particlesCount);
	dgVector* const collisionDir2 = dgAlloca(dgVector, m_particlesCount);
	dgVector* const collidingAccel = dgAlloca(dgVector, m_particlesCount);
	dgVector* const tmp0 = dgAlloca(dgVector, m_particlesCount);
	dgVector* const tmp1 = dgAlloca(dgVector, m_particlesCount);
	dgVector* const tmp2 = dgAlloca(dgVector, m_particlesCount);
	dgVector* const dpdv = dgAlloca(dgVector, m_linksCount);
//	dgVector* const diag = dgAlloca(dgVector, m_particlesCount);
//	dgVector* const offDiag = dgAlloca(dgVector, m_particlesCount);
	dgVector* const veloc0 = dgAlloca(dgVector, m_particlesCount);

	dgVector unitAccel(m_body->m_externalForce.CompProduct4(m_body->m_invMass.m_w));
	dgVector deltaOmega(m_body->m_invWorldInertiaMatrix.RotateVector (m_body->m_externalTorque.Scale4 (timestep)));

	m_body->m_alpha = dgVector::m_zero;
	m_body->m_omega = dgVector::m_zero;
	m_body->m_externalForce = dgVector::m_zero;
	m_body->m_externalTorque = dgVector::m_zero;

	// here I need to add all other external acceleration like wind and pressure, friction and collision.
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		accel[i] = unitAccel;
		veloc[i] += deltaOmega.CrossProduct3(m_posit[i]);
		veloc0[i] = veloc[i];
	}

	const dgSoftLink* const links = &m_linkList[0];
	dgFloat32 ks_dt = - timestep * kSpring;
//	dgFloat32 kd_dt0 = -timestep * kDamper;
//	dgFloat32 kd_dt1 = -timestep * kDamper * dgFloat32(2.0f);

	dgVector dtRK4 (timestep / iter);
	HandleCollision (timestep, collisionDir0, collisionDir1, collisionDir2, collidingAccel);
	for (dgInt32 k = 0; k < iter; k ++) {
		for (dgInt32 i = 0; i < m_linksCount; i++) {
			const dgInt32 j0 = links[i].m_m0;
			const dgInt32 j1 = links[i].m_m1;
			dv[i] = veloc[j0] - veloc[j1];
			dx[i] = posit[j0] - posit[j1];

			const dgVector p0p1 (dx[i]);
			const dgVector v0v1 (dv[i]);
			const dgVector length2(p0p1.DotProduct4(p0p1));
			const dgVector mask(length2 > m_smallestLenght2);

			const dgVector lenght2 ((length2 & mask) | length2.AndNot(mask));
			const dgFloat32 length = (lenght2.Sqrt()).GetScalar();
			const dgFloat32 den = dgFloat32 (1.0f) / length;
			const dgFloat32 lenghtRatio = restLenght[i] * den;
			const dgFloat32 compression = dgFloat32(1.0f) - lenghtRatio;
			const dgVector fs(p0p1.Scale4(kSpring * compression));
			const dgVector fd(p0p1.Scale4(kDamper * den * den * (v0v1.DotProduct4(p0p1)).GetScalar()));

			dpdv[i] = p0p1.CompProduct4(v0v1);

			accel[j0] -= (fs + fd);
			accel[j1] += (fs + fd);

			spring_A01[i] = ks_dt * compression;
			spring_B01[i] = ks_dt * lenghtRatio * den * den;
		}

		for (dgInt32 i = 0; i < m_linksCount; i++) {
			const dgVector dv0 (dv[i]);
			const dgVector A01(spring_A01[i]);
			const dgVector B01(spring_B01[i]);
			const dgVector dfdx (A01.CompProduct4(dv0) + B01.CompProduct4(dx[i].CompProduct4(dpdv[i])));

			const dgInt32 j0 = links[i].m_m0;
			const dgInt32 j1 = links[i].m_m1;
			accel[j0] += dfdx;
			accel[j1] -= dfdx;
		}

		for (dgInt32 i = 0; i < m_particlesCount; i++) {
			tmp0[i] = accel[i].CompProduct4(collisionDir0[i]);
			tmp1[i] = accel[i].CompProduct4(collisionDir1[i]);
			tmp2[i] = accel[i].CompProduct4(collisionDir2[i]);
		}

		for (dgInt32 i = 0; i < m_particlesCount; i++) {
			accel[i] = accel[i] - collisionDir0[i].CompProduct4(tmp0[i]) - collisionDir1[i].CompProduct4(tmp1[i])  - collisionDir2[i].CompProduct4(tmp2[i]) + collidingAccel[i];
		}

		for (dgInt32 i = 0; i < m_particlesCount; i++) {
			veloc[i] += accel[i].CompProduct4 (dtRK4);
			posit[i] += veloc[i].CompProduct4(dtRK4);
			accel[i] = unitAccel;
		}
	}

/*
	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgInt32 j0 = links[i].m_v0;
		const dgInt32 j1 = links[i].m_v1;

		dv[i] = veloc[j0] - veloc[j1];
		dx[i] = posit[j0] - posit[j1];
		const dgVector& p0p1 = dx[i];
		const dgVector& v0v1 = dv[i];
		dpdv[i] = p0p1.CompProduct4(v0v1);

		dgVector length2(p0p1.DotProduct4(p0p1));
		dgVector mask(length2 > m_smallestLenght2);
		length2 = (length2 & mask) | length2.AndNot(mask);
		dgFloat32 length = length2.Sqrt().GetScalar();
		dgFloat32 den = dgFloat32 (1.0f) / length;
//		dgFloat32 den2 = den * den;
		dgFloat32 lenghtRatio = m_restlength[i] * den;

		dgFloat32 compression = dgFloat32(1.0f) - lenghtRatio;
		dgVector fs(p0p1.Scale4(kSpring * compression));
		//dgVector fd(p0p1.Scale4(kDamper * den * v0v1.DotProduct4(p0p1).GetScalar()));

		accel[j0] -= fs;
		accel[j1] += fs;

		spring_A01[i] = ks_dt0 * compression;
		spring_B01[i] = ks_dt1 * lenghtRatio / length2.GetScalar();
//		damper_A01[i] = 
	}

	dgVector timeV(timestep);
	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgVector dx0 (dx[i]);
		const dgVector dv0 (dv[i]);
		const dgVector dpdv0 (dpdv[i]);
		const dgVector A01(spring_A01[i]);
		const dgVector B01(spring_B01[i]);
		const dgVector dxdx(dx0.CompProduct4(dx0));
		const dgVector dxdy(dx0.CompProduct4(dx0.ShiftTripleRight()));
		const dgVector diag0 (timeV.CompProduct4(A01 + B01.CompProduct4(dxdx)));
		const dgVector offDiag0 (timeV.CompProduct4(A01 + B01.CompProduct4(dxdy)));
		const dgVector dfdxdv0 (A01.CompProduct4(dv0) + B01.CompProduct4(dx[i].CompProduct4(dpdv0)));

		const dgInt32 j0 = links[i].m_v0;
		const dgInt32 j1 = links[i].m_v1;

		accel[j0] += dfdxdv0;
		accel[j1] -= dfdxdv0;
		
		diag[j0] -= diag0;
		diag[j1] -= diag0;
		offDiag[j0] -= offDiag0;
		offDiag[j1] -= offDiag0;
	}
*/
}
