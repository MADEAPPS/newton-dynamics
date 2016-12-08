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

#include "dgWorld.h"
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgDynamicBody.h"
#include "dgCollisionBVH.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableMesh.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_SMALLEST_SPRING_LENGTH						dgFloat32 (1.0e-3f) 
dgVector dgCollisionDeformableMesh::m_smallestLenght2	(DG_SMALLEST_SPRING_LENGTH * DG_SMALLEST_SPRING_LENGTH);


dgCollisionDeformableMesh::dgCollisionDeformableMesh(dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID)
	:dgCollisionConvex(mesh->GetAllocator(), 0, collsionID)
	,m_posit(NULL)
	,m_veloc(NULL)
	,m_accel(NULL)
	,m_linkList(NULL)
	,m_restlength(NULL)
	,m_externalforce(NULL)
	,m_unitMassScaler(NULL)
	,m_linksCount(mesh->GetEdgeCount() / 2)
	,m_particlesCount(mesh->GetVertexCount())
{
	m_rtti |= dgCollisionDeformableMesh_RTTI;

	m_unitMassScaler = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_particlesCount);
	m_posit = (dgVector*)dgMallocStack(sizeof(dgVector) * m_particlesCount);
	m_veloc = (dgVector*)dgMallocStack(sizeof(dgVector) * m_particlesCount);
	m_accel = (dgVector*)dgMallocStack(sizeof(dgVector) * m_particlesCount);
	m_externalforce = (dgVector*)dgMallocStack(sizeof(dgVector) * m_particlesCount);
	m_linkList = (dgSoftLink*)dgMallocStack(sizeof(dgSoftLink) * m_linksCount);
	m_restlength = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_linksCount);

	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		dgBigVector p(mesh->GetVertex(i));
		m_posit[i] = p;
		com += m_posit[i];
		m_unitMassScaler[i] = dgFloat32 (1.0f);
		m_externalforce[i] = dgVector(dgFloat32(0.0f));
	}

	// for now use a fix size box
	m_boxSize = dgVector (dgFloat32 (1.0f), dgFloat32(1.0f), dgFloat32(1.0f), dgFloat32(0.0f));
	m_boxOrigin = com.CompProduct4(dgFloat32(1.0f) / m_particlesCount);

	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_accel[i] = dgVector(0.0f);
		m_veloc[i] = dgVector(0.0f);
		m_posit[i] = m_posit[i] - m_boxOrigin;
	}

	dgInt32 edgeCount = 0;
	for (void* edgePtr = mesh->GetFirstEdge(); edgePtr; edgePtr = mesh->GetNextEdge(edgePtr)) {
		dgInt32 v0;
		dgInt32 v1;
		mesh->GetEdgeIndex(edgePtr, v0, v1);
		m_linkList[edgeCount].m_v0 = dgInt16(v0);
		m_linkList[edgeCount].m_v1 = dgInt16(v1);
		dgVector dp(m_posit[v0] - m_posit[v1]);
		m_restlength[edgeCount] = dgSqrt(dp.DotProduct3(dp));
		dgAssert(edgeCount < m_linksCount);
		edgeCount++;
	}
	dgAssert(edgeCount == m_linksCount);

m_linksCount = 1;
m_particlesCount = 2;

m_posit[1] = m_posit[0];
m_posit[1].m_y -= 1.0f;
m_restlength[0] = 1.0f;
m_linkList[0].m_v0 = 0;
m_linkList[0].m_v1 = 1;
}

dgCollisionDeformableMesh::dgCollisionDeformableMesh(const dgCollisionDeformableMesh& source)
	:dgCollisionConvex(source)
	,m_posit((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_particlesCount))
	,m_veloc((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_particlesCount))
	,m_accel((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_particlesCount))
	,m_linkList((dgSoftLink*)dgMallocStack(sizeof(dgSoftLink) * source.m_linksCount))
	,m_restlength((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_linksCount))
	,m_externalforce((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_particlesCount))
	,m_unitMassScaler((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_particlesCount))
	,m_linksCount(source.m_linksCount)
	,m_particlesCount(source.m_particlesCount)
{
	m_rtti = source.m_rtti;
	memcpy(m_veloc, source.m_veloc, m_particlesCount * sizeof(dgVector));
	memcpy(m_posit, source.m_posit, m_particlesCount * sizeof(dgVector));
	memcpy(m_accel, source.m_posit, m_particlesCount * sizeof(dgVector));

	memcpy(m_unitMassScaler, source.m_unitMassScaler, m_particlesCount * sizeof(dgFloat32));
	memcpy(m_linkList, source.m_linkList, m_linksCount * sizeof(dgSoftLink));
	memcpy(m_restlength, source.m_restlength, m_linksCount * sizeof(dgFloat32));
	memcpy(m_externalforce, source.m_externalforce, m_particlesCount * sizeof(dgVector));
}

dgCollisionDeformableMesh::dgCollisionDeformableMesh(dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionConvex(world, deserialization, userData, revisionNumber)
{
	dgAssert(0);
}


dgCollisionDeformableMesh::~dgCollisionDeformableMesh(void)
{
	dgFree(m_posit);
	dgFree(m_veloc);
	dgFree(m_accel);
	dgFree(m_linkList);
	dgFree(m_restlength);
	dgFree(m_externalforce);
	dgFree(m_unitMassScaler);
}

dgInt32 dgCollisionDeformableMesh::CalculateSignature() const
{
	dgAssert(0);
	return 0;
}

void dgCollisionDeformableMesh::SetCollisionBBox(const dgVector& p0, const dgVector& p1)
{
	dgAssert(0);
}

void dgCollisionDeformableMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert(0);
}


dgMatrix dgCollisionDeformableMesh::CalculateInertiaAndCenterOfMass(const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const
{
	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		com = matrix.RotateVector(m_posit[i].CompProduct4 (localScale));
	}
	dgVector den (dgFloat32(1.0f / m_particlesCount));
	dgMatrix inertia(dgGetIdentityMatrix());
	inertia.m_posit = com.CompProduct4(den);
	inertia.m_posit.m_w = dgFloat32(1.0f);

	return inertia;
}

void dgCollisionDeformableMesh::CalcAABB(const dgMatrix& matrix, dgVector& p0, dgVector& p1) const
{
	dgVector origin(matrix.TransformVector(m_boxOrigin));
	dgVector size(matrix.m_front.Abs().Scale4(m_boxSize.m_x) + matrix.m_up.Abs().Scale4(m_boxSize.m_y) + matrix.m_right.Abs().Scale4(m_boxSize.m_z));
	p0 = (origin - size) & dgVector::m_triplexMask;
	p1 = (origin + size) & dgVector::m_triplexMask;
}


dgInt32 dgCollisionDeformableMesh::GetLinksCount() const
{
	return m_linksCount;
}

void dgCollisionDeformableMesh::DisableInactiveLinks ()
{
	dgAssert (0);
	for (dgInt32 i = 0; i < m_linksCount; i ++) {
		dgInt32 v0 = m_linkList[i].m_v0;
		dgInt32 v1 = m_linkList[i].m_v1;
		if ((m_unitMassScaler[v0] == dgFloat32 (0.0f)) && (m_unitMassScaler[v1] == dgFloat32 (0.0f))) {
			m_linksCount --;
			dgSwap(m_linkList[m_linksCount], m_linkList[i]);
			i --;
		}
	}
}

dgInt32 dgCollisionDeformableMesh::GetParticleCount() const
{
	return m_particlesCount;
}

const dgInt16* dgCollisionDeformableMesh::GetLinks() const
{
	return &m_linkList[0].m_v0;
}

const dgVector* dgCollisionDeformableMesh::GetVelocity() const
{
	return m_veloc;
}

const dgVector* dgCollisionDeformableMesh::GetPositions() const
{
	return m_posit;
}

const dgInt32* dgCollisionDeformableMesh::GetParticleToVertexMap() const
{
	dgAssert(0);
	return 0;
//	return m_indexMap;
}


void dgCollisionDeformableMesh::ConstraintParticle(dgInt32 particleIndex, const dgVector& posit, const dgBody* const body)
{
	dgAssert(0);
}

void dgCollisionDeformableMesh::CollideMasses(dgDynamicBody* const myBody, dgBody* const otherBody)
{
}



void dgCollisionDeformableMesh::IntegrateForces(dgDynamicBody* const body, dgFloat32 timestep)
{
	dgAssert(body->m_invMass.m_w > dgFloat32(0.0f));

	// calculate particles accelerations
	CalculateAcceleration (timestep, body);

	const dgMatrix& matrix = body->GetCollision()->GetGlobalMatrix();
	
	dgVector damp(dgFloat32(1.0f));
	if (body->m_linearDampOn) {
		const dgFloat32 tau = dgFloat32(1.0f) / (dgFloat32(60.0f) * timestep);
		damp = dgVector(dgPow(dgFloat32(1.0f) - body->m_dampCoef.m_w, tau));
	}

	// rigid body dynamic state
	dgVector timeV (timestep);
	dgVector den (dgFloat32(1.0f / m_particlesCount));
	
	dgVector xxSum(dgFloat32(0.0f));
	dgVector xySum(dgFloat32(0.0f));
	dgVector xxSum2(dgFloat32(0.0f));
	dgVector comVeloc(dgFloat32(0.0f));

	dgVector angularMomentum(0.0f);
	dgVector origin(matrix.TransformVector(m_boxOrigin));
	dgVector unitMass (body->m_mass.m_w * den.GetScalar());
//	dgVector unitInvMass (body->m_invMass.m_w * m_massesCount);
	
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_veloc[i] = m_veloc[i].CompProduct4(damp) + m_accel[i].CompProduct4(timeV);
		m_posit[i] = m_posit[i] + m_veloc[i].CompProduct4(timeV);
		comVeloc += m_veloc[i];
		xxSum += m_posit[i];
		xxSum2 += m_posit[i].CompProduct4(m_posit[i]);
		xySum += m_posit[i].CompProduct4(m_posit[i].ShiftTripleRight());
		angularMomentum += m_posit[i].CrossProduct3(m_veloc[i]);
	}

	dgVector yySum(xxSum.ShiftTripleRight());
	dgVector com(xxSum.CompProduct4(den) + origin);
	dgVector pxx0(origin - com);
	dgVector pxy0(pxx0.ShiftTripleRight());
	dgVector Ixx(unitMass.CompProduct4(xxSum2 + xxSum.CompProduct4(pxx0.CompProduct4(dgVector::m_two))) + pxx0.CompProduct4(pxx0.Scale4(body->m_mass.m_w)));
	dgVector Ixy(unitMass.CompProduct4(xySum + xxSum.CompProduct4(pxy0) + yySum.CompProduct4(pxx0)) + pxx0.CompProduct4(pxy0.Scale4(body->m_mass.m_w)));
	comVeloc = comVeloc.CompProduct4(den);

	dgMatrix inertia(dgGetIdentityMatrix());
	inertia[0][0] = Ixx[1] + Ixx[2];
	inertia[1][1] = Ixx[0] + Ixx[2];
	inertia[2][2] = Ixx[0] + Ixx[1];

	inertia[0][1] = -Ixy[0];
	inertia[1][0] = -Ixy[0];
	inertia[0][2] = -Ixy[1];
	inertia[2][0] = -Ixy[1];
	inertia[1][2] = -Ixy[2];
	inertia[2][1] = -Ixy[2];
	body->m_invWorldInertiaMatrix = inertia.Symetric3by3Inverse();

	body->m_accel = dgVector(0.0f);
	body->m_alpha = dgVector(0.0f);
	angularMomentum = unitMass.CompProduct4(angularMomentum + pxx0.CrossProduct3(comVeloc));

	body->m_veloc = comVeloc;
	body->m_omega = body->m_invWorldInertiaMatrix.RotateVector(angularMomentum);

	com = xxSum.CompProduct4(den);
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_posit[i] -= com.Scale4(m_unitMassScaler[i]);
	}
}


void dgCollisionDeformableMesh::CalculateAcceleration(dgFloat32 timestep, const dgDynamicBody* const body)
{
// K is in [sec^2] a spring constant acceleration, not a spring force acceleration. 
// for now make a share value for all springs. later this is a per material feature.
dgFloat32 fKs = dgFloat32(100.0f);

	dgFloat32* const spring_A01 = dgAlloca(dgFloat32, m_linksCount);
	dgFloat32* const spring_B01 = dgAlloca(dgFloat32, m_linksCount);
	dgVector* const dx = dgAlloca(dgVector, m_linksCount);
	dgVector* const dv = dgAlloca(dgVector, m_linksCount);
	dgVector* const dpdv = dgAlloca(dgVector, m_linksCount);
	dgVector* const diag = dgAlloca(dgVector, m_particlesCount * 3);

	//dgVector den(dgFloat32(1.0f / m_massesCount));
	//dgVector unitforce(body->m_externalForce.CompProduct4(den));
	dgVector unitAccel(body->m_externalForce.CompProduct4(body->m_invMass.m_w));
	for (dgInt32 i = 0; i < m_particlesCount; i++) {
		m_accel[i] = unitAccel;
		diag[i] = dgVector::m_one;
	}

	dgFloat32 timestepTow = dgFloat32(2.0f) * timestep;
	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgInt32 j0 = m_linkList[i].m_v0;
		const dgInt32 j1 = m_linkList[i].m_v1;

		dv[i] = m_veloc[j0] - m_veloc[j1];
		dx[i] = m_posit[j0] - m_posit[j1];
		const dgVector& p0p1 = dx[i];
		const dgVector& v0v1 = dv[i];
		dpdv[i] = p0p1.CompProduct4(v0v1);

		dgVector length2(p0p1.DotProduct4(p0p1));
		dgVector mask(length2 > m_smallestLenght2);
		length2 = (length2 & mask) | length2.AndNot(mask);
		dgFloat32 length = length2.Sqrt().GetScalar();
		dgFloat32 lenghtRatio = m_restlength[i] / length;
		spring_A01[i] = - fKs * (dgFloat32(1.0f) - lenghtRatio) * timestep;
		spring_B01[i] = - fKs * lenghtRatio * timestepTow / length2.GetScalar();
	}

	dgVector timeV(timestep);
	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgInt32 j0 = m_linkList[i].m_v0;
		const dgInt32 j1 = m_linkList[i].m_v1;
		const dgVector& dv0 = dv[j0];
		const dgVector& dv1 = dv[j1];
		const dgVector& dpdv0 = dpdv[j0];
		const dgVector& dpdv1 = dpdv[j1];
		const dgVector A01(spring_A01[i]);
		const dgVector B01(spring_B01[i]);
		const dgVector dxdx(dx[i].CompProduct4(dx[i]));

		m_accel[j0] += (A01.CompProduct4(dv0) + B01.CompProduct4(dx[i].CompProduct4(dpdv0)));
		m_accel[j1] -= (A01.CompProduct4(dv1) + B01.CompProduct4(dx[i].CompProduct4(dpdv1)));
		
		diag[j0] -= timeV.CompProduct4(A01 + B01.CompProduct4(dxdx));
		diag[j1] -= timeV.CompProduct4(A01 + B01.CompProduct4(dxdx));
	}
}
