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

class dgCollisionDeformableMesh::dgSoftLink
{
	public:
	dgInt16 m_v0;
	dgInt16 m_v1;
};


/*
class dgCollisionDeformableMesh::dgSpringMassSolver
{
	public:
	dgSpringMassSolver(dgCollisionDeformableMesh* const me, const dgJacobianPair* const matrix, const dgFloat32* const invDiagonal, dgFloat32* const b, dgFloat32 unitInvMass)
		:m_unitInvMass(unitInvMass)
		,m_matrix(matrix)
		,m_invDiagonal(invDiagonal)
		,m_edgeCount(me->m_edgeCount)
		,m_massesCount(me->m_massesCount)
	{
		dgFloat32* const x = me->m_lambda;
		m_edgeList = me->m_edgeList;
		m_massScaler = me->m_unitMassScaler;
		
		m_buffer = dgAlloca(dgVector, m_massesCount);
		dgFloat32* const r0 = dgAlloca(dgFloat32, m_edgeCount);
		dgFloat32* const p0 = dgAlloca(dgFloat32, m_edgeCount);
		dgFloat32* const MinvR0 = dgAlloca(dgFloat32, m_edgeCount);
		dgFloat32* const matrixP0 = dgAlloca(dgFloat32, m_edgeCount);

		MatrixTimeVector(matrixP0, x);
		Sub(r0, b, matrixP0);
		InversePrecoditionerTimeVector(p0, r0);

		dgInt32 iter = 0;
		dgFloat32 num = DotProduct(r0, p0);
		dgFloat32 error2 = num;

		const dgInt32 maxPasses = 8;
		const dgFloat32 tolerance = dgFloat32(1.0e-2f);
		for (dgInt32 j = 0; (j < maxPasses) && (error2 > tolerance); j++) {

			MatrixTimeVector(matrixP0, p0);
			dgFloat32 den = DotProduct(p0, matrixP0);

			dgAssert(fabs(den) > dgFloat32(0.0f));
			dgFloat32 alpha = num / den;

			ScaleAdd(x, x, alpha, p0);
			ScaleAdd(r0, r0, -alpha, matrixP0);
			InversePrecoditionerTimeVector(MinvR0, r0);

			dgFloat32 num1 = DotProduct(r0, MinvR0);
			dgFloat32 beta = num1 / num;
			ScaleAdd(p0, MinvR0, beta, p0);
			num = DotProduct(r0, MinvR0);
			iter++;
			error2 = num;
		}
	}

	private:
	DG_INLINE void MatrixTimeVector(dgFloat32* const out, const dgFloat32* const v)
	{
		memset(m_buffer, 0, m_massesCount * sizeof(dgVector));
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			const dgInt32 j0 = m_edgeList[i].m_v0;
			const dgInt32 j1 = m_edgeList[i].m_v1;
			dgAssert(j0 < m_massesCount);
			dgAssert(j1 < m_massesCount);
			dgVector lamda(v[i]);
			const dgJacobianPair* const jacobian = &m_matrix[i];
			m_buffer[j0] += jacobian->m_j01.CompProduct4(lamda);
			m_buffer[j1] += jacobian->m_j10.CompProduct4(lamda);
		}

		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			const dgInt32 j0 = m_edgeList[i].m_v0;
			const dgInt32 j1 = m_edgeList[i].m_v1;
			dgAssert(j0 < m_massesCount);
			dgAssert(j1 < m_massesCount);
			const dgJacobianPair* const jacobian = &m_matrix[i];
			dgVector invMass0(m_unitInvMass.Scale4(m_massScaler[j0]));
			dgVector invMass1(m_unitInvMass.Scale4(m_massScaler[j1]));
			dgVector accel(jacobian->m_j01.CompProduct4(invMass0).CompProduct4(m_buffer[j0]) + jacobian->m_j10.CompProduct4(invMass1).CompProduct4(m_buffer[j1]));
			out[i] = (accel.AddHorizontal()).GetScalar();
		}
	}

	DG_INLINE void Sub(dgFloat32* const a, const dgFloat32* const b, const dgFloat32* const c) const
	{
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			a[i] = b[i] - c[i];
		}
	}

	DG_INLINE void InversePrecoditionerTimeVector(dgFloat32* const out, const dgFloat32* const v) const
	{
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			out[i] = v[i] * m_invDiagonal[i];
		}
	}

	DG_INLINE dgFloat32 DotProduct(const dgFloat32* const b, const dgFloat32* const c) const
	{
		dgFloat32 product = dgFloat64(0.0f);
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			product += b[i] * c[i];
		}
		return product;
	}

	void ScaleAdd(dgFloat32* const a, const dgFloat32* const b, dgFloat32 scale, const dgFloat32* const c) const
	{
		for (dgInt32 i = 0; i < m_edgeCount; i++) {
			a[i] = b[i] + scale * c[i];
		}
	}


	dgVector m_unitInvMass;
	dgVector* m_buffer;
	dgFloat32* m_massScaler;
	const dgEdge* m_edgeList;
	const dgJacobianPair* m_matrix;
	const dgFloat32* m_invDiagonal;
	dgInt32 m_edgeCount;
	dgInt32 m_massesCount;
};
*/


						   
dgCollisionDeformableMesh::dgCollisionDeformableMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData, dgInt32 revisionNumber)
	:dgCollisionConvex (world, deserialization, userData, revisionNumber)
{
	dgAssert (0);
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


dgCollisionDeformableMesh::~dgCollisionDeformableMesh(void)
{
	dgFree(m_posit);
	dgFree(m_veloc);
	dgFree(m_accel);
	dgFree(m_linkList);
	dgFree(m_indexMap);
	dgFree(m_restlength);
	dgFree(m_externalforce);
	dgFree(m_unitMassScaler);
}

dgCollisionDeformableMesh::dgCollisionDeformableMesh(dgWorld* const world, dgMeshEffect* const mesh, dgCollisionID collsionID)
	:dgCollisionConvex(mesh->GetAllocator(), 0, collsionID)
	,m_posit(NULL)
	,m_veloc(NULL)
	,m_accel(NULL)
	,m_linkList(NULL)
	,m_indexMap(NULL)
	,m_restlength(NULL)
	,m_externalforce(NULL)
	,m_unitMassScaler(NULL)
	,m_linksCount(0)
	,m_massesCount(0)
	,m_vertexCount(mesh->GetVertexCount())
{
	m_rtti |= dgCollisionDeformableMesh_RTTI;

	dgStack<dgInt32> indexList(m_vertexCount);
	dgStack<dgVector> positBuff(m_vertexCount);
	dgVector* const posit = &positBuff[0];
	const dgFloat64* const meshVertex = mesh->GetVertexPool();
	const dgInt32 stride = mesh->GetVertexStrideInByte() / sizeof (dgFloat64);
	for (dgInt32 i = 0; i < m_vertexCount; i++) {
		posit[i] = dgVector(dgFloat32(meshVertex[i * stride + 0]), dgFloat32(meshVertex[i * stride + 1]), dgFloat32(meshVertex[i * stride + 2]), dgFloat32(0.0f));
	}
	m_massesCount = dgVertexListToIndexList(&posit[0].m_x, sizeof(dgVector), sizeof(dgVector), 0, m_vertexCount, &indexList[0], dgFloat32(1.0e-8f));

	m_indexMap = (dgInt32*)dgMallocStack(sizeof(dgInt32) * m_vertexCount);
	m_unitMassScaler = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_massesCount);
	m_posit = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	m_veloc = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	m_accel = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	m_externalforce = (dgVector*)dgMallocStack(sizeof(dgVector) * m_massesCount);
	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		com += posit[i];
		m_unitMassScaler[i] = dgFloat32 (1.0f);
		m_externalforce[i] = dgVector(dgFloat32(0.0f));
	}
// for now use a fix size box
	m_boxSize = dgVector (dgFloat32 (1.0f), dgFloat32(1.0f), dgFloat32(1.0f), dgFloat32(0.0f));
	m_boxOrigin = com.CompProduct4(dgFloat32(1.0f) / m_massesCount);

	for (dgInt32 i = 0; i < m_massesCount; i++) {
		m_accel[i] = dgVector(0.0f);
		m_veloc[i] = dgVector(0.0f);
		m_posit[i] = posit[i] - m_boxOrigin;
	}

	for (dgInt32 i = 0; i < m_vertexCount; i++) {
		m_indexMap[i] = indexList[i];
	}

	dgPolyhedra polyhedra(GetAllocator());
	polyhedra.BeginFace();
	for (void* facePtr = mesh->GetFirstFace(); facePtr; facePtr = mesh->GetNextEdge(facePtr)) {
		if (!mesh->IsFaceOpen(facePtr)) {
			dgInt32 indices[256];
			dgInt32 count = mesh->GetFaceIndexCount (facePtr);
			dgAssert(count < 256);
			mesh->GetFaceIndex(facePtr, indices);
			for (dgInt32 i = 0; i < count; i ++) {
				dgInt32 j = indices[i];
				indices[i] = m_indexMap[j];
			}
			polyhedra.AddFace(3, indices);
		}
	}
	polyhedra.EndFace();
	m_linksCount = polyhedra.GetCount() / 2;
	m_linkList = (dgSoftLink*)dgMallocStack(sizeof(dgSoftLink) * m_linksCount);
	m_restlength = (dgFloat32*)dgMallocStack(sizeof(dgFloat32) * m_linksCount);

	dgInt32 edgeCount = 0;
	dgInt32 lru = polyhedra.IncLRU();
	dgPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) {
		::dgEdge& edge = iter.GetNode()->GetInfo();
		if (edge.m_mark != lru) {
			edge.m_mark = lru;
			edge.m_twin->m_mark = lru;
			const dgInt16 v0 = dgInt16(edge.m_incidentVertex);
			const dgInt16 v1 = dgInt16(edge.m_twin->m_incidentVertex);
			m_linkList[edgeCount].m_v0 = v0;
			m_linkList[edgeCount].m_v1 = v1;
			dgVector dp(m_posit[v0] - m_posit[v1]);
			m_restlength[edgeCount] = dgSqrt(dp.DotProduct3(dp));
			edgeCount++;
		}
	}
}

dgCollisionDeformableMesh::dgCollisionDeformableMesh(const dgCollisionDeformableMesh& source)
	:dgCollisionConvex(source)
	,m_posit((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_massesCount))
	,m_veloc((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_massesCount))
	,m_accel((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_massesCount))
	,m_indexMap((dgInt32*)dgMallocStack(sizeof(dgInt32) * source.m_vertexCount))
	,m_linkList((dgSoftLink*)dgMallocStack(sizeof(dgSoftLink) * source.m_linksCount))
	,m_restlength((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_linksCount))
	,m_externalforce((dgVector*)dgMallocStack(sizeof(dgVector) * source.m_vertexCount))
	,m_unitMassScaler((dgFloat32*)dgMallocStack(sizeof(dgFloat32) * source.m_massesCount))
	,m_linksCount(source.m_linksCount)
	,m_massesCount(source.m_massesCount)
	,m_vertexCount(source.m_vertexCount)
{
	m_rtti = source.m_rtti;
	memcpy(m_veloc, source.m_veloc, m_massesCount * sizeof(dgVector));
	memcpy(m_posit, source.m_posit, m_massesCount * sizeof(dgVector));
	memcpy(m_accel, source.m_posit, m_massesCount * sizeof(dgVector));

	memcpy(m_unitMassScaler, source.m_unitMassScaler, m_massesCount * sizeof(dgFloat32));
	memcpy(m_linkList, source.m_linkList, m_linksCount * sizeof(dgSoftLink));
	memcpy(m_indexMap, source.m_indexMap, m_vertexCount * sizeof(dgInt32));
	memcpy(m_restlength, source.m_restlength, m_linksCount * sizeof(dgFloat32));
	memcpy(m_externalforce, source.m_externalforce, m_massesCount * sizeof(dgVector));
}

dgMatrix dgCollisionDeformableMesh::CalculateInertiaAndCenterOfMass(const dgMatrix& m_alignMatrix, const dgVector& localScale, const dgMatrix& matrix) const
{
	dgVector com(dgFloat32(0.0f));
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		com = matrix.RotateVector(m_posit[i].CompProduct4 (localScale));
	}
	dgVector den (dgFloat32(1.0f / m_massesCount));
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
	return m_massesCount;
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
	return m_indexMap;
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

	// calculate partical accelerations
	CalculateAcceleration (timestep, body);

	const dgMatrix& matrix = body->GetCollision()->GetGlobalMatrix();
	
	dgVector damp(dgFloat32(1.0f));
	if (body->m_linearDampOn) {
		const dgFloat32 tau = dgFloat32(1.0f) / (dgFloat32(60.0f) * timestep);
		damp = dgVector(dgPow(dgFloat32(1.0f) - body->m_dampCoef.m_w, tau));
	}

	// rigid body physyx dynamic state
	dgVector timeV (timestep);
	dgVector den (dgFloat32(1.0f / m_massesCount));
	
	dgVector xxSum(dgFloat32(0.0f));
	dgVector xySum(dgFloat32(0.0f));
	dgVector xxSum2(dgFloat32(0.0f));
	dgVector comVeloc(dgFloat32(0.0f));

	dgVector angularMomentum(0.0f);
	dgVector origin(matrix.TransformVector(m_boxOrigin));
	dgVector unitMass (body->m_mass.m_w * den.GetScalar());
//	dgVector unitInvMass (body->m_invMass.m_w * m_massesCount);
	
	for (dgInt32 i = 0; i < m_massesCount; i++) {
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
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		m_posit[i] -= com.Scale4(m_unitMassScaler[i]);
	}
}


void dgCollisionDeformableMesh::CalculateAcceleration(dgFloat32 timestep, const dgDynamicBody* const body)
{
	dgVector timeV(timestep);
	dgFloat32* const spring_A01 = dgAlloca(dgFloat32, m_linksCount);
	dgFloat32* const spring_B01 = dgAlloca(dgFloat32, m_linksCount);
	dgVector* const linkDisplacement = dgAlloca(dgVector, m_linksCount);

	//dgVector den(dgFloat32(1.0f / m_massesCount));
	//dgVector unitforce(body->m_externalForce.CompProduct4(den));
	dgVector unitAccel(body->m_externalForce.CompProduct4(body->m_invMass.m_w));
	for (dgInt32 i = 0; i < m_massesCount; i++) {
		m_accel[i] = unitAccel;
	}

	// K is in [sec^2] a spring constant acceleration, not a spring force acceleration. 
	// for now make a share value for all springs. later this is a per material feature.
	dgFloat32 fKs = dgFloat32(100.0f);
	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgInt32 j0 = m_linkList[i].m_v0;
		const dgInt32 j1 = m_linkList[i].m_v1;

		dgVector deltaPosition(m_posit[j1] - m_posit[j0]);
		dgVector length2(deltaPosition.DotProduct4(deltaPosition));
		dgVector mask(length2 > m_smallestLenght2);
		length2 = (length2 & mask) | length2.AndNot(mask);
		dgFloat32 length = length2.Sqrt().GetScalar();
		dgFloat32 lenghtRatio = m_restlength[i] / length;
		spring_A01[i] = -fKs * (dgFloat32(1.0f) - lenghtRatio);
		spring_B01[i] = -fKs * lenghtRatio / length2.GetScalar();
		linkDisplacement[i] = timeV.CompProduct4(m_veloc[j0] - m_veloc[j1]);
	}

	for (dgInt32 i = 0; i < m_linksCount; i++) {
		const dgInt32 j0 = m_linkList[i].m_v0;
		const dgInt32 j1 = m_linkList[i].m_v1;
		m_accel[j0] += linkDisplacement[j0].Scale4(spring_A01[i]);
		m_accel[j1] -= linkDisplacement[j1].Scale4(spring_A01[i]);
	}
}
