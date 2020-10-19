/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndShapeConvexHull.h"
//#include "dgPhysicsStdafx.h"
//#include "dgBody.h"
//#include "dgContact.h"
//#include "dgMeshEffect.h"
//#include "ndShapeConvexHull.h"


//#define D_CONVEX_VERTEX_BOX_CELL_SIZE	(1<<3)
//#define D_CONVEX_VERTEX_SPLITE_SIZE	(D_CONVEX_VERTEX_BOX_CELL_SIZE * 3)
#define D_CONVEX_VERTEX_SPLITE_SIZE	48


D_MSV_NEWTON_ALIGN_32
class ndShapeConvexHull::dgSOAVectorArray
{
	public:
	dVector m_x[D_CONVEX_VERTEX_SPLITE_SIZE/4];
	dVector m_y[D_CONVEX_VERTEX_SPLITE_SIZE/4];
	dVector m_z[D_CONVEX_VERTEX_SPLITE_SIZE/4];
	dVector m_index[D_CONVEX_VERTEX_SPLITE_SIZE/4];
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndShapeConvexHull::ndConvexBox
{
	public:
	dVector m_box[2];
	dInt32 m_vertexStart;
	dInt32 m_vertexCount;
	dInt32 m_leftBox;
	dInt32 m_rightBox;
} D_GCC_NEWTON_ALIGN_32;

#if 0
ndShapeConvexHull::ndShapeConvexHull(dgMemoryAllocator* const allocator, dUnsigned32 signature)
	:dgCollisionConvex(allocator, signature, m_convexHullCollision)
	,m_supportTree(nullptr)
	,m_faceArray(nullptr)
	,m_soaVertexArray(nullptr)
	,m_vertexToEdgeMapping(nullptr)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_edgeCount = 0;
	m_vertexCount = 0;
	m_vertex = nullptr;
	m_simplex = nullptr;
	m_rtti |= dgCollisionConvexHull_RTTI;
}

ndShapeConvexHull::ndShapeConvexHull(dgWorld* const world, dgDeserialize callback, void* const userData, dInt32 revisionNumber)
	:dgCollisionConvex (world, callback, userData, revisionNumber)
	,m_supportTree(nullptr)
	,m_faceArray(nullptr)
	,m_soaVertexArray(nullptr)
	,m_vertexToEdgeMapping(nullptr)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_rtti |= dgCollisionConvexHull_RTTI;

	dInt32 edgeCount;
	dInt32 vertexCount;
	callback(userData, &vertexCount, sizeof(dInt32));
	callback(userData, &edgeCount, sizeof(dInt32));
	callback(userData, &m_faceCount, sizeof(dInt32));
	callback(userData, &m_supportTreeCount, sizeof(dInt32));

	m_edgeCount = dUnsigned16(edgeCount);
	m_vertexCount = dUnsigned16 (vertexCount);
	
	m_vertex = (dVector*) m_allocator->Malloc (dInt32 (m_vertexCount * sizeof (dVector)));
	m_simplex = (ndConvexSimplexEdge*) m_allocator->Malloc (dInt32 (m_edgeCount * sizeof (ndConvexSimplexEdge)));
	m_faceArray = (ndConvexSimplexEdge **) m_allocator->Malloc(dInt32 (m_faceCount * sizeof(ndConvexSimplexEdge *)));
	m_vertexToEdgeMapping = (const ndConvexSimplexEdge **) m_allocator->Malloc(dInt32 (m_vertexCount * sizeof(ndConvexSimplexEdge *)));

	callback(userData, m_vertex, m_vertexCount * sizeof(dVector));

	if (m_supportTreeCount) {
		m_supportTree = (dgConvexBox *) m_allocator->Malloc(dInt32 (m_supportTreeCount * sizeof(dgConvexBox)));
		callback (userData, m_supportTree, m_supportTreeCount * sizeof(dgConvexBox));
	}

	for (dInt32 i = 0; i < m_edgeCount; i ++) {
		dInt32 serialization[4];
		callback (userData, serialization, sizeof (serialization));

		m_simplex[i].m_vertex = serialization[0];
		m_simplex[i].m_twin = m_simplex + serialization[1];
		m_simplex[i].m_next = m_simplex + serialization[2];
		m_simplex[i].m_prev = m_simplex + serialization[3];
	}

	for (dInt32 i = 0; i < m_faceCount; i ++) {
		dInt32 faceOffset;
		callback (userData, &faceOffset, sizeof (dInt32));
		m_faceArray[i] = m_simplex + faceOffset; 
	}

	for (dInt32 i = 0; i < m_vertexCount; i ++) {
		dInt32 faceOffset;
		callback (userData, &faceOffset, sizeof (dInt32));
		m_vertexToEdgeMapping[i] = m_simplex + faceOffset; 
	}

	if (vertexCount <= D_CONVEX_VERTEX_SPLITE_SIZE) {
		CreateSOAdata();
	}

	SetVolumeAndCG ();
}

void ndShapeConvexHull::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);

	dInt32 edgeCount = m_edgeCount;
	dInt32 vertexCount = m_vertexCount;
	callback(userData, &vertexCount, sizeof(dInt32));
	callback(userData, &edgeCount, sizeof(dInt32));
	callback(userData, &m_faceCount, sizeof(dInt32));
	callback(userData, &m_supportTreeCount, sizeof(dInt32));

	callback(userData, m_vertex, m_vertexCount * sizeof(dVector));

	if (m_supportTreeCount) {
		callback(userData, m_supportTree, m_supportTreeCount * sizeof(dgConvexBox));
	}

	for (dInt32 i = 0; i < m_edgeCount; i++) {
		dInt32 serialization[4];
		serialization[0] = m_simplex[i].m_vertex;
		serialization[1] = dInt32(m_simplex[i].m_twin - m_simplex);
		serialization[2] = dInt32(m_simplex[i].m_next - m_simplex);
		serialization[3] = dInt32(m_simplex[i].m_prev - m_simplex);
		callback(userData, serialization, sizeof(serialization));
	}

	for (dInt32 i = 0; i < m_faceCount; i++) {
		dInt32 faceOffset;
		faceOffset = dInt32(m_faceArray[i] - m_simplex);
		callback(userData, &faceOffset, sizeof(dInt32));
	}

	for (dInt32 i = 0; i < m_vertexCount; i++) {
		dInt32 faceOffset;
		faceOffset = dInt32(m_vertexToEdgeMapping[i] - m_simplex);
		callback(userData, &faceOffset, sizeof(dInt32));
	}
}

void ndShapeConvexHull::BuildHull (dInt32 count, dInt32 strideInBytes, dFloat32 tolerance, const dFloat32* const vertexArray)
{
	Create (count, strideInBytes, vertexArray, tolerance);
}

void ndShapeConvexHull::MassProperties ()
{
	dFloat32 volume = dgCollisionConvex::CalculateMassProperties(dGetIdentityMatrix(), m_inertia, m_crossInertia, m_centerOfMass);
	if (volume < dFloat32 (1.0e-6f)) {
		volume = dFloat32 (1.0e-6f);
	}
	dFloat32 invVolume = dFloat32(1.0f) / volume;
	m_inertia = m_inertia.Scale(invVolume);
	m_crossInertia = m_crossInertia.Scale(invVolume);
	m_centerOfMass = m_centerOfMass.Scale(invVolume);
	m_centerOfMass.m_w = volume;


	dMatrix inertia(dGetIdentityMatrix());
	inertia[0][0] = m_inertia[0];
	inertia[1][1] = m_inertia[1];
	inertia[2][2] = m_inertia[2];
	inertia[0][1] = m_crossInertia[2];
	inertia[1][0] = m_crossInertia[2];
	inertia[0][2] = m_crossInertia[1];
	inertia[2][0] = m_crossInertia[1];
	inertia[1][2] = m_crossInertia[0];
	inertia[2][1] = m_crossInertia[0];

	dVector origin(m_centerOfMass);
	dFloat32 originMag2 = origin.DotProduct(origin & dVector::m_triplexMask).GetScalar();

	dMatrix Covariance(origin, origin);
	dMatrix parallel(dGetIdentityMatrix());
	for (dInt32 i = 0; i < 3; i++) {
		parallel[i][i] = originMag2;
		inertia[i] -= (parallel[i] - Covariance[i]);
		dAssert(inertia[i][i] > dFloat32(0.0f));
	}

	m_inertia[0] = inertia[0][0];
	m_inertia[1] = inertia[1][1];
	m_inertia[2] = inertia[2][2];
	m_crossInertia[0] = inertia[2][1];
	m_crossInertia[1] = inertia[2][0];
	m_crossInertia[2] = inertia[1][0];
}

dInt32 ndShapeConvexHull::GetFaceIndices (dInt32 index, dInt32* const indices) const
{
	dInt32 count = 0;
	const ndConvexSimplexEdge* face = m_faceArray[index];
	do {
		indices [count] = face->m_vertex;
		count ++;
		face = face->m_next;
	} while (face != m_faceArray[index]);

	return count;
}


bool ndShapeConvexHull::CheckConvex (dPolyhedra& polyhedra1, const dBigVector* hullVertexArray) const
{
	dPolyhedra polyhedra(polyhedra1);

	dPolyhedra::Iterator iter (polyhedra);
	dBigVector center (dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f), dFloat32 (0.0f));

	dInt32 count = 0;
	dInt32 mark = polyhedra.IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dEdge* const edge = &(*iter);
		if (edge->m_mark < mark) {
			count ++;
			center += hullVertexArray[edge->m_incidentVertex];
			dEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}
	center = center.Scale (dFloat64 (1.0f) / dFloat64 (count));

	for (iter.Begin(); iter; iter ++) {
		dEdge* const edge = &(*iter);
		dBigVector normal0 (FaceNormal (edge, hullVertexArray));
		dBigVector normal1 (FaceNormal (edge->m_twin, hullVertexArray));

		dBigPlane plane0 (normal0, - normal0.DotProduct(hullVertexArray[edge->m_incidentVertex]).GetScalar());
		dBigPlane plane1 (normal1, - normal1.DotProduct(hullVertexArray[edge->m_twin->m_incidentVertex]).GetScalar());
		dFloat64 test0 = plane0.Evalue(center);
		if (test0 > dFloat64 (1.0e-3f)) {
			return false;
		}
		dFloat64 test1 = plane1.Evalue(center);
//		if (test1 > dFloat64 (0.0f)) {
		if (test1 > dFloat64 (1.0e-3f)) {
			return false;
		}
	}
	return true;
}


void ndShapeConvexHull::DebugCollision (const dMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dTriplex vertex[256];
	for (dInt32 i = 0; i < m_faceCount; i ++) {
		ndConvexSimplexEdge* const face = m_faceArray[i];
		ndConvexSimplexEdge* ptr = face;
		dInt32 count = 0;
		do {
			vertex[count].m_x = m_vertex[ptr->m_vertex].m_x;
			vertex[count].m_y = m_vertex[ptr->m_vertex].m_y;
			vertex[count].m_z = m_vertex[ptr->m_vertex].m_z;
			count ++;
			dAssert (count < sizeof (vertex)/ sizeof (vertex[0]));
			ptr = ptr->m_next;
		} while (ptr != face);
		matrix.TransformTriplex (&vertex[0].m_x, sizeof (dTriplex), &vertex[0].m_x, sizeof (dTriplex), count);
		callback (userData, count, &vertex[0].m_x, 0);
	}
}

void ndShapeConvexHull::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_convexHull.m_vertexCount = m_vertexCount;
	info->m_convexHull.m_strideInBytes = sizeof (dVector);
	info->m_convexHull.m_faceCount = m_faceCount;
	info->m_convexHull.m_vertex = &m_vertex[0];
}


dVector ndShapeConvexHull::SupportVertex(const dVector& dir, dInt32* const vertexIndex) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dInt32 index = -1;
	dVector maxProj(dFloat32(-1.0e20f));
	if (m_vertexCount > D_CONVEX_VERTEX_SPLITE_SIZE) {
		dFloat32 distPool[32];
		const dgConvexBox* stackPool[32];

		dInt32 ix = (dir[0] > dFloat64(0.0f)) ? 1 : 0;
		dInt32 iy = (dir[1] > dFloat64(0.0f)) ? 1 : 0;
		dInt32 iz = (dir[2] > dFloat64(0.0f)) ? 1 : 0;

		const dgConvexBox& leftBox = m_supportTree[m_supportTree[0].m_leftBox];
		const dgConvexBox& rightBox = m_supportTree[m_supportTree[0].m_rightBox];

		dVector leftP(leftBox.m_box[ix][0], leftBox.m_box[iy][1], leftBox.m_box[iz][2], dFloat32(0.0f));
		dVector rightP(rightBox.m_box[ix][0], rightBox.m_box[iy][1], rightBox.m_box[iz][2], dFloat32(0.0f));

		dFloat32 leftDist = leftP.DotProduct(dir).GetScalar();
		dFloat32 rightDist = rightP.DotProduct(dir).GetScalar();
		if (rightDist >= leftDist) {
			distPool[0] = leftDist;
			stackPool[0] = &leftBox;

			distPool[1] = rightDist;
			stackPool[1] = &rightBox;
		} else {
			distPool[0] = rightDist;
			stackPool[0] = &rightBox;

			distPool[1] = leftDist;
			stackPool[1] = &leftBox;
		}

		dInt32 stack = 2;

		while (stack) {
			stack--;
			dFloat32 dist = distPool[stack];
			if (dist > maxProj.m_x) {
				const dgConvexBox& box = *stackPool[stack];

				if (box.m_leftBox > 0) {
					dAssert(box.m_rightBox > 0);
					const dgConvexBox& leftBox1 = m_supportTree[box.m_leftBox];
					const dgConvexBox& rightBox1 = m_supportTree[box.m_rightBox];

					dVector leftBoxP(leftBox1.m_box[ix][0], leftBox1.m_box[iy][1], leftBox1.m_box[iz][2], dFloat32(0.0f));
					dVector rightBoxP(rightBox1.m_box[ix][0], rightBox1.m_box[iy][1], rightBox1.m_box[iz][2], dFloat32(0.0f));

					dFloat32 leftBoxDist = leftBoxP.DotProduct(dir).GetScalar();
					dFloat32 rightBoxDist = rightBoxP.DotProduct(dir).GetScalar();
					if (rightBoxDist >= leftBoxDist) {
						distPool[stack] = leftBoxDist;
						stackPool[stack] = &leftBox1;
						stack++;
						dAssert(stack < sizeof (distPool) / sizeof (distPool[0]));

						distPool[stack] = rightBoxDist;
						stackPool[stack] = &rightBox1;
						stack++;
						dAssert(stack < sizeof (distPool) / sizeof (distPool[0]));

					} else {
						distPool[stack] = rightBoxDist;
						stackPool[stack] = &rightBox1;
						stack++;
						dAssert(stack < sizeof (distPool) / sizeof (distPool[0]));

						distPool[stack] = leftBoxDist;
						stackPool[stack] = &leftBox1;
						stack++;
						dAssert(stack < sizeof (distPool) / sizeof (distPool[0]));
					}
				} else {
					for (dInt32 i = 0; i < box.m_vertexCount; i++) {
						const dVector& p = m_vertex[box.m_vertexStart + i];
						dAssert(p.m_x >= box.m_box[0].m_x);
						dAssert(p.m_x <= box.m_box[1].m_x);
						dAssert(p.m_y >= box.m_box[0].m_y);
						dAssert(p.m_y <= box.m_box[1].m_y);
						dAssert(p.m_z >= box.m_box[0].m_z);
						dAssert(p.m_z <= box.m_box[1].m_z);
						dVector projectionDist(p.DotProduct(dir));
						dVector mask(projectionDist > maxProj);
						dInt32 intMask = *((dInt32*)&mask.m_x);
						index = ((box.m_vertexStart + i) & intMask) | (index & ~intMask);
						maxProj = maxProj.GetMax(projectionDist);
					}
				}
			}
		}
	} else {
#if 0
		for (dInt32 i = 0; i < m_vertexCount; i++) {
			const dVector& p = m_vertex[i];
			dVector dist(p.DotProduct(dir));
			dVector mask(dist > maxProj);
			dInt32 intMask = *((dInt32*)&mask.m_x);
			index = (i & intMask) | (index & ~intMask);
			maxProj = maxProj.GetMax(dist);
		}
#else
		const dVector x(dir.m_x);
		const dVector y(dir.m_y);
		const dVector z(dir.m_z);
		dVector support (dVector::m_negOne);
		for (dInt32 i = 0; i < m_soaVertexCount; i+=2) {
			dVector dot (m_soaVertexArray->m_x[i] * x + 
						  m_soaVertexArray->m_y[i] * y + 
						  m_soaVertexArray->m_z[i] * z);
			support = support.Select (m_soaVertexArray->m_index[i], dot > maxProj);
			maxProj = maxProj.GetMax(dot);

			dot = m_soaVertexArray->m_x[i + 1] * x +
				  m_soaVertexArray->m_y[i + 1] * y +
				  m_soaVertexArray->m_z[i + 1] * z;
			support = support.Select(m_soaVertexArray->m_index[i + 1], dot > maxProj);
			maxProj = maxProj.GetMax(dot);
		}
		 
		dVector dot (maxProj.ShiftRight().ShiftRight());
		dVector support1 (support.ShiftRight().ShiftRight());
		support = support.Select(support1, dot > maxProj);
		maxProj = maxProj.GetMax(dot);

		dot = dVector (maxProj.ShiftRight());
		support1 = dVector (support.ShiftRight());
		support = support.Select(support1, dot > maxProj);

		index = dInt32 (support.GetScalar()); 
#endif
	}

	if (vertexIndex) {
		*vertexIndex = index;
	}
	dAssert(index != -1);
	return m_vertex[index];
}

#endif


//ndShapeConvexHull::ndShapeConvexHull(dInt32 count, dInt32 strideInBytes, dFloat32 tolerance, const dFloat32* const vertexArray)
ndShapeConvexHull::ndShapeConvexHull (dInt32 count, dInt32 strideInBytes, dFloat32 tolerance, const dFloat32* const vertexArray)
	:ndShapeConvex(m_convexHull)
	,m_supportTree(nullptr)
	,m_faceArray(nullptr)
	,m_soaVertexArray(nullptr)
	,m_vertexToEdgeMapping(nullptr)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_edgeCount = 0;
	m_vertexCount = 0;
	m_vertex = nullptr;
	m_simplex = nullptr;
	Create(count, strideInBytes, vertexArray, tolerance);
}

ndShapeConvexHull::~ndShapeConvexHull()
{
	if (m_vertexToEdgeMapping) 
	{
		dMemory::Free(m_vertexToEdgeMapping);
	}
	
	if (m_faceArray) 
	{
		dMemory::Free(m_faceArray);
	}
	
	if (m_supportTree) 
	{
		dMemory::Free(m_supportTree);
	}
	
	if (m_soaVertexArray) 
	{
		dMemory::Free(m_soaVertexArray);
	}
}

bool ndShapeConvexHull::Create(dInt32 count, dInt32 strideInBytes, const dFloat32* const vertexArray, dFloat32 tolerance)
{
	dInt32 stride = strideInBytes / sizeof(dFloat32);
	dStack<dBigVector> buffer(2 * count);
	for (dInt32 i = 0; i < count; i++) 
	{
		buffer[i] = dVector(vertexArray[i * stride + 0], vertexArray[i * stride + 1], vertexArray[i * stride + 2], dFloat32(0.0f));
	}

	dConvexHull3d* convexHull = new dConvexHull3d(&buffer[0].m_x, sizeof (dBigVector), count, tolerance);
	if (!convexHull->GetCount()) 
	{
		dAssert(0);
		//// this is a degenerated hull hull to add some thickness and for a thick plane
		//delete convexHull;
		//
		//dStack<dVector> tmp(3 * count);
		//for (dInt32 i = 0; i < count; i++) 
		//{
		//	tmp[i][0] = dFloat32(buffer[i].m_x);
		//	tmp[i][1] = dFloat32(buffer[i].m_y);
		//	tmp[i][2] = dFloat32(buffer[i].m_z);
		//	tmp[i][2] = dFloat32(0.0f);
		//}
		//
		//dObb sphere;
		//sphere.SetDimensions(&tmp[0][0], sizeof(dVector), count);
		//
		//dInt32 index = 0;
		//dFloat32 size = dFloat32(1.0e10f);
		//for (dInt32 i = 0; i < 3; i++) 
		//{
		//	if (sphere.m_size[i] < size) 
		//	{
		//		index = i;
		//		size = sphere.m_size[i];
		//	}
		//}
		//dVector normal(dFloat32(0.0f));
		//normal[index] = dFloat32(1.0f);
		//dVector step = sphere.RotateVector(normal.Scale(dFloat32(0.05f)));
		//for (dInt32 i = 0; i < count; i++) 
		//{
		//	dVector p1(tmp[i] + step);
		//	dVector p2(tmp[i] - step);
		//
		//	buffer[i * 3 + 0] = p1.m_x;
		//	buffer[i * 3 + 1] = p1.m_y;
		//	buffer[i * 3 + 2] = p1.m_z;
		//	buffer[(i + count) * 3 + 0] = p2.m_x;
		//	buffer[(i + count) * 3 + 1] = p2.m_y;
		//	buffer[(i + count) * 3 + 2] = p2.m_z;
		//}
		//count *= 2;
		//convexHull = new dConvexHull3d(&buffer[0].m_x, sizeof(dBigVector), count, tolerance);
		//if (!convexHull->GetCount()) 
		//{
		//	delete convexHull;
		//	return false;
		//}
	}

	// check for degenerated faces
	for (bool success = false; !success; ) 
	{
		success = true;
		const dBigVector* const hullVertexArray = convexHull->GetVertexPool();

		dStack<dInt8> mask(convexHull->GetVertexCount());
		memset(&mask[0], 1, mask.GetSizeInBytes());
		for (dConvexHull3d::dListNode* node = convexHull->GetFirst(); node; node = node->GetNext()) 
		{
			dConvexHull3dFace& face = node->GetInfo();
			const dBigVector& p0 = hullVertexArray[face.m_index[0]];
			const dBigVector& p1 = hullVertexArray[face.m_index[1]];
			const dBigVector& p2 = hullVertexArray[face.m_index[2]];
			dAssert(p0.m_w == p1.m_w);
			dAssert(p0.m_w == p2.m_w);
			dBigVector p1p0(p1 - p0);
			dBigVector p2p0(p2 - p0);
			dBigVector normal(p2p0.CrossProduct(p1p0));
			dFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < dFloat64(1.0e-6f * 1.0e-6f)) 
			{
				success = false;
				dInt32 index = -1;
				dBigVector p2p1(p2 - p1);
				dFloat64 dist10 = p1p0.DotProduct(p1p0).GetScalar();
				dFloat64 dist20 = p2p0.DotProduct(p2p0).GetScalar();
				dFloat64 dist21 = p2p1.DotProduct(p2p1).GetScalar();
				if ((dist10 >= dist20) && (dist10 >= dist21)) 
				{
					index = 2;
				}
				else if ((dist20 >= dist10) && (dist20 >= dist21)) 
				{
					index = 1;
				}
				else if ((dist21 >= dist10) && (dist21 >= dist20)) 
				{
					index = 0;
				}
				dAssert(index != -1);
				mask[face.m_index[index]] = 0;
			}
		}
		if (!success) 
		{
			dInt32 count1 = 0;
			dInt32 vertexCount = convexHull->GetVertexCount();
			for (dInt32 i = 0; i < vertexCount; i++) 
			{
				if (mask[i]) 
				{
					buffer[count1] = hullVertexArray[i] & dBigVector::m_triplexMask;
					count1++;
				}
			}
			delete convexHull;
			convexHull = new dConvexHull3d(&buffer[0].m_x, sizeof(dBigVector), count1, tolerance);
		}
	}

	dAssert(convexHull);
	dInt32 vertexCount = convexHull->GetVertexCount();
	if (vertexCount < 4) 
	{
		delete convexHull;
		return false;
	}

	const dBigVector* const hullVertexArray = convexHull->GetVertexPool();
	dPolyhedra polyhedra;
	polyhedra.BeginFace();
	for (dConvexHull3d::dListNode* node = convexHull->GetFirst(); node; node = node->GetNext()) 
	{
		dConvexHull3dFace& face = node->GetInfo();
		polyhedra.AddFace(face.m_index[0], face.m_index[1], face.m_index[2]);
	}
	polyhedra.EndFace();

	if (vertexCount > 4) 
	{
		while (RemoveCoplanarEdge(polyhedra, hullVertexArray));
	}

	dStack<dInt32> vertexMap(vertexCount);
	memset(&vertexMap[0], -1, vertexCount * sizeof(dInt32));

	dInt32 mark = polyhedra.IncLRU();
	dPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) 
		{
			if (vertexMap[edge->m_incidentVertex] == -1) 
			{
				vertexMap[edge->m_incidentVertex] = m_vertexCount;
				m_vertexCount++;
			}
			dEdge* ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ptr->m_userData = m_edgeCount;
				m_edgeCount++;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}

	m_vertex = (dVector*)dMemory::Malloc(dInt32(m_vertexCount * sizeof(dVector)));
	m_simplex = (ndConvexSimplexEdge*)dMemory::Malloc(dInt32(m_edgeCount * sizeof(ndConvexSimplexEdge)));
	m_vertexToEdgeMapping = (const ndConvexSimplexEdge**)dMemory::Malloc(dInt32(m_vertexCount * sizeof(ndConvexSimplexEdge*)));

	for (dInt32 i = 0; i < vertexCount; i++) 
	{
		if (vertexMap[i] != -1) 
		{
			m_vertex[vertexMap[i]] = hullVertexArray[i];
			m_vertex[vertexMap[i]].m_w = dFloat32(0.0f);
		}
	}
	delete convexHull;

	vertexCount = m_vertexCount;
	mark = polyhedra.IncLRU();;
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) 
		{
			dEdge *ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				ndConvexSimplexEdge* const simplexPtr = &m_simplex[ptr->m_userData];
				simplexPtr->m_vertex = vertexMap[ptr->m_incidentVertex];
				simplexPtr->m_next = &m_simplex[ptr->m_next->m_userData];
				simplexPtr->m_prev = &m_simplex[ptr->m_prev->m_userData];
				simplexPtr->m_twin = &m_simplex[ptr->m_twin->m_userData];

				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}

	m_faceCount = 0;
	dStack<char> faceMarks(m_edgeCount);
	memset(&faceMarks[0], 0, m_edgeCount * sizeof(dInt8));

	dStack<ndConvexSimplexEdge*> faceArray(m_edgeCount);

	for (dInt32 i = 0; i < m_edgeCount; i++) 
	{
		ndConvexSimplexEdge* const face = &m_simplex[i];
		if (!faceMarks[i]) 
		{
			ndConvexSimplexEdge* ptr = face;
			do 
			{
				dAssert((ptr - m_simplex) >= 0);
				faceMarks[dInt32(ptr - m_simplex)] = '1';
				ptr = ptr->m_next;
			} while (ptr != face);

			faceArray[m_faceCount] = face;
			m_faceCount++;
		}
	}
	m_faceArray = (ndConvexSimplexEdge **)dMemory::Malloc(dInt32(m_faceCount * sizeof(ndConvexSimplexEdge *)));
	memcpy(m_faceArray, &faceArray[0], m_faceCount * sizeof(ndConvexSimplexEdge *));

	if (vertexCount > D_CONVEX_VERTEX_SPLITE_SIZE) 
	{
		// create a face structure for support vertex
		dStack<ndConvexBox> boxTree(vertexCount);
		dTree<dVector, dInt32> sortTree;
		dStack<dTree<dVector, dInt32>::dTreeNode*> vertexNodeList(vertexCount);
		
		dVector boxP0(dFloat32(1.0e15f));
		dVector boxP1(-dFloat32(1.0e15f));
		for (dInt32 i = 0; i < vertexCount; i++) 
		{
			const dVector& p = m_vertex[i];
			vertexNodeList[i] = sortTree.Insert(p, i);
			boxP0 = boxP0.GetMin(p);
			boxP1 = boxP1.GetMax(p);
		}
		
		boxTree[0].m_box[0] = boxP0 & dVector::m_triplexMask;
		boxTree[0].m_box[1] = boxP1 & dVector::m_triplexMask;
		boxTree[0].m_leftBox = -1;
		boxTree[0].m_rightBox = -1;
		boxTree[0].m_vertexStart = 0;
		boxTree[0].m_vertexCount = vertexCount;
		dInt32 boxCount = 1;
		
		dInt32 stack = 1;
		dInt32 stackBoxPool[64];
		stackBoxPool[0] = 0;
		
		dAssert(0);
		//while (stack) 
		//{
		//	stack--;
		//	dInt32 boxIndex = stackBoxPool[stack];
		//	ndConvexBox& box = boxTree[boxIndex];
		//	if (box.m_vertexCount > D_CONVEX_VERTEX_BOX_CELL_SIZE) 
		//	{
		//		dVector median(dVector::m_zero);
		//		dVector varian(dVector::m_zero);
		//		for (dInt32 i = 0; i < box.m_vertexCount; i++) 
		//		{
		//			dVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
		//			boxP0 = boxP0.GetMin(p);
		//			boxP1 = boxP1.GetMax(p);
		//			median += p;
		//			varian += p * p;
		//		}
		//
		//		dInt32 index = 0;
		//		dFloat64 maxVarian = dFloat64(-1.0e10f);
		//		varian = varian.Scale(dFloat32(box.m_vertexCount)) - median * median;
		//		for (dInt32 i = 0; i < 3; i++) 
		//		{
		//			if (varian[i] > maxVarian) 
		//			{
		//				index = i;
		//				maxVarian = varian[i];
		//			}
		//		}
		//		dVector center = median.Scale(dFloat32(1.0f) / dFloat32(box.m_vertexCount));
		//		dFloat32 test = center[index];
		//
		//		dInt32 i0 = 0;
		//		dInt32 i1 = box.m_vertexCount - 1;
		//		do 
		//		{
		//			for (; i0 <= i1; i0++) 
		//			{
		//				dFloat32 val = vertexNodeList[box.m_vertexStart + i0]->GetInfo()[index];
		//				if (val > test) 
		//				{
		//					break;
		//				}
		//			}
		//
		//			for (; i1 >= i0; i1--) 
		//			{
		//				dFloat32 val = vertexNodeList[box.m_vertexStart + i1]->GetInfo()[index];
		//				if (val < test) 
		//				{
		//					break;
		//				}
		//			}
		//
		//			if (i0 < i1) 
		//			{
		//				dSwap(vertexNodeList[box.m_vertexStart + i0], vertexNodeList[box.m_vertexStart + i1]);
		//				i0++;
		//				i1--;
		//			}
		//		} while (i0 <= i1);
		//
		//		if (i0 == 0) 
		//		{
		//			i0 = box.m_vertexCount / 2;
		//		}
		//		if (i0 >= (box.m_vertexCount - 1)) 
		//		{
		//			i0 = box.m_vertexCount / 2;
		//		}
		//
		//		{
		//			// insert right branch AABB
		//			dVector rightBoxP0(dFloat32(1.0e15f));
		//			dVector rightBoxP1(-dFloat32(1.0e15f));
		//			for (dInt32 i = i0; i < box.m_vertexCount; i++) 
		//			{
		//				const dVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
		//				rightBoxP0 = rightBoxP0.GetMin(p);
		//				rightBoxP1 = rightBoxP1.GetMax(p);
		//			}
		//
		//			box.m_rightBox = boxCount;
		//			boxTree[boxCount].m_box[0] = rightBoxP0 & dVector::m_triplexMask;
		//			boxTree[boxCount].m_box[1] = rightBoxP1 & dVector::m_triplexMask;
		//			boxTree[boxCount].m_leftBox = -1;
		//			boxTree[boxCount].m_rightBox = -1;
		//			boxTree[boxCount].m_vertexStart = box.m_vertexStart + i0;
		//			boxTree[boxCount].m_vertexCount = box.m_vertexCount - i0;
		//			stackBoxPool[stack] = boxCount;
		//			stack++;
		//			boxCount++;
		//		}
		//
		//		{
		//			// insert left branch AABB
		//			dVector leftBoxP0(dFloat32(1.0e15f));
		//			dVector leftBoxP1(-dFloat32(1.0e15f));
		//			for (dInt32 i = 0; i < i0; i++) {
		//				const dVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
		//				leftBoxP0 = leftBoxP0.GetMin(p);
		//				leftBoxP1 = leftBoxP1.GetMax(p);
		//			}
		//
		//			box.m_leftBox = boxCount;
		//			boxTree[boxCount].m_box[0] = leftBoxP0 & dVector::m_triplexMask;;
		//			boxTree[boxCount].m_box[1] = leftBoxP1 & dVector::m_triplexMask;;
		//			boxTree[boxCount].m_leftBox = -1;
		//			boxTree[boxCount].m_rightBox = -1;
		//			boxTree[boxCount].m_vertexStart = box.m_vertexStart;
		//			boxTree[boxCount].m_vertexCount = i0;
		//			stackBoxPool[stack] = boxCount;
		//			stack++;
		//			boxCount++;
		//		}
		//	}
		//}
		//
		//for (dInt32 i = 0; i < m_vertexCount; i++) 
		//{
		//	m_vertex[i] = vertexNodeList[i]->GetInfo();
		//	vertexNodeList[i]->GetInfo().m_w = dFloat32(i);
		//}
		//
		//m_supportTreeCount = boxCount;
		//m_supportTree = (ndConvexBox*)dMemory::Malloc(dInt32(boxCount * sizeof(ndConvexBox)));
		//memcpy(m_supportTree, &boxTree[0], boxCount * sizeof(ndConvexBox));
		//
		//for (dInt32 i = 0; i < m_edgeCount; i++) 
		//{
		//	ndConvexSimplexEdge* const ptr = &m_simplex[i];
		//	dTree<dVector, dInt32>::dTreeNode* const node = sortTree.Find(ptr->m_vertex);
		//	dInt32 index = dInt32(node->GetInfo().m_w);
		//	ptr->m_vertex = dInt16(index);
		//}
	}
	else 
	{
		CreateSOAdata();
	}

	for (dInt32 i = 0; i < m_edgeCount; i++) 
	{
		ndConvexSimplexEdge* const edge = &m_simplex[i];
		m_vertexToEdgeMapping[edge->m_vertex] = edge;
	}

	SetVolumeAndCG();

	return true;
}

dBigVector ndShapeConvexHull::FaceNormal(const dEdge *face, const dBigVector* const pool) const
{
	const dEdge* edge = face;
	dBigVector p0(pool[edge->m_incidentVertex]);
	edge = edge->m_next;

	dBigVector p1(pool[edge->m_incidentVertex]);
	dBigVector e1(p1 - p0);

	dBigVector normal(dBigVector::m_zero);
	for (edge = edge->m_next; edge != face; edge = edge->m_next) 
	{
		dBigVector p2(pool[edge->m_incidentVertex]);
		dBigVector e2(p2 - p0);
		dBigVector n1(e1.CrossProduct(e2));
#ifdef _DEBUG
		dAssert(n1.m_w == dFloat32(0.0f));
		dFloat64 mag = normal.DotProduct(n1).GetScalar();
		dAssert(mag >= -dFloat32(0.1f));
#endif
		normal += n1;
		e1 = e2;
	}

	dFloat64 den = sqrt(normal.DotProduct(normal).GetScalar()) + dFloat64(1.0e-24f);
	normal = normal.Scale(dFloat64(1.0f) / den);

#ifdef _DEBUG
	edge = face;
	dBigVector e0(pool[edge->m_incidentVertex] - pool[edge->m_prev->m_incidentVertex]);
	do 
	{
		dBigVector de1(pool[edge->m_next->m_incidentVertex] - pool[edge->m_incidentVertex]);
		dBigVector dn1(e0.CrossProduct(de1));
		dFloat64 x = normal.DotProduct(dn1).GetScalar();
		dAssert(x > -dFloat64(0.01f));
		e0 = de1;
		edge = edge->m_next;
	} while (edge != face);
#endif
	return normal;
}

bool ndShapeConvexHull::RemoveCoplanarEdge(dPolyhedra& polyhedra, const dBigVector* const hullVertexArray) const
{
	bool removeEdge = false;
	// remove coplanar edges
	dInt32 mark = polyhedra.IncLRU();
	dPolyhedra::Iterator iter(polyhedra);
	for (iter.Begin(); iter; ) 
	{
		dEdge* edge0 = &(*iter);
		iter++;

		if (edge0->m_incidentFace != -1) 
		{
			if (edge0->m_mark < mark) 
			{
				edge0->m_mark = mark;
				edge0->m_twin->m_mark = mark;
				dBigVector normal0(FaceNormal(edge0, &hullVertexArray[0]));
				dBigVector normal1(FaceNormal(edge0->m_twin, &hullVertexArray[0]));

				dFloat64 test = normal0.DotProduct(normal1).GetScalar();
				if (test > dFloat64(0.99995f)) 
				{
					if ((edge0->m_twin->m_next->m_twin->m_next != edge0) && (edge0->m_next->m_twin->m_next != edge0->m_twin)) 
					{
						#define DG_MAX_EDGE_ANGLE dFloat32 (1.0e-3f)
						if (edge0->m_twin == &(*iter)) 
						{
							if (iter) 
							{
								iter++;
							}
						}

						dBigVector e1(hullVertexArray[edge0->m_twin->m_next->m_next->m_incidentVertex] - hullVertexArray[edge0->m_incidentVertex]);
						dBigVector e0(hullVertexArray[edge0->m_incidentVertex] - hullVertexArray[edge0->m_prev->m_incidentVertex]);

						dAssert(e0.m_w == dFloat64(0.0f));
						dAssert(e1.m_w == dFloat64(0.0f));
						dAssert(e0.DotProduct(e0).GetScalar() >= dFloat64(0.0f));
						dAssert(e1.DotProduct(e1).GetScalar() >= dFloat64(0.0f));

						e0 = e0.Scale(dFloat64(1.0f) / sqrt(e0.DotProduct(e0).GetScalar()));
						e1 = e1.Scale(dFloat64(1.0f) / sqrt(e1.DotProduct(e1).GetScalar()));
						dBigVector n1(e0.CrossProduct(e1));

						dFloat64 projection = n1.DotProduct(normal0).GetScalar();
						if (projection >= DG_MAX_EDGE_ANGLE) 
						{
							dBigVector e11(hullVertexArray[edge0->m_next->m_next->m_incidentVertex] - hullVertexArray[edge0->m_twin->m_incidentVertex]);
							dBigVector e00(hullVertexArray[edge0->m_twin->m_incidentVertex] - hullVertexArray[edge0->m_twin->m_prev->m_incidentVertex]);
							dAssert(e00.m_w == dFloat64(0.0f));
							dAssert(e11.m_w == dFloat64(0.0f));
							dAssert(e00.DotProduct(e00).GetScalar() >= dFloat64(0.0f));
							dAssert(e11.DotProduct(e11).GetScalar() >= dFloat64(0.0f));
							e00 = e00.Scale(dFloat64(1.0f) / sqrt(e00.DotProduct(e00).GetScalar()));
							e11 = e11.Scale(dFloat64(1.0f) / sqrt(e11.DotProduct(e11).GetScalar()));

							dBigVector n11(e00.CrossProduct(e11));
							projection = n11.DotProduct(normal0).GetScalar();
							if (projection >= DG_MAX_EDGE_ANGLE) 
							{
								dAssert(&(*iter) != edge0);
								dAssert(&(*iter) != edge0->m_twin);
								polyhedra.DeleteEdge(edge0);
								removeEdge = true;
							}
						}
					}
					else 
					{
						dEdge* next = edge0->m_next;
						dEdge* prev = edge0->m_prev;
						polyhedra.DeleteEdge(edge0);
						for (edge0 = next; edge0->m_prev->m_twin == edge0; edge0 = next) 
						{
							next = edge0->m_next;
							polyhedra.DeleteEdge(edge0);
						}

						for (edge0 = prev; edge0->m_next->m_twin == edge0; edge0 = prev) 
						{
							prev = edge0->m_prev;
							polyhedra.DeleteEdge(edge0);
						}
						iter.Begin();
						removeEdge = true;
					}
				}
			}
		}
	}

	return removeEdge;
}

void ndShapeConvexHull::CreateSOAdata()
{
	m_soaVertexArray = (dgSOAVectorArray*)dMemory::Malloc(sizeof(dgSOAVectorArray));

	m_soaVertexCount = ((m_vertexCount + 7) & -8) / 8;
	dVector array[D_CONVEX_VERTEX_SPLITE_SIZE];
	for (dInt32 i = 0; i < m_vertexCount; i++) 
	{
		array[i] = m_vertex[i];
	}

	for (dInt32 i = m_vertexCount; i < D_CONVEX_VERTEX_SPLITE_SIZE; i++) 
	{
		array[i] = array[0];
	}

	dVector step(dFloat32(4.0f));
	dVector index(dFloat32(0.0f), dFloat32(1.0f), dFloat32(2.0f), dFloat32(3.0f));
	for (dInt32 i = 0; i < D_CONVEX_VERTEX_SPLITE_SIZE; i += 4) 
	{
		dVector temp;
		dInt32 j = i / 4;
		dVector::Transpose4x4(
			m_soaVertexArray->m_x[j], m_soaVertexArray->m_y[j], m_soaVertexArray->m_z[j], temp,
			m_vertex[i + 0], m_vertex[i + 1], m_vertex[i + 2], m_vertex[i + 3]);
		m_soaVertexArray->m_index[j] = index;
		index += step;
	}

	dFloat32* const indexPtr = &m_soaVertexArray->m_index[0][0];
	for (dInt32 i = m_vertexCount; i < D_CONVEX_VERTEX_SPLITE_SIZE; i++) 
	{
		indexPtr[i] = dFloat32(0.0f);
	}
}
