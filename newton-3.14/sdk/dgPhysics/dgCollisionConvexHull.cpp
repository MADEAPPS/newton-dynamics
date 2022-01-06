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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgCollisionConvexHull.h"



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define DG_CONVEX_VERTEX_BOX_CELL_SIZE	(1<<3)
#define DG_CONVEX_VERTEX_SPLITE_SIZE	(DG_CONVEX_VERTEX_BOX_CELL_SIZE * 3)

DG_MSC_VECTOR_ALIGNMENT
class dgCollisionConvexHull::dgSOAVectorArray
{
	public:
	dgVector m_x[DG_CONVEX_VERTEX_SPLITE_SIZE/4];
	dgVector m_y[DG_CONVEX_VERTEX_SPLITE_SIZE/4];
	dgVector m_z[DG_CONVEX_VERTEX_SPLITE_SIZE/4];
	dgVector m_index[DG_CONVEX_VERTEX_SPLITE_SIZE/4];
} DG_GCC_VECTOR_ALIGNMENT;

DG_MSC_VECTOR_ALIGNMENT
class dgCollisionConvexHull::dgConvexBox
{
	public:
	dgVector m_box[2];
	dgInt32 m_vertexStart;
	dgInt32 m_vertexCount;
	dgInt32 m_leftBox;
	dgInt32 m_rightBox;
} DG_GCC_VECTOR_ALIGNMENT;

dgCollisionConvexHull::dgCollisionConvexHull(dgMemoryAllocator* const allocator, dgUnsigned32 signature)
	:dgCollisionConvex(allocator, signature, m_convexHullCollision)
	,m_supportTree(NULL)
	,m_faceArray(NULL)
	,m_soaVertexArray(NULL)
	,m_vertexToEdgeMapping(NULL)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_edgeCount = 0;
	m_vertexCount = 0;
	m_vertex = NULL;
	m_simplex = NULL;
	m_rtti |= dgCollisionConvexHull_RTTI;
}

dgCollisionConvexHull::dgCollisionConvexHull(dgMemoryAllocator* const allocator, dgUnsigned32 signature, dgInt32 count, dgInt32 strideInBytes, dgFloat32 tolerance, const dgFloat32* const vertexArray)
	:dgCollisionConvex(allocator, signature, m_convexHullCollision)
	,m_supportTree(NULL)
	,m_faceArray(NULL)
	,m_soaVertexArray(NULL)
	,m_vertexToEdgeMapping(NULL)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_edgeCount = 0;
	m_vertexCount = 0;
	m_vertex = NULL;
	m_simplex = NULL;
	m_rtti |= dgCollisionConvexHull_RTTI;

	BuildHull (count, strideInBytes, tolerance, vertexArray);
}

dgCollisionConvexHull::dgCollisionConvexHull(dgWorld* const world, dgDeserialize callback, void* const userData, dgInt32 revisionNumber)
	:dgCollisionConvex (world, callback, userData, revisionNumber)
	,m_supportTree(NULL)
	,m_faceArray(NULL)
	,m_soaVertexArray(NULL)
	,m_vertexToEdgeMapping(NULL)
	,m_faceCount(0)
	,m_soaVertexCount(0)
	,m_supportTreeCount(0)
{
	m_rtti |= dgCollisionConvexHull_RTTI;

	dgInt32 edgeCount;
	dgInt32 vertexCount;
	callback(userData, &vertexCount, sizeof(dgInt32));
	callback(userData, &edgeCount, sizeof(dgInt32));
	callback(userData, &m_faceCount, sizeof(dgInt32));
	callback(userData, &m_supportTreeCount, sizeof(dgInt32));

	m_edgeCount = dgUnsigned16(edgeCount);
	m_vertexCount = dgUnsigned16 (vertexCount);
	
	m_vertex = (dgVector*) m_allocator->Malloc (dgInt32 (m_vertexCount * sizeof (dgVector)));
	m_simplex = (dgConvexSimplexEdge*) m_allocator->Malloc (dgInt32 (m_edgeCount * sizeof (dgConvexSimplexEdge)));
	m_faceArray = (dgConvexSimplexEdge **) m_allocator->Malloc(dgInt32 (m_faceCount * sizeof(dgConvexSimplexEdge *)));
	m_vertexToEdgeMapping = (const dgConvexSimplexEdge **) m_allocator->Malloc(dgInt32 (m_vertexCount * sizeof(dgConvexSimplexEdge *)));

	callback(userData, m_vertex, m_vertexCount * sizeof(dgVector));

	if (m_supportTreeCount) {
		m_supportTree = (dgConvexBox *) m_allocator->Malloc(dgInt32 (m_supportTreeCount * sizeof(dgConvexBox)));
		callback (userData, m_supportTree, m_supportTreeCount * sizeof(dgConvexBox));
	}

	for (dgInt32 i = 0; i < m_edgeCount; i ++) {
		dgInt32 serialization[4];
		callback (userData, serialization, sizeof (serialization));

		m_simplex[i].m_vertex = serialization[0];
		m_simplex[i].m_twin = m_simplex + serialization[1];
		m_simplex[i].m_next = m_simplex + serialization[2];
		m_simplex[i].m_prev = m_simplex + serialization[3];
	}

	for (dgInt32 i = 0; i < m_faceCount; i ++) {
		dgInt32 faceOffset;
		callback (userData, &faceOffset, sizeof (dgInt32));
		m_faceArray[i] = m_simplex + faceOffset; 
	}

	for (dgInt32 i = 0; i < m_vertexCount; i ++) {
		dgInt32 faceOffset;
		callback (userData, &faceOffset, sizeof (dgInt32));
		m_vertexToEdgeMapping[i] = m_simplex + faceOffset; 
	}

	if (vertexCount <= DG_CONVEX_VERTEX_SPLITE_SIZE) {
		CreateSOAdata();
	}

	SetVolumeAndCG ();
}

dgCollisionConvexHull::~dgCollisionConvexHull()
{
	if (m_vertexToEdgeMapping) {
		m_allocator->Free(m_vertexToEdgeMapping);
	}

	if (m_faceArray) {
		m_allocator->Free(m_faceArray);
	}

	if (m_supportTree) {
		m_allocator->Free(m_supportTree);
	}

	if (m_soaVertexArray) {
		m_allocator->Free(m_soaVertexArray);
	}
}

void dgCollisionConvexHull::Serialize(dgSerialize callback, void* const userData) const
{
	SerializeLow(callback, userData);

	dgInt32 edgeCount = m_edgeCount;
	dgInt32 vertexCount = m_vertexCount;
	callback(userData, &vertexCount, sizeof(dgInt32));
	callback(userData, &edgeCount, sizeof(dgInt32));
	callback(userData, &m_faceCount, sizeof(dgInt32));
	callback(userData, &m_supportTreeCount, sizeof(dgInt32));

	callback(userData, m_vertex, m_vertexCount * sizeof(dgVector));

	if (m_supportTreeCount) {
		callback(userData, m_supportTree, m_supportTreeCount * sizeof(dgConvexBox));
	}

	for (dgInt32 i = 0; i < m_edgeCount; i++) {
		dgInt32 serialization[4];
		serialization[0] = m_simplex[i].m_vertex;
		serialization[1] = dgInt32(m_simplex[i].m_twin - m_simplex);
		serialization[2] = dgInt32(m_simplex[i].m_next - m_simplex);
		serialization[3] = dgInt32(m_simplex[i].m_prev - m_simplex);
		callback(userData, serialization, sizeof(serialization));
	}

	for (dgInt32 i = 0; i < m_faceCount; i++) {
		dgInt32 faceOffset;
		faceOffset = dgInt32(m_faceArray[i] - m_simplex);
		callback(userData, &faceOffset, sizeof(dgInt32));
	}

	for (dgInt32 i = 0; i < m_vertexCount; i++) {
		dgInt32 faceOffset;
		faceOffset = dgInt32(m_vertexToEdgeMapping[i] - m_simplex);
		callback(userData, &faceOffset, sizeof(dgInt32));
	}
}

void dgCollisionConvexHull::BuildHull (dgInt32 count, dgInt32 strideInBytes, dgFloat32 tolerance, const dgFloat32* const vertexArray)
{
	Create (count, strideInBytes, vertexArray, tolerance);
}

void dgCollisionConvexHull::MassProperties ()
{
	dgFloat32 volume = dgCollisionConvex::CalculateMassProperties(dgGetIdentityMatrix(), m_inertia, m_crossInertia, m_centerOfMass);
	if (volume < dgFloat32 (1.0e-6f)) {
		volume = dgFloat32 (1.0e-6f);
	}
	dgFloat32 invVolume = dgFloat32(1.0f) / volume;
	m_inertia = m_inertia.Scale(invVolume);
	m_crossInertia = m_crossInertia.Scale(invVolume);
	m_centerOfMass = m_centerOfMass.Scale(invVolume);
	m_centerOfMass.m_w = volume;


	dgMatrix inertia(dgGetIdentityMatrix());
	inertia[0][0] = m_inertia[0];
	inertia[1][1] = m_inertia[1];
	inertia[2][2] = m_inertia[2];
	inertia[0][1] = m_crossInertia[2];
	inertia[1][0] = m_crossInertia[2];
	inertia[0][2] = m_crossInertia[1];
	inertia[2][0] = m_crossInertia[1];
	inertia[1][2] = m_crossInertia[0];
	inertia[2][1] = m_crossInertia[0];

	dgVector origin(m_centerOfMass);
	dgFloat32 originMag2 = origin.DotProduct(origin & dgVector::m_triplexMask).GetScalar();

	dgMatrix Covariance(origin, origin);
	dgMatrix parallel(dgGetIdentityMatrix());
	for (dgInt32 i = 0; i < 3; i++) {
		parallel[i][i] = originMag2;
		inertia[i] -= (parallel[i] - Covariance[i]);
		dgAssert(inertia[i][i] > dgFloat32(0.0f));
	}

	m_inertia[0] = inertia[0][0];
	m_inertia[1] = inertia[1][1];
	m_inertia[2] = inertia[2][2];
	m_crossInertia[0] = inertia[2][1];
	m_crossInertia[1] = inertia[2][0];
	m_crossInertia[2] = inertia[1][0];
}

dgInt32 dgCollisionConvexHull::GetFaceIndices (dgInt32 index, dgInt32* const indices) const
{
	dgInt32 count = 0;
	const dgConvexSimplexEdge* face = m_faceArray[index];
	do {
		indices [count] = face->m_vertex;
		count ++;
		face = face->m_next;
	} while (face != m_faceArray[index]);

	return count;
}

dgBigVector dgCollisionConvexHull::FaceNormal (const dgEdge *face, const dgBigVector* const pool) const
{
	const dgEdge* edge = face;
	dgBigVector p0 (pool[edge->m_incidentVertex]);
	edge = edge->m_next;

	dgBigVector p1 (pool[edge->m_incidentVertex]);
	dgBigVector e1 (p1 - p0);

	dgBigVector normal (dgFloat32 (0.0f));
	for (edge = edge->m_next; edge != face; edge = edge->m_next) {
		dgBigVector p2 (pool[edge->m_incidentVertex]);
		dgBigVector e2 (p2 - p0);
		dgBigVector n1 (e1.CrossProduct(e2));
#ifdef _DEBUG
		dgAssert(n1.m_w == dgFloat32(0.0f));
		dgFloat64 mag = normal.DotProduct(n1).GetScalar();
		dgAssert ( mag >= -dgFloat32 (0.1f));
#endif
		normal += n1;
		e1 = e2;
	} 

	dgFloat64 den = sqrt (normal.DotProduct(normal).GetScalar()) + dgFloat64 (1.0e-24f);
	normal = normal.Scale (dgFloat64 (1.0f)/ den);

#ifdef _DEBUG
	edge = face;
	dgBigVector e0 (pool[edge->m_incidentVertex] - pool[edge->m_prev->m_incidentVertex]);	
	do {
		dgBigVector de1 (pool[edge->m_next->m_incidentVertex] - pool[edge->m_incidentVertex]);	
		dgBigVector dn1 (e0.CrossProduct(de1));
		dgFloat64 x = normal.DotProduct(dn1).GetScalar();
		dgAssert (x > -dgFloat64 (0.01f));
		e0 = de1;
		edge = edge->m_next;
	} while (edge != face);
#endif
	return normal;
}

bool dgCollisionConvexHull::RemoveCoplanarEdge (dgPolyhedra& polyhedra, const dgBigVector* const hullVertexArray) const
{
	bool removeEdge = false;
	// remove coplanar edges
	dgInt32 mark = polyhedra.IncLRU();
	dgPolyhedra::Iterator iter (polyhedra);
	for (iter.Begin(); iter; ) {
		dgEdge* edge0 = &(*iter);
		iter ++;

		if (edge0->m_incidentFace != -1) {

			if (edge0->m_mark < mark) {
				edge0->m_mark = mark;
				edge0->m_twin->m_mark = mark;
				dgBigVector normal0 (FaceNormal (edge0, &hullVertexArray[0]));
				dgBigVector normal1 (FaceNormal (edge0->m_twin, &hullVertexArray[0]));

				dgFloat64 test = normal0.DotProduct(normal1).GetScalar();
				if (test > dgFloat64 (0.99995f)) {

					if ((edge0->m_twin->m_next->m_twin->m_next != edge0) && (edge0->m_next->m_twin->m_next != edge0->m_twin)) {
						#define DG_MAX_EDGE_ANGLE dgFloat32 (1.0e-3f)

						if (edge0->m_twin == &(*iter)) {
							if (iter) {
								iter ++;
							}
						}

						dgBigVector e1 (hullVertexArray[edge0->m_twin->m_next->m_next->m_incidentVertex] - hullVertexArray[edge0->m_incidentVertex]);
						dgBigVector e0 (hullVertexArray[edge0->m_incidentVertex] - hullVertexArray[edge0->m_prev->m_incidentVertex]);

						dgAssert(e0.m_w == dgFloat64(0.0f));
						dgAssert(e1.m_w == dgFloat64(0.0f));
						dgAssert (e0.DotProduct(e0).GetScalar() >= dgFloat64 (0.0f));
						dgAssert (e1.DotProduct(e1).GetScalar() >= dgFloat64 (0.0f));

						e0 = e0.Scale (dgFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar()));
						e1 = e1.Scale (dgFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar()));
						dgBigVector n1 (e0.CrossProduct(e1));

						dgFloat64 projection = n1.DotProduct(normal0).GetScalar();
						if (projection >= DG_MAX_EDGE_ANGLE) {

							dgBigVector e11 (hullVertexArray[edge0->m_next->m_next->m_incidentVertex] - hullVertexArray[edge0->m_twin->m_incidentVertex]);
							dgBigVector e00 (hullVertexArray[edge0->m_twin->m_incidentVertex] - hullVertexArray[edge0->m_twin->m_prev->m_incidentVertex]);
							dgAssert (e00.m_w == dgFloat64 (0.0f));
							dgAssert (e11.m_w == dgFloat64 (0.0f));
							dgAssert (e00.DotProduct(e00).GetScalar() >= dgFloat64 (0.0f));
							dgAssert (e11.DotProduct(e11).GetScalar() >= dgFloat64 (0.0f));
							e00 = e00.Scale(dgFloat64(1.0f) / sqrt(e00.DotProduct(e00).GetScalar()));
							e11 = e11.Scale(dgFloat64(1.0f) / sqrt(e11.DotProduct(e11).GetScalar()));

							dgBigVector n11 (e00.CrossProduct(e11));
							projection = n11.DotProduct(normal0).GetScalar();
							if (projection >= DG_MAX_EDGE_ANGLE) {
								dgAssert (&(*iter) != edge0);
								dgAssert (&(*iter) != edge0->m_twin);
								polyhedra.DeleteEdge(edge0);
								removeEdge = true;
							}
						}

					} else {
						dgEdge* next = edge0->m_next;
						dgEdge* prev = edge0->m_prev;
						polyhedra.DeleteEdge(edge0);
						for (edge0 = next; edge0->m_prev->m_twin == edge0; edge0 = next) {
							next = edge0->m_next;
							polyhedra.DeleteEdge(edge0);
						}

						for (edge0 = prev; edge0->m_next->m_twin == edge0; edge0 = prev) {
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


bool dgCollisionConvexHull::CheckConvex (dgPolyhedra& polyhedra1, const dgBigVector* hullVertexArray) const
{
	dgPolyhedra polyhedra(polyhedra1);

	dgPolyhedra::Iterator iter (polyhedra);
	dgBigVector center (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	dgInt32 count = 0;
	dgInt32 mark = polyhedra.IncLRU();
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		if (edge->m_mark < mark) {
			count ++;
			center += hullVertexArray[edge->m_incidentVertex];
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
		}
	}
	center = center.Scale (dgFloat64 (1.0f) / dgFloat64 (count));

	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &(*iter);
		dgBigVector normal0 (FaceNormal (edge, hullVertexArray));
		dgBigVector normal1 (FaceNormal (edge->m_twin, hullVertexArray));

		dgBigPlane plane0 (normal0, - normal0.DotProduct(hullVertexArray[edge->m_incidentVertex]).GetScalar());
		dgBigPlane plane1 (normal1, - normal1.DotProduct(hullVertexArray[edge->m_twin->m_incidentVertex]).GetScalar());
		dgFloat64 test0 = plane0.Evalue(center);
		if (test0 > dgFloat64 (1.0e-3f)) {
			return false;
		}
		dgFloat64 test1 = plane1.Evalue(center);
//		if (test1 > dgFloat64 (0.0f)) {
		if (test1 > dgFloat64 (1.0e-3f)) {
			return false;
		}
	}
	return true;
}

bool dgCollisionConvexHull::Create (dgInt32 count, dgInt32 strideInBytes, const dgFloat32* const vertexArray, dgFloat32 tolerance)
{
	dgInt32 stride = strideInBytes / sizeof (dgFloat32);
	dgStack<dgFloat64> buffer(3 * 2 * count);
	for (dgInt32 i = 0; i < count; i ++) {
		buffer[i * 3 + 0] = vertexArray[i * stride + 0];
		buffer[i * 3 + 1] = vertexArray[i * stride + 1];
		buffer[i * 3 + 2] = vertexArray[i * stride + 2];
	}

	dgConvexHull3d* convexHull =  new (GetAllocator()) dgConvexHull3d (GetAllocator(), &buffer[0], 3 * sizeof (dgFloat64), count, tolerance);
	if (!convexHull->GetCount()) {
		// this is a degenerated hull hull to add some thickness and for a thick plane
		delete convexHull;

		dgStack<dgVector> tmp(3 * count);
		for (dgInt32 i = 0; i < count; i ++) {
			tmp[i][0] = dgFloat32 (buffer[i*3 + 0]);
			tmp[i][1] = dgFloat32 (buffer[i*3 + 1]);
			tmp[i][2] = dgFloat32 (buffer[i*3 + 2]);
			tmp[i][3] = dgFloat32 (0.0f);
		}
	
		dgObb sphere;
		sphere.SetDimensions (&tmp[0][0], sizeof (dgVector), count);

		dgInt32 index = 0;
		dgFloat32 size = dgFloat32 (1.0e10f);
		for (dgInt32 i = 0; i < 3; i ++) {
			if (sphere.m_size[i] < size) {
				index = i;
				size = sphere.m_size[i];
			}
		}
		dgVector normal (dgFloat32 (0.0f));
		normal[index] = dgFloat32 (1.0f);
		dgVector step = sphere.RotateVector (normal.Scale (dgFloat32 (0.05f)));
		for (dgInt32 i = 0; i < count; i ++) {
			dgVector p1 (tmp[i] + step);
			dgVector p2 (tmp[i] - step);

			buffer[i * 3 + 0] = p1.m_x;
			buffer[i * 3 + 1] = p1.m_y;
			buffer[i * 3 + 2] = p1.m_z;
			buffer[(i + count) * 3 + 0] = p2.m_x;
			buffer[(i + count) * 3 + 1] = p2.m_y;
			buffer[(i + count) * 3 + 2] = p2.m_z;
		}
		count *= 2;
		convexHull =  new (GetAllocator()) dgConvexHull3d (GetAllocator(), &buffer[0], 3 * sizeof (dgFloat64), count, tolerance);
		if (!convexHull->GetCount()) {
			delete convexHull;
			return false;
		}
	}

	// check for degenerated faces
	for (bool success = false; !success;  ) {
		success = true;
		const dgBigVector* const hullVertexArray = convexHull->GetVertexPool();

		dgStack<dgInt8> mask(convexHull->GetVertexCount());
		memset (&mask[0], 1, mask.GetSizeInBytes());
		for (dgConvexHull3d::dgListNode* node = convexHull->GetFirst(); node; node = node->GetNext()) {
			dgConvexHull3DFace& face = node->GetInfo();
			const dgBigVector& p0 = hullVertexArray[face.m_index[0]];
			const dgBigVector& p1 = hullVertexArray[face.m_index[1]];
			const dgBigVector& p2 = hullVertexArray[face.m_index[2]];
			dgAssert(p0.m_w == p1.m_w);
			dgAssert(p0.m_w == p2.m_w);
			dgBigVector p1p0 (p1 - p0);
			dgBigVector p2p0 (p2 - p0);
			dgBigVector normal (p2p0.CrossProduct(p1p0));
			dgFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < dgFloat64 (1.0e-6f * 1.0e-6f)) {
				success = false;
				dgInt32 index = -1;
				dgBigVector p2p1 (p2 - p1);
				dgFloat64 dist10 = p1p0.DotProduct(p1p0).GetScalar();
				dgFloat64 dist20 = p2p0.DotProduct(p2p0).GetScalar();
				dgFloat64 dist21 = p2p1.DotProduct(p2p1).GetScalar();
				if ((dist10 >= dist20) && (dist10 >= dist21)) {
					index = 2;
				} else if ((dist20 >= dist10) && (dist20 >= dist21)) {
					index = 1;
				} else if ((dist21 >= dist10) && (dist21 >= dist20)) {
					index = 0;
				}
				dgAssert (index != -1);
				mask[face.m_index[index]] = 0;
			}
		}
		if (!success) {
			dgInt32 count1 = 0;
			dgInt32 vertexCount = convexHull->GetVertexCount();
			for (dgInt32 i = 0; i < vertexCount; i ++) {
				if (mask[i]) {
					buffer[count1 * 3 + 0] = hullVertexArray[i].m_x;
					buffer[count1 * 3 + 1] = hullVertexArray[i].m_y;
					buffer[count1 * 3 + 2] = hullVertexArray[i].m_z;
					count1 ++;
				}
			}
			delete convexHull;
			convexHull =  new (GetAllocator()) dgConvexHull3d (GetAllocator(), &buffer[0], 3 * sizeof (dgFloat64), count1, tolerance);
		}
	}

	dgAssert (convexHull);
	dgInt32 vertexCount = convexHull->GetVertexCount();
	if (vertexCount < 4) {
		delete convexHull;
		return false;
	}

	const dgBigVector* const hullVertexArray = convexHull->GetVertexPool();
	dgPolyhedra polyhedra (GetAllocator());
	polyhedra.BeginFace();
	for (dgConvexHull3d::dgListNode* node = convexHull->GetFirst(); node; node = node->GetNext()) {
		dgConvexHull3DFace& face = node->GetInfo();
		polyhedra.AddFace (face.m_index[0], face.m_index[1], face.m_index[2]);
	}
	polyhedra.EndFace();

	if (vertexCount > 4) {
		while (RemoveCoplanarEdge (polyhedra, hullVertexArray));
	}

	dgStack<dgInt32> vertexMap(vertexCount);
	memset (&vertexMap[0], -1, vertexCount * sizeof (dgInt32));

	dgInt32 mark = polyhedra.IncLRU();
	dgPolyhedra::Iterator iter (polyhedra);
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) {
			if (vertexMap[edge->m_incidentVertex] == -1) {
				vertexMap[edge->m_incidentVertex] = m_vertexCount;
				m_vertexCount ++;
			}
			dgEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr->m_userData = m_edgeCount;
				m_edgeCount ++;
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge) ;
		}
	} 

	m_vertex = (dgVector*) m_allocator->Malloc (dgInt32 (m_vertexCount * sizeof (dgVector)));
	m_simplex = (dgConvexSimplexEdge*) m_allocator->Malloc (dgInt32 (m_edgeCount * sizeof (dgConvexSimplexEdge)));
	m_vertexToEdgeMapping = (const dgConvexSimplexEdge**) m_allocator->Malloc (dgInt32 (m_vertexCount * sizeof (dgConvexSimplexEdge*)));

	for (dgInt32 i = 0; i < vertexCount; i ++) {
		if (vertexMap[i] != -1) {
			m_vertex[vertexMap[i]] = hullVertexArray[i];
			m_vertex[vertexMap[i]].m_w = dgFloat32 (0.0f);
		}
	}
	delete convexHull;

	vertexCount = m_vertexCount;
	mark = polyhedra.IncLRU();;
	for (iter.Begin(); iter; iter ++) {
		dgEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_mark != mark) {
			dgEdge *ptr = edge;
			do {
				ptr->m_mark = mark;
				dgConvexSimplexEdge* const simplexPtr = &m_simplex[ptr->m_userData];
				simplexPtr->m_vertex = vertexMap[ptr->m_incidentVertex];
				simplexPtr->m_next = &m_simplex[ptr->m_next->m_userData];
				simplexPtr->m_prev = &m_simplex[ptr->m_prev->m_userData];
				simplexPtr->m_twin = &m_simplex[ptr->m_twin->m_userData];

				ptr = ptr->m_twin->m_next;
			} while (ptr != edge) ;
		}
	} 

	
	m_faceCount = 0;
	dgStack<char> faceMarks (m_edgeCount);
	memset (&faceMarks[0], 0, m_edgeCount * sizeof (dgInt8));

	dgStack<dgConvexSimplexEdge*> faceArray (m_edgeCount);
	for (dgInt32 i = 0; i < m_edgeCount; i ++) {
		dgConvexSimplexEdge* const face = &m_simplex[i];
		if (!faceMarks[i]) {
			dgConvexSimplexEdge* ptr = face;
			do {
				dgAssert ((ptr - m_simplex) >= 0);
				faceMarks[dgInt32 (ptr - m_simplex)] = '1';
				ptr = ptr->m_next;
			} while (ptr != face);

			faceArray[m_faceCount] = face;
			m_faceCount ++;
		}
	}
	m_faceArray = (dgConvexSimplexEdge **) m_allocator->Malloc(dgInt32 (m_faceCount * sizeof(dgConvexSimplexEdge *)));
	memcpy (m_faceArray, &faceArray[0], m_faceCount * sizeof(dgConvexSimplexEdge *));
	
	if (vertexCount > DG_CONVEX_VERTEX_SPLITE_SIZE) {
		// create a face structure for support vertex
			dgStack<dgConvexBox> boxTree (vertexCount);
		dgTree<dgVector,dgInt32> sortTree(GetAllocator());
		dgStack<dgTree<dgVector,dgInt32>::dgTreeNode*> vertexNodeList(vertexCount);

		dgVector boxP0 ( dgFloat32 (1.0e15f)); 
		dgVector boxP1 (-dgFloat32 (1.0e15f));
		for (dgInt32 i = 0; i < vertexCount; i ++) {
			const dgVector& p = m_vertex[i];
			vertexNodeList[i] = sortTree.Insert (p, i);
			boxP0 = boxP0.GetMin(p);
			boxP1 = boxP1.GetMax(p);
		}

		boxTree[0].m_box[0] = boxP0 & dgVector::m_triplexMask;
		boxTree[0].m_box[1] = boxP1 & dgVector::m_triplexMask;
		boxTree[0].m_leftBox = -1;
		boxTree[0].m_rightBox = -1;
		boxTree[0].m_vertexStart = 0;
		boxTree[0].m_vertexCount = vertexCount;
		dgInt32 boxCount = 1;

		dgInt32 stack = 1;
		dgInt32 stackBoxPool[64];
		stackBoxPool[0] = 0;

		while (stack) {
			stack --;
			dgInt32 boxIndex = stackBoxPool[stack];
			dgConvexBox& box = boxTree[boxIndex];
			if (box.m_vertexCount > DG_CONVEX_VERTEX_BOX_CELL_SIZE) {
				dgVector median (dgFloat32 (0.0f));
				dgVector varian (dgFloat32 (0.0f));
				for (dgInt32 i = 0; i < box.m_vertexCount; i ++) {
					dgVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
					boxP0 = boxP0.GetMin(p);
					boxP1 = boxP1.GetMax(p);
					median += p;
					varian += p * p;
				}

				varian = varian.Scale (dgFloat32 (box.m_vertexCount)) - median * median;
				dgInt32 index = 0;
				dgFloat64 maxVarian = dgFloat64 (-1.0e10f);
				for (dgInt32 i = 0; i < 3; i ++) {
					if (varian[i] > maxVarian) {
						index = i;
						maxVarian = varian[i];
					}
				}
				dgVector center = median.Scale (dgFloat32 (1.0f) / dgFloat32 (box.m_vertexCount));
				dgFloat32 test = center[index];

				dgInt32 i0 = 0;
				dgInt32 i1 = box.m_vertexCount - 1;
				do {    
					for (; i0 <= i1; i0 ++) {
						dgFloat32 val = vertexNodeList[box.m_vertexStart + i0]->GetInfo()[index];
						if (val > test) {
							break;
						}
					}

					for (; i1 >= i0; i1 --) {
						dgFloat32 val = vertexNodeList[box.m_vertexStart + i1]->GetInfo()[index];
						if (val < test) {
							break;
						}
					}

					if (i0 < i1)	{
						dgSwap(vertexNodeList[box.m_vertexStart + i0], vertexNodeList[box.m_vertexStart + i1]);
						i0++; 
						i1--;
					}
				} while (i0 <= i1);

				if (i0 == 0){
					i0 = box.m_vertexCount / 2;
				}
				if (i0 >= (box.m_vertexCount - 1)){
					i0 = box.m_vertexCount / 2;
				}

				{
					// insert right branch AABB
					dgVector rightBoxP0 ( dgFloat32 (1.0e15f)); 
					dgVector rightBoxP1 (-dgFloat32 (1.0e15f)); 	
					for (dgInt32 i = i0; i < box.m_vertexCount; i ++) {
						const dgVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
						rightBoxP0 = rightBoxP0.GetMin(p);
						rightBoxP1 = rightBoxP1.GetMax(p);
					}

					box.m_rightBox = boxCount;
					boxTree[boxCount].m_box[0] = rightBoxP0 & dgVector::m_triplexMask;
					boxTree[boxCount].m_box[1] = rightBoxP1 & dgVector::m_triplexMask;
					boxTree[boxCount].m_leftBox = -1;
					boxTree[boxCount].m_rightBox = -1;
					boxTree[boxCount].m_vertexStart = box.m_vertexStart + i0;
					boxTree[boxCount].m_vertexCount = box.m_vertexCount - i0;
					stackBoxPool[stack] = boxCount;
					stack ++;
					boxCount ++;
				}

				{
					// insert left branch AABB
					dgVector leftBoxP0 ( dgFloat32 (1.0e15f));
					dgVector leftBoxP1 (-dgFloat32 (1.0e15f));
					for (dgInt32 i = 0; i < i0; i ++) {
						const dgVector& p = vertexNodeList[box.m_vertexStart + i]->GetInfo();
						leftBoxP0 = leftBoxP0.GetMin(p);
						leftBoxP1 = leftBoxP1.GetMax(p);
					}

					box.m_leftBox = boxCount;
					boxTree[boxCount].m_box[0] = leftBoxP0 & dgVector::m_triplexMask;;
					boxTree[boxCount].m_box[1] = leftBoxP1 & dgVector::m_triplexMask;;
					boxTree[boxCount].m_leftBox = -1;
					boxTree[boxCount].m_rightBox = -1;
					boxTree[boxCount].m_vertexStart = box.m_vertexStart;
					boxTree[boxCount].m_vertexCount = i0;
					stackBoxPool[stack] = boxCount;
					stack ++;
					boxCount ++;
				}
			}
		}

		for (dgInt32 i = 0; i < m_vertexCount; i ++) {
			m_vertex[i] = vertexNodeList[i]->GetInfo();
			vertexNodeList[i]->GetInfo().m_w = dgFloat32 (i);
		}

		m_supportTreeCount = boxCount;
		m_supportTree = (dgConvexBox*) m_allocator->Malloc(dgInt32 (boxCount * sizeof(dgConvexBox)));		
		memcpy (m_supportTree, &boxTree[0], boxCount * sizeof(dgConvexBox));

		for (dgInt32 i = 0; i < m_edgeCount; i ++) {
			dgConvexSimplexEdge* const ptr = &m_simplex[i];
			dgTree<dgVector,dgInt32>::dgTreeNode* const node = sortTree.Find(ptr->m_vertex);
			dgInt32 index = dgInt32 (node->GetInfo().m_w);
			ptr->m_vertex = dgInt16 (index);
		}
	} else {
		CreateSOAdata();
	}

	for (dgInt32 i = 0; i < m_edgeCount; i ++) {
		dgConvexSimplexEdge* const edge = &m_simplex[i];
		m_vertexToEdgeMapping[edge->m_vertex] = edge;
	}

	SetVolumeAndCG ();
	return true;
}

dgInt32 dgCollisionConvexHull::CalculateSignature (dgInt32 vertexCount, const dgFloat32* const vertexArray, dgInt32 strideInBytes)
{
	dgStack<dgUnsigned32> buffer(1 + 3 * vertexCount);  
	dgInt32 stride = dgInt32 (strideInBytes / sizeof (dgFloat32));

	memset (&buffer[0], 0, size_t (buffer.GetSizeInBytes()));
	buffer[0] = m_convexHullCollision;
	
	for (dgInt32 i = 0; i < vertexCount; i ++) {
		buffer[1 + i * 3 + 0] = dgCollision::Quantize (vertexArray[i * stride + 0]);
		buffer[1 + i * 3 + 1] = dgCollision::Quantize (vertexArray[i * stride + 1]);
		buffer[1 + i * 3 + 2] = dgCollision::Quantize (vertexArray[i * stride + 2]);
	}
	return Quantize(&buffer[0], buffer.GetSizeInBytes());
}

dgInt32 dgCollisionConvexHull::CalculateSignature () const
{
	return dgInt32 (GetSignature());
//	return CalculateSignature (m_vertexCount, &m_vertex[0].m_x, sizeof (dgVector));
}



void dgCollisionConvexHull::SetCollisionBBox (const dgVector& p0__, const dgVector& p1__)
{
	dgAssert (0);
}



void dgCollisionConvexHull::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgTriplex vertex[256];
	for (dgInt32 i = 0; i < m_faceCount; i ++) {
		dgConvexSimplexEdge* const face = m_faceArray[i];
		dgConvexSimplexEdge* ptr = face;
		dgInt32 count = 0;
		do {
			vertex[count].m_x = m_vertex[ptr->m_vertex].m_x;
			vertex[count].m_y = m_vertex[ptr->m_vertex].m_y;
			vertex[count].m_z = m_vertex[ptr->m_vertex].m_z;
			count ++;
			dgAssert (count < sizeof (vertex)/ sizeof (vertex[0]));
			ptr = ptr->m_next;
		} while (ptr != face);
		matrix.TransformTriplex (&vertex[0].m_x, sizeof (dgTriplex), &vertex[0].m_x, sizeof (dgTriplex), count);
		callback (userData, count, &vertex[0].m_x, 0);
	}
}

void dgCollisionConvexHull::GetCollisionInfo(dgCollisionInfo* const info) const
{
	dgCollisionConvex::GetCollisionInfo(info);

	info->m_convexHull.m_vertexCount = m_vertexCount;
	info->m_convexHull.m_strideInBytes = sizeof (dgVector);
	info->m_convexHull.m_faceCount = m_faceCount;
	info->m_convexHull.m_vertex = &m_vertex[0];
}

void dgCollisionConvexHull::CreateSOAdata()
{
	m_soaVertexArray = (dgSOAVectorArray*)m_allocator->Malloc(sizeof(dgSOAVectorArray));

	m_soaVertexCount = ((m_vertexCount + 7) & -8) / 8;
	dgVector array[DG_CONVEX_VERTEX_SPLITE_SIZE];
	for (dgInt32 i = 0; i < m_vertexCount; i++) {
		array[i] = m_vertex[i];
	}

	for (dgInt32 i = m_vertexCount; i < DG_CONVEX_VERTEX_SPLITE_SIZE; i++) {
		array[i] = array[0];
	}

	dgVector step(dgFloat32(4.0f));
	dgVector index(dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(2.0f), dgFloat32(3.0f));
	for (dgInt32 i = 0; i < DG_CONVEX_VERTEX_SPLITE_SIZE; i += 4) {
		dgVector temp;
		dgInt32 j = i / 4;
		dgVector::Transpose4x4(m_soaVertexArray->m_x[j], m_soaVertexArray->m_y[j], m_soaVertexArray->m_z[j], temp,
			m_vertex[i + 0], m_vertex[i + 1], m_vertex[i + 2], m_vertex[i + 3]);
		m_soaVertexArray->m_index[j] = index;
		index += step;
	}

	dgFloat32* const indexPtr = &m_soaVertexArray->m_index[0][0];
	for (dgInt32 i = m_vertexCount; i < DG_CONVEX_VERTEX_SPLITE_SIZE; i++) {
		indexPtr[i] = dgFloat32(0.0f);
	}
}

dgVector dgCollisionConvexHull::SupportVertex(const dgVector& dir, dgInt32* const vertexIndex) const
{
	dgAssert(dir.m_w == dgFloat32(0.0f));
	dgInt32 index = -1;
	dgVector maxProj(dgFloat32(-1.0e20f));
	if (m_vertexCount > DG_CONVEX_VERTEX_SPLITE_SIZE) {
		dgFloat32 distPool[32];
		const dgConvexBox* stackPool[32];

		dgInt32 ix = (dir[0] > dgFloat64(0.0f)) ? 1 : 0;
		dgInt32 iy = (dir[1] > dgFloat64(0.0f)) ? 1 : 0;
		dgInt32 iz = (dir[2] > dgFloat64(0.0f)) ? 1 : 0;

		const dgConvexBox& leftBox = m_supportTree[m_supportTree[0].m_leftBox];
		const dgConvexBox& rightBox = m_supportTree[m_supportTree[0].m_rightBox];

		dgVector leftP(leftBox.m_box[ix][0], leftBox.m_box[iy][1], leftBox.m_box[iz][2], dgFloat32(0.0f));
		dgVector rightP(rightBox.m_box[ix][0], rightBox.m_box[iy][1], rightBox.m_box[iz][2], dgFloat32(0.0f));

		dgFloat32 leftDist = leftP.DotProduct(dir).GetScalar();
		dgFloat32 rightDist = rightP.DotProduct(dir).GetScalar();
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

		dgInt32 stack = 2;

		while (stack) {
			stack--;
			dgFloat32 dist = distPool[stack];
			if (dist > maxProj.m_x) {
				const dgConvexBox& box = *stackPool[stack];

				if (box.m_leftBox > 0) {
					dgAssert(box.m_rightBox > 0);
					const dgConvexBox& leftBox1 = m_supportTree[box.m_leftBox];
					const dgConvexBox& rightBox1 = m_supportTree[box.m_rightBox];

					dgVector leftBoxP(leftBox1.m_box[ix][0], leftBox1.m_box[iy][1], leftBox1.m_box[iz][2], dgFloat32(0.0f));
					dgVector rightBoxP(rightBox1.m_box[ix][0], rightBox1.m_box[iy][1], rightBox1.m_box[iz][2], dgFloat32(0.0f));

					dgFloat32 leftBoxDist = leftBoxP.DotProduct(dir).GetScalar();
					dgFloat32 rightBoxDist = rightBoxP.DotProduct(dir).GetScalar();
					if (rightBoxDist >= leftBoxDist) {
						distPool[stack] = leftBoxDist;
						stackPool[stack] = &leftBox1;
						stack++;
						dgAssert(stack < sizeof (distPool) / sizeof (distPool[0]));

						distPool[stack] = rightBoxDist;
						stackPool[stack] = &rightBox1;
						stack++;
						dgAssert(stack < sizeof (distPool) / sizeof (distPool[0]));

					} else {
						distPool[stack] = rightBoxDist;
						stackPool[stack] = &rightBox1;
						stack++;
						dgAssert(stack < sizeof (distPool) / sizeof (distPool[0]));

						distPool[stack] = leftBoxDist;
						stackPool[stack] = &leftBox1;
						stack++;
						dgAssert(stack < sizeof (distPool) / sizeof (distPool[0]));
					}
				} else {
					for (dgInt32 i = 0; i < box.m_vertexCount; i++) {
						const dgVector& p = m_vertex[box.m_vertexStart + i];
						dgAssert(p.m_x >= box.m_box[0].m_x);
						dgAssert(p.m_x <= box.m_box[1].m_x);
						dgAssert(p.m_y >= box.m_box[0].m_y);
						dgAssert(p.m_y <= box.m_box[1].m_y);
						dgAssert(p.m_z >= box.m_box[0].m_z);
						dgAssert(p.m_z <= box.m_box[1].m_z);
						dgVector projectionDist(p.DotProduct(dir));
						dgVector mask(projectionDist > maxProj);
						dgInt32 intMask = *((dgInt32*)&mask.m_x);
						index = ((box.m_vertexStart + i) & intMask) | (index & ~intMask);
						maxProj = maxProj.GetMax(projectionDist);
					}
				}
			}
		}
	} else {
#if 0
		for (dgInt32 i = 0; i < m_vertexCount; i++) {
			const dgVector& p = m_vertex[i];
			dgVector dist(p.DotProduct(dir));
			dgVector mask(dist > maxProj);
			dgInt32 intMask = *((dgInt32*)&mask.m_x);
			index = (i & intMask) | (index & ~intMask);
			maxProj = maxProj.GetMax(dist);
		}
#else
		const dgVector x(dir.m_x);
		const dgVector y(dir.m_y);
		const dgVector z(dir.m_z);
		dgVector support (dgVector::m_negOne);
		for (dgInt32 i = 0; i < m_soaVertexCount; i+=2) {
			dgVector dot (m_soaVertexArray->m_x[i] * x + 
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
		 
		dgVector dot (maxProj.ShiftRight().ShiftRight());
		dgVector support1 (support.ShiftRight().ShiftRight());
		support = support.Select(support1, dot > maxProj);
		maxProj = maxProj.GetMax(dot);

		dot = dgVector (maxProj.ShiftRight());
		support1 = dgVector (support.ShiftRight());
		support = support.Select(support1, dot > maxProj);

		index = dgInt32 (support.GetScalar()); 
#endif
	}

	if (vertexIndex) {
		*vertexIndex = index;
	}
	dgAssert(index != -1);
	return m_vertex[index];
}


