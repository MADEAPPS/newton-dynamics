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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndScene.h"
#include "ndContact.h"
#include "ndShapeNull.h"
#include "ndShapeConvex.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"

#define D_MAX_MIN_VOLUME				ndFloat32 (1.0e-6f)
#define D_MAX_VERTEX_CLIP_FACE			16

ndShapeConvex::ndShapeConvex(ndShapeID id)
	:ndShape(id)
	,m_vertex(nullptr)
	,m_simplex(nullptr)
	,m_boxMinRadius(ndFloat32(0.0f))
	,m_boxMaxRadius(ndFloat32(0.0f))
	,m_simplexVolume(ndFloat32(0.0f))
	,m_edgeCount(0)
	,m_vertexCount(0)
{
}

ndShapeConvex::~ndShapeConvex()
{
	if (m_vertex) 
	{
		ndMemory::Free(m_vertex);
	}
	
	if (m_simplex) 
	{
		ndMemory::Free(m_simplex);
	}
}

void ndShapeConvex::DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const
{
	ndVector tmp[D_MAX_EDGE_COUNT];
	ndVector vertex[D_MAX_EDGE_COUNT];
	ndInt8 mark[D_MAX_EDGE_COUNT];
	ndShapeDebugNotify::ndEdgeType edgeType[D_MAX_EDGE_COUNT];

	dAssert(m_edgeCount < D_MAX_EDGE_COUNT);
	dAssert(m_vertexCount < D_MAX_EDGE_COUNT);

	memset(mark, 0, sizeof(mark));
	memset(edgeType, ndShapeDebugNotify::m_shared, sizeof(edgeType));
	matrix.TransformTriplex(&tmp[0].m_x, sizeof(ndVector), &m_vertex[0].m_x, sizeof(ndVector), m_vertexCount);
	for (ndInt32 i = 0; i < m_edgeCount; i++) 
	{
		if (!mark[i]) 
		{
			ndConvexSimplexEdge* const face = &m_simplex[i];
			ndConvexSimplexEdge* edge = face;
			ndInt32 count = 0;
			do 
			{
				mark[edge - m_simplex] = '1';
				ndInt32 index = edge->m_vertex;
				vertex[count] = tmp[index] & ndVector::m_triplexMask;
				count++;
				edge = edge->m_next;
			} while (edge != face);
			debugCallback.DrawPolygon(count, vertex, edgeType);
		}
	}
}

void ndShapeConvex::SetVolumeAndCG()
{
	ndVector faceVertex[D_MAX_EDGE_COUNT];
	ndInt8* const edgeMarks = dAlloca(ndInt8, m_edgeCount + 32);
	memset(&edgeMarks[0], 0, sizeof(ndInt8) * m_edgeCount);

	ndPolyhedraMassProperties localData;
	for (ndInt32 i = 0; i < m_edgeCount; i++) 
	{
		ndConvexSimplexEdge* const face = &m_simplex[i];
		if (!edgeMarks[i]) {
			ndConvexSimplexEdge* edge = face;
			ndInt32 count = 0;
			do 
			{
				dAssert((edge - m_simplex) >= 0);
				edgeMarks[ndInt32(edge - m_simplex)] = '1';
				faceVertex[count] = m_vertex[edge->m_vertex];
				count++;
				dAssert(count < ndInt32(sizeof(faceVertex) / sizeof(faceVertex[0])));
				edge = edge->m_next;
			} while (edge != face);

			localData.AddCGFace(count, faceVertex);
		}
	}

	ndVector origin;
	ndVector inertia;
	ndVector crossInertia;
	ndFloat32 volume = localData.MassProperties(origin, inertia, crossInertia);
	m_simplexVolume = volume;

	// calculate the origin of the bound box of this primitive
	ndVector p0(ndVector::m_zero);
	ndVector p1(ndVector::m_zero);

	for (ndInt32 i = 0; i < 3; i++) 
	{
		ndVector dir(ndFloat32(0.0f));
		dir[i] = ndFloat32(-1.0f);
		p0[i] = SupportVertex(dir, nullptr)[i];

		dir[i] = ndFloat32(1.0f);
		p1[i] = SupportVertex(dir, nullptr)[i];
	}

	dAssert(p0.m_w == ndFloat32(0.0f));
	dAssert(p1.m_w == ndFloat32(0.0f));
	m_boxSize = (p1 - p0) * ndVector::m_half;
	m_boxOrigin = (p1 + p0) * ndVector::m_half;
	m_boxMinRadius = dMin(dMin(m_boxSize.m_x, m_boxSize.m_y), m_boxSize.m_z);
	m_boxMaxRadius = ndSqrt((m_boxSize.DotProduct(m_boxSize)).GetScalar());

	MassProperties();
}

void ndShapeConvex::MassProperties()
{
	ndFloat32 volume = CalculateMassProperties(dGetIdentityMatrix(), m_inertia, m_crossInertia, m_centerOfMass);
	if (volume < D_MAX_MIN_VOLUME) 
	{
		volume = D_MAX_MIN_VOLUME;
	}
	ndFloat32 invVolume = ndFloat32(1.0f) / volume;
	m_inertia = m_inertia.Scale(invVolume);
	m_crossInertia = m_crossInertia.Scale(invVolume);
	m_centerOfMass = m_centerOfMass.Scale(invVolume);
	m_centerOfMass.m_w = volume;
	
	// complete the calculation 
	ndShape::MassProperties();
}

ndFloat32 ndShapeConvex::CalculateMassProperties(const ndMatrix& offset, ndVector& inertia, ndVector& crossInertia, ndVector& centerOfMass) const
{
	class dMassPropertiesCalculator : public ndShapeDebugNotify
	{
		public:
		dMassPropertiesCalculator()
			:m_localData()
		{
		}

		virtual void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const)
		{
			m_localData.AddInertiaAndCrossFace(vertexCount, faceArray);
		}

		ndPolyhedraMassProperties m_localData;
	};
		
	dMassPropertiesCalculator massPropretiesCalculator;
		
	DebugShape(offset, massPropretiesCalculator);
	return massPropretiesCalculator.m_localData.MassProperties(centerOfMass, inertia, crossInertia);
}

ndMatrix ndShapeConvex::CalculateInertiaAndCenterOfMass(const ndMatrix& alignMatrix, const ndVector& localScale, const ndMatrix& matrix) const
{
	if ((dAbs(localScale.m_x - localScale.m_y) < ndFloat32(1.0e-5f)) && 
		(dAbs(localScale.m_x - localScale.m_z) < ndFloat32(1.0e-5f)) && 
		(dAbs(localScale.m_y - localScale.m_z) < ndFloat32(1.0e-5f)))
	{
		dAssert(alignMatrix.TestIdentity());
	
		// using general central theorem, is much faster and more accurate;
		//IImatrix = IIorigin + mass * [(displacemnet % displacemnet) * identityMatrix - transpose(displacement) * displacement)];
		ndFloat32 mag2 = localScale.m_x * localScale.m_x;
		ndMatrix inertia(dGetIdentityMatrix());
		inertia[0][0] = m_inertia[0] * mag2;
		inertia[1][1] = m_inertia[1] * mag2;
		inertia[2][2] = m_inertia[2] * mag2;
		inertia[0][1] = m_crossInertia[2] * mag2;
		inertia[1][0] = m_crossInertia[2] * mag2;
		inertia[0][2] = m_crossInertia[1] * mag2;
		inertia[2][0] = m_crossInertia[1] * mag2;
		inertia[1][2] = m_crossInertia[0] * mag2;
		inertia[2][1] = m_crossInertia[0] * mag2;
		inertia = matrix.Inverse() * inertia * matrix;
	
		dAssert(localScale.m_w == ndFloat32(0.0f));
		ndVector origin(matrix.TransformVector(m_centerOfMass * localScale));
	
		origin.m_w = ndFloat32(0.0f);
		ndFloat32 originMag2 = origin.DotProduct(origin).GetScalar();
		ndMatrix Covariance(origin, origin);
		ndMatrix parallel(dGetIdentityMatrix());
		for (ndInt32 i = 0; i < 3; i++) 
		{
			parallel[i][i] = originMag2;
			inertia[i] += (parallel[i] - Covariance[i]);
			dAssert(inertia[i][i] > ndFloat32(0.0f));
		}
	
		inertia.m_posit = origin;
		inertia.m_posit.m_w = ndFloat32 (1.0f);
		return inertia;
	}
	else 
	{
		// for non uniform scale we need to the general divergence theorem
		ndVector inertiaII;
		ndVector crossInertia;
		ndVector centerOfMass;
		ndMatrix scaledMatrix(matrix);
		scaledMatrix[0] = scaledMatrix[0].Scale(localScale.m_x);
		scaledMatrix[1] = scaledMatrix[1].Scale(localScale.m_y);
		scaledMatrix[2] = scaledMatrix[2].Scale(localScale.m_z);
		scaledMatrix = alignMatrix * scaledMatrix;
	
		ndFloat32 volume = CalculateMassProperties(scaledMatrix, inertiaII, crossInertia, centerOfMass);
		if (volume < D_MAX_MIN_VOLUME) {
			volume = D_MAX_MIN_VOLUME;
		}
	
		ndFloat32 invVolume = ndFloat32(1.0f) / volume;
		centerOfMass = centerOfMass.Scale(invVolume);
		inertiaII = inertiaII.Scale(invVolume);
		crossInertia = crossInertia.Scale(invVolume);
		ndMatrix inertia(dGetIdentityMatrix());
		inertia[0][0] = inertiaII[0];
		inertia[1][1] = inertiaII[1];
		inertia[2][2] = inertiaII[2];
		inertia[0][1] = crossInertia[2];
		inertia[1][0] = crossInertia[2];
		inertia[0][2] = crossInertia[1];
		inertia[2][0] = crossInertia[1];
		inertia[1][2] = crossInertia[0];
		inertia[2][1] = crossInertia[0];
		inertia[3] = centerOfMass;
		return inertia;
	}
}

void ndShapeConvex::CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const
{
	ndVector origin(matrix.TransformVector(m_boxOrigin));
	ndVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & ndVector::m_triplexMask;
	p1 = (origin + size) & ndVector::m_triplexMask;
}

ndFloat32 ndShapeConvex::RayCast(ndRayCastNotify&, const ndVector& localP0, const ndVector& localP1, ndFloat32, const ndBody* const body, ndContactPoint& contactOut) const
{
	ndBodyKinematic* const kinBody = ((ndBodyKinematic*)body)->GetAsBodyKinematic();
	ndShapeInstance tempInstance (kinBody->GetCollisionShape(), (ndShape*)this);
	ndContactNotify* const notify = kinBody->GetScene() ? kinBody->GetScene()->GetContactNotify() : nullptr;
	ndContactSolver rayCaster(&tempInstance, notify, ndFloat32 (1.0f), 0);
	return rayCaster.RayCast(localP0, localP1, contactOut);
}

ndVector ndShapeConvex::SupportVertex(const ndVector& dir, ndInt32* const) const
{
	dAssert(dir.m_w == ndFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-3f));

	ndInt16 cache[16];
	memset(cache, -1, sizeof(cache));
	ndConvexSimplexEdge* edge = &m_simplex[0];

	ndInt32 index = edge->m_vertex;
	ndFloat32 side0 = m_vertex[index].DotProduct(dir).GetScalar();

	cache[index & (sizeof(cache) / sizeof(cache[0]) - 1)] = ndInt16(index);
	ndConvexSimplexEdge* ptr = edge;
	ndInt32 maxCount = 128;
	do 
	{
		ndInt32 index1 = ptr->m_twin->m_vertex;
		if (cache[index1 & (sizeof(cache) / sizeof(cache[0]) - 1)] != index1) 
		{
			cache[index1 & (sizeof(cache) / sizeof(cache[0]) - 1)] = ndInt16(index1);
			ndFloat32 side1 = m_vertex[index1].DotProduct(dir).GetScalar();
			if (side1 > side0) 
			{
				index = index1;
				side0 = side1;
				edge = ptr->m_twin;
				ptr = edge;
			}
		}
		ptr = ptr->m_twin->m_next;
		maxCount--;
	} while ((ptr != edge) && maxCount);
	dAssert(maxCount);

	dAssert(index != -1);
	return m_vertex[index];
}

bool ndShapeConvex::SanityCheck(ndInt32 count, const ndVector& normal, ndVector* const contactsOut) const
{
	if (count > 1) 
	{
		ndInt32 j = count - 1;
		for (ndInt32 i = 0; i < count; i++) 
		{
			ndVector error(contactsOut[i] - contactsOut[j]);
			dAssert(error.m_w == ndFloat32(0.0f));
			if (error.DotProduct(error).GetScalar() <= ndFloat32(1.0e-20f)) 
			{
				return false;
			}
			j = i;
		}

		if (count >= 3) 
		{
			ndVector n(ndFloat32(0.0f));
			ndVector e0(contactsOut[1] - contactsOut[0]);
			for (ndInt32 i = 2; i < count; i++) 
			{
				ndVector e1(contactsOut[i] - contactsOut[0]);
				n += e0.CrossProduct(e1);
				e0 = e1;
			}
			dAssert(n.m_w == ndFloat32(0.0f));
			dAssert(n.DotProduct(n).GetScalar() > ndFloat32(0.0f));
			n = n.Scale(ndRsqrt(n.DotProduct(n).GetScalar()));
			ndFloat32 projection;
			projection = n.DotProduct(normal).GetScalar();
			dAssert(projection > ndFloat32(0.9f));
			if (projection < ndFloat32(0.9f)) 
			{
				return false;
			}

			e0 = contactsOut[count - 1] - contactsOut[count - 2];
			j = count - 1;
			for (ndInt32 i = 0; i < count; i++) 
			{
				ndVector e1(contactsOut[i] - contactsOut[j]);
				ndVector n1(e0.CrossProduct(e1));
				dAssert(n1.m_w == ndFloat32(0.0f));
				ndFloat32 error = n1.DotProduct(normal).GetScalar();
				dAssert(error >= ndFloat32(-1.0e-4f));
				if (error < ndFloat32(-1.0e-4f)) 
				{
					return false;
				}
				j = i;
				e0 = e1;
			}
		}
	}
	return true;
}

ndInt32 ndShapeConvex::RectifyConvexSlice(ndInt32 count, const ndVector& normal, ndVector* const contactsOut) const
{
	class ndConvexFaceNode
	{
		public:
		ndInt32 m_vertexIndex;
		ndConvexFaceNode* m_next;
		ndConvexFaceNode* m_prev;
		ndInt32 m_mask;
	};

	ndConvexFaceNode convexHull[D_MAX_CONTATCS + 1];
	char buffer[D_MAX_CONTATCS * (sizeof(void*) + sizeof(ndFloat32))];

	ndInt32 start = count;
	ndInt32 i0 = count - 1;
	for (ndInt32 i = 0; i < count; i++) 
	{
		convexHull[i].m_vertexIndex = i;
		convexHull[i].m_next = &convexHull[i + 1];
		convexHull[i].m_prev = &convexHull[i0];
		i0 = i;
	}
	convexHull[count - 1].m_next = &convexHull[0];

	ndVector hullArea(ndVector::m_zero);
	ndVector edge0(contactsOut[1] - contactsOut[0]);
	for (ndInt32 i = 2; i < count; i++) 
	{
		ndVector edge1(contactsOut[i] - contactsOut[0]);
		hullArea += edge1.CrossProduct(edge0);
		edge0 = edge1;
	}

	ndFloat32 totalArea = dAbs(hullArea.DotProduct(normal).GetScalar());
	if (totalArea < ndFloat32(1.0e-5f)) 
	{
		return 1;
	}
	ndConvexFaceNode* hullPoint = &convexHull[0];

	bool hasLinearCombination = true;
	ndUpHeap<ndConvexFaceNode*, ndFloat32> sortHeap(buffer, sizeof(buffer));
	while (hasLinearCombination) 
	{
		hasLinearCombination = false;
		sortHeap.Flush();
		ndConvexFaceNode* ptr = hullPoint;
		ndVector e0(contactsOut[ptr->m_next->m_vertexIndex] - contactsOut[ptr->m_vertexIndex]);
		do 
		{
			ndVector e1(contactsOut[ptr->m_next->m_next->m_vertexIndex] - contactsOut[ptr->m_next->m_vertexIndex]);
			ndFloat32 area = dAbs(e0.CrossProduct(e1).DotProduct(normal).GetScalar());
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (sortHeap.Value() * ndFloat32(32.0f) < totalArea)) 
		{
			ndConvexFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) 
			{
				if (hullPoint == corner) 
				{
					hullPoint = corner->m_prev;
				}
				count--;
				hasLinearCombination = true;
				corner->m_prev->m_mask = 0;
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}

	const ndInt32 maxVertexCount = D_MAX_VERTEX_CLIP_FACE;
	while (count > maxVertexCount) 
	{
		sortHeap.Flush();
		ndConvexFaceNode* ptr = hullPoint;
		ndVector e0(contactsOut[ptr->m_next->m_vertexIndex] - contactsOut[ptr->m_vertexIndex]);
		do 
		{
			ndVector e1(contactsOut[ptr->m_next->m_next->m_vertexIndex] - contactsOut[ptr->m_next->m_vertexIndex]);
			ndFloat32 area = dAbs(e0.CrossProduct(e1).DotProduct(normal).GetScalar());
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (count > maxVertexCount)) 
		{
			ndConvexFaceNode* const corner = sortHeap[0];
			if (corner->m_mask && corner->m_prev->m_mask) 
			{
				if (hullPoint == corner) 
				{
					hullPoint = corner->m_prev;
				}
				count--;
				corner->m_prev->m_mask = 0;
				corner->m_next->m_prev = corner->m_prev;
				corner->m_prev->m_next = corner->m_next;
			}
			sortHeap.Pop();
		}
	}

	ndInt32 index = start;
	ndConvexFaceNode* ptr = hullPoint;
	do 
	{
		contactsOut[index] = contactsOut[ptr->m_vertexIndex];
		index++;
		ptr = ptr->m_next;
	} while (ptr != hullPoint);
	memcpy(contactsOut, &contactsOut[start], count * sizeof(ndVector));

	dAssert(SanityCheck(count, normal, contactsOut));
	return count;
}

ndInt32 ndShapeConvex::CalculatePlaneIntersection(const ndVector& normal, const ndVector& origin, ndVector* const contactsOut) const
{
	ndVector support[4];
	ndInt32 featureCount = 3;
	const ndConvexSimplexEdge* edge = &m_simplex[0];
	const ndConvexSimplexEdge** const vertToEdgeMapping = GetVertexToEdgeMapping();
	dAssert(normal.m_w == ndFloat32(0.0f));
	if (vertToEdgeMapping) 
	{
		ndInt32 edgeIndex;
		featureCount = 1;
		support[0] = SupportVertex(normal, &edgeIndex);
		edge = vertToEdgeMapping[edgeIndex];

		// 5 degrees
		const ndFloat32 tiltAngle = ndFloat32(0.087f);
		const ndFloat32 tiltAngle2 = tiltAngle * tiltAngle;
		ndPlane testPlane(normal, -(normal.DotProduct(support[0]).GetScalar()));
		const ndConvexSimplexEdge* ptr = edge;
		do 
		{
			const ndVector& p = m_vertex[ptr->m_twin->m_vertex];
			ndFloat32 test = testPlane.Evalue(p);
			ndVector dist(p - support[0]);
			dAssert(dist.m_w == ndFloat32(0.0f));
			ndFloat32 angle2 = test * test / (dist.DotProduct(dist).GetScalar());

			if (angle2 < tiltAngle2) 
			{
				support[featureCount] = p;
				featureCount++;
			}
			ptr = ptr->m_twin->m_next;
		} while ((ptr != edge) && (featureCount < 3));
	}

	ndInt32 count = 0;
	ndPlane plane(normal, -normal.DotProduct(origin).GetScalar());
	switch (featureCount)
	{
		case 1:
			contactsOut[0] = support[0] - normal * normal.DotProduct(support[0] - origin);
			count = 1;
			break;

		case 2:
			contactsOut[0] = support[0] - normal * normal.DotProduct(support[0] - origin);
			contactsOut[1] = support[1] - normal * normal.DotProduct(support[1] - origin);
			count = 2;
			break;

		default:
		{
			ndFloat32 side0 = plane.Evalue(m_vertex[edge->m_vertex]);
			ndFloat32 side1 = side0;
			const ndConvexSimplexEdge* firstEdge = nullptr;
			if (side0 > ndFloat32(0.0f)) 
			{
				const ndConvexSimplexEdge* ptr = edge;
				do 
				{
					dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == ndFloat32(0.0f));
					side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
					if (side1 < side0) 
					{
						if (side1 < ndFloat32(0.0f)) 
						{
							firstEdge = ptr;
							break;
						}

						side0 = side1;
						edge = ptr->m_twin;
						ptr = edge;
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);


				if (!firstEdge) 
				{
					// we may have a local minimal in the convex hull do to a big flat face
					for (ndInt32 i = 0; i < m_edgeCount; i++) 
					{
						ptr = &m_simplex[i];
						side0 = plane.Evalue(m_vertex[ptr->m_vertex]);
						side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
						if ((side1 < ndFloat32(0.0f)) && (side0 > ndFloat32(0.0f))) 
						{
							firstEdge = ptr;
							break;
						}
					}
				}

			}
			else if (side0 < ndFloat32(0.0f)) 
			{
				const ndConvexSimplexEdge* ptr = edge;
				do 
				{
					dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == ndFloat32(0.0f));
					side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
					if (side1 > side0) 
					{
						if (side1 >= ndFloat32(0.0f)) 
						{
							side0 = side1;
							firstEdge = ptr->m_twin;
							break;
						}

						side0 = side1;
						edge = ptr->m_twin;
						ptr = edge;
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);

				if (!firstEdge) 
				{
					// we may have a local minimal in the convex hull due to a big flat face
					for (ndInt32 i = 0; i < m_edgeCount; i++) 
					{
						ptr = &m_simplex[i];
						side0 = plane.Evalue(m_vertex[ptr->m_vertex]);
						//ndFloat32 side1 = plane.Evalue (m_vertex[ptr->m_twin->m_vertex]);
						side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
						if ((side1 < ndFloat32(0.0f)) && (side0 > ndFloat32(0.0f))) 
						{
							firstEdge = ptr;
							break;
						}
					}
				}
			}

			if (firstEdge) 
			{
				dAssert(side0 >= ndFloat32(0.0f));
				dAssert((side1 = plane.Evalue(m_vertex[firstEdge->m_vertex])) >= ndFloat32(0.0f));
				dAssert((side1 = plane.Evalue(m_vertex[firstEdge->m_twin->m_vertex])) < ndFloat32(0.0f));
				dAssert(dAbs(side0 - plane.Evalue(m_vertex[firstEdge->m_vertex])) < ndFloat32(1.0e-5f));

				ndInt32 maxCount = 0;
				const ndConvexSimplexEdge* ptr = firstEdge;
				do 
				{
					if (side0 > ndFloat32(0.0f)) 
					{
						dAssert(plane.Evalue(m_vertex[ptr->m_vertex]) > ndFloat32(0.0f));
						dAssert(plane.Evalue(m_vertex[ptr->m_twin->m_vertex]) < ndFloat32(0.0f));

						ndVector dp(m_vertex[ptr->m_twin->m_vertex] - m_vertex[ptr->m_vertex]);
						dAssert(dp.m_w == ndFloat32(0.0f));
						ndFloat32 t = plane.DotProduct(dp).GetScalar();
						if (t >= ndFloat32(-1.e-24f)) 
						{
							t = ndFloat32(0.0f);
						}
						else 
						{
							t = side0 / t;
							if (t > ndFloat32(0.0f)) 
							{
								t = ndFloat32(0.0f);
							}
							if (t < ndFloat32(-1.0f)) 
							{
								t = ndFloat32(-1.0f);
							}
						}

						dAssert(t <= ndFloat32(0.01f));
						dAssert(t >= ndFloat32(-1.05f));
						contactsOut[count] = m_vertex[ptr->m_vertex] - dp.Scale(t);

						ndConvexSimplexEdge* ptr1 = ptr->m_next;
						for (; ptr1 != ptr; ptr1 = ptr1->m_next) 
						{
							dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == ndFloat32(0.0f));
							side0 = plane.Evalue(m_vertex[ptr1->m_twin->m_vertex]);
							if (side0 >= ndFloat32(0.0f)) 
							{
								break;
							}
						}
						dAssert(ptr1 != ptr);
						ptr = ptr1->m_twin;
					}
					else 
					{
						contactsOut[count] = m_vertex[ptr->m_vertex];
						ndConvexSimplexEdge* ptr1 = ptr->m_next;
						for (; ptr1 != ptr; ptr1 = ptr1->m_next) 
						{
							dAssert(m_vertex[ptr1->m_twin->m_vertex].m_w == ndFloat32(0.0f));
							side0 = plane.Evalue(m_vertex[ptr1->m_twin->m_vertex]);
							if (side0 >= ndFloat32(0.0f)) 
							{
								break;
							}
						}

						if (ptr1 == ptr) 
						{
							ptr = ptr1->m_prev->m_twin;
						}
						else 
						{
							ptr = ptr1->m_twin;
						}
					}

					count++;
					maxCount++;
					if (count >= D_CLIP_MAX_POINT_COUNT) 
					{
						for (count = 0; count < (D_CLIP_MAX_POINT_COUNT >> 1); count++) 
						{
							contactsOut[count] = contactsOut[count * 2];
						}
					}

				} while ((ptr != firstEdge) && (maxCount < D_CLIP_MAX_COUNT));
				dAssert(maxCount < D_CLIP_MAX_COUNT);

				if (count > 2) 
				{
					count = RectifyConvexSlice(count, normal, contactsOut);
				}
			}
		}
	}
	return count;
}

ndShapeInfo ndShapeConvex::GetShapeInfo() const
{
	ndShapeInfo info(ndShape::GetShapeInfo());
	return info;
}

ndFloat32 ndShapeConvex::GetVolume() const
{
	return m_centerOfMass.m_w;
}

ndFloat32 ndShapeConvex::GetBoxMinRadius() const
{
	return m_boxMinRadius;
}

ndFloat32 ndShapeConvex::GetBoxMaxRadius() const
{
	return m_boxMaxRadius;
}

bool ndShapeConvex::SanityCheck(ndPolyhedra& hull) const
{
	ndPolyhedra::Iterator iter(hull);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) 
		{
			return false;
		}
		ndEdge* ptr = edge;
		ndVector p0(m_vertex[edge->m_incidentVertex]);
		ptr = ptr->m_next;
		ndVector p1(m_vertex[ptr->m_incidentVertex]);
		ndVector e1(p1 - p0);
		ndVector n0(ndFloat32(0.0f));
		for (ptr = ptr->m_next; ptr != edge; ptr = ptr->m_next) 
		{
			ndVector p2(m_vertex[ptr->m_incidentVertex]);
			ndVector e2(p2 - p0);
			n0 += e1.CrossProduct(e2);
			e1 = e2;
		}

		dAssert(n0.m_w == ndFloat32(0.0f));
		ptr = edge;
		do 
		{
			ndVector q0(m_vertex[ptr->m_twin->m_incidentVertex]);
			for (ndEdge* neiborg = ptr->m_twin->m_next->m_next; neiborg != ptr->m_twin; neiborg = neiborg->m_next) 
			{
				ndVector q1(m_vertex[neiborg->m_incidentVertex]);
				ndVector q1q0(q1 - q0);
				ndFloat32 project = q1q0.DotProduct(n0).GetScalar();
				if (project > ndFloat32(1.0e-5f)) 
				{
					return false;
				}
			}

			ptr = ptr->m_next;
		} while (ptr != edge);
	}

	return true;
}

ndVector ndShapeConvex::CalculateVolumeIntegral(const ndPlane& plane) const
{
	ndInt8* const mark = dAlloca(ndInt8, m_edgeCount + 256);
	ndFloat32* test = dAlloca(ndFloat32, m_edgeCount + 256);
	ndVector* faceVertex = dAlloca(ndVector, m_edgeCount + 256);

	ndInt32 positive = 0;
	ndInt32 negative = 0;
	for (ndInt32 i = 0; i < m_vertexCount; i++) 
	{
		test[i] = plane.Evalue(m_vertex[i]);
		if (test[i] > ndFloat32(1.0e-5f)) 
		{
			positive++;
		}
		else if (test[i] < -ndFloat32(1.0e-5f)) 
		{
			negative++;
		}
		else 
		{
			test[i] = ndFloat32(0.0f);
		}
	}

	if (positive == m_vertexCount) 
	{
		return ndVector::m_zero;
	}

	if (negative == m_vertexCount) 
	{
		return m_centerOfMass;
	}

	ndPolyhedraMassProperties localData;
	ndConvexSimplexEdge* capEdge = nullptr;

	ndVector cg(ndVector::m_zero);
	memset(mark, 0, m_edgeCount);
	for (ndInt32 i = 0; i < m_edgeCount; i++) 
	{
		if (!mark[i]) 
		{
			ndConvexSimplexEdge* const face = &m_simplex[i];
			ndConvexSimplexEdge* edge = face;
			ndInt32 count = 0;
			ndFloat32 size0 = test[edge->m_prev->m_vertex];
			do 
			{
				mark[edge - m_simplex] = '1';
				ndFloat32 size1 = test[edge->m_vertex];
				if (size0 <= ndFloat32(0.0f)) 
				{
					faceVertex[count] = m_vertex[edge->m_prev->m_vertex];
					count++;
					if (size1 > ndFloat32(0.0f)) 
					{
						ndVector dp(m_vertex[edge->m_vertex] - m_vertex[edge->m_prev->m_vertex]);
						dAssert(dp.m_w == ndFloat32(0.0f));
						faceVertex[count] = m_vertex[edge->m_prev->m_vertex] - dp.Scale(size0 / dp.DotProduct(plane).GetScalar());
						count++;
					}
				}
				else if (size1 < ndFloat32(0.0f)) 
				{
					ndVector dp(m_vertex[edge->m_vertex] - m_vertex[edge->m_prev->m_vertex]);
					dAssert(dp.m_w == ndFloat32(0.0f));
					faceVertex[count] = m_vertex[edge->m_prev->m_vertex] - dp.Scale(size0 / dp.DotProduct(plane).GetScalar());
					count++;
					dAssert(count <= m_vertexCount);
				}

				if (!capEdge) 
				{
					if ((size1 > ndFloat32(0.0f)) && (size0 < ndFloat32(0.0f))) 
					{
						capEdge = edge->m_prev->m_twin;
					}
				}

				size0 = size1;
				edge = edge->m_next;
			} while (edge != face);

			if (count) 
			{
				localData.AddCGFace(count, faceVertex);
			}
		}
	}

	if (capEdge) 
	{
		ndInt32 count = 0;
		ndConvexSimplexEdge* edge = capEdge;
		ndConvexSimplexEdge* ptr = nullptr;
		do 
		{
			ndVector dp(m_vertex[edge->m_twin->m_vertex] - m_vertex[edge->m_vertex]);
			dAssert(dp.m_w == ndFloat32(0.0f));
			faceVertex[count] = m_vertex[edge->m_vertex] - dp.Scale(test[edge->m_vertex] / dp.DotProduct(plane).GetScalar());
			count++;
			if (count >= m_edgeCount)
			{
				dTrace(("%s something is wrong return zero\n", __FUNCTION__));
				return ndVector::m_zero;
			}

			for (ptr = edge->m_next; ptr != edge; ptr = ptr->m_next) 
			{
				ndInt32 index0 = ptr->m_twin->m_vertex;
				if (test[index0] > ndFloat32(0.0f)) 
				{
					index0 = ptr->m_vertex;
					if (test[index0] < ndFloat32(0.0f)) 
					{
						break;
					}
				}
			}
			edge = ptr->m_twin;
		} while (edge != capEdge);
		localData.AddCGFace(count, faceVertex);
	}

	ndVector inertia;
	ndVector crossInertia;
	ndFloat32 volume = localData.MassProperties(cg, inertia, crossInertia);
	cg = cg.Scale(ndFloat32(1.0f) / dMax(volume, ndFloat32(1.0e-6f)));
	cg.m_w = volume;
	return cg;
}

ndVector ndShapeConvex::CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& globalPlane, const ndShapeInstance& parentScale) const
{
	ndPlane localPlane(globalMatrix.UntransformPlane(globalPlane));

	const ndVector& scale = parentScale.m_scale;
	switch (parentScale.m_scaleType)
	{
		case ndShapeInstance::m_unit:
		{
			break;
		}

		case ndShapeInstance::m_uniform:
		{
			localPlane.m_w *= parentScale.m_invScale.m_x;
			break;
		}

		case ndShapeInstance::m_nonUniform:
		{
			localPlane = localPlane * (scale | ndVector::m_wOne);
			ndFloat32 mag2 = localPlane.DotProduct(localPlane & ndVector::m_triplexMask).GetScalar();
			localPlane = localPlane.Scale(ndRsqrt(mag2));
			break;
		}

		default:
		{
			localPlane = localPlane * (scale | ndVector::m_wOne);
			ndFloat32 mag2 = localPlane.DotProduct(localPlane & ndVector::m_triplexMask).GetScalar();
			localPlane = localPlane.Scale(ndRsqrt(mag2));
			localPlane = parentScale.m_aligmentMatrix.UntransformPlane(localPlane);
		}
	}

	ndVector cg(CalculateVolumeIntegral(localPlane));

	ndFloat32 volume = cg.m_w * scale.m_x * scale.m_y * scale.m_z;
	cg = parentScale.m_aligmentMatrix.RotateVector(cg);
	cg = cg * scale;
	cg = globalMatrix.TransformVector(cg);
	cg.m_w = volume;
	return cg;
}

//ndInt32 dgCollisionConvex::BuildCylinderCapPoly(ndFloat32 radius, const dMatrix& transform, ndVector* const vertexOut) const
ndInt32 ndShapeConvex::BuildCylinderCapPoly(ndFloat32 radius, const ndMatrix& transform, ndVector* const vertexOut) const
{
	/*
	ndFloat32 h = 2.0;
	ndInt32 n = 8;
	ndFloat32 a0 = h * h * (dgPi / n);

	ndFloat32 h0 = h * dgSin (0.5 * dgPI2 / n);
	ndFloat32 h1 = h * dgCos (0.5 * dgPI2 / n);
	ndFloat32 a1 = h * h * (dgSin (0.5 * dgPI2 / n) * dgCos (0.5 * dgPI2 / n));

	ndFloat32 a = h * h * (dgPi / n - 0.5f * dgSin (dgPI2 / n));

	for (ndInt32 i = 8; i < 16; i ++) {
	ndFloat32 den = dgPi / i - 0.5f * dgSin (dgPI2 / i);
	ndFloat32 h1 = dgSqrt (a / den);
	ndFloat32 h2 = dgSqrt (a / den);
	}
	*/

	ndInt32 count = (radius < ndFloat32(1.0f)) ? 8 : ((radius < ndFloat32(2.0f)) ? 12 : 16);

	ndFloat32 angle = ndFloat32 (2.0f) * ndPi / count;
	ndVector r(ndFloat32(0.0f), ndFloat32(0.0f), radius, ndFloat32(0.0f));
	ndMatrix rotation(dPitchMatrix(angle));

	for (ndInt32 i = 0; i < count; i++) 
	{
		vertexOut[i] = transform.TransformVector(r);
		r = rotation.RotateVector(r);
	}

	return count;
}

void ndShapeConvex::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShape::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	// maybe save some stuff here
}