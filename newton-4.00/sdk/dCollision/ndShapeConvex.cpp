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
#include "ndContact.h"
#include "ndShapeNull.h"
#include "ndContactSolver.h"
#include "ndBodyKinematic.h"

#define D_MAX_MIN_VOLUME				dFloat32 (1.0e-6f)
#define D_MAX_VERTEX_CLIP_FACE			16

#if 0
ndShapeConvex::ndShapeConvex (dgWorld* const world, dgDeserialize deserialization, void* const userData, dInt32 revisionNumber)
	:dShape (world, deserialization, userData, revisionNumber)
	,m_vertex (nullptr)
	,m_simplex (nullptr)
	,m_boxMinRadius (dFloat32 (0.0f))
	,m_boxMaxRadius (dFloat32 (0.0f))
	,m_simplexVolume (dFloat32 (0.0f))
	,m_edgeCount (0)
	,m_vertexCount (0)
{
	dAssert (m_rtti | ndShapeConvexShape_RTTI);
}

void ndShapeConvex::SerializeLow(dgSerialize callback, void* const userData) const
{
	dShape::SerializeLow(callback, userData);
}

dMatrix ndShapeConvex::CalculateInertiaAndCenterOfMass (const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const
{
	if ((dAbs (localScale.m_x - localScale.m_y) < dFloat32 (1.0e-5f)) && (dAbs (localScale.m_x - localScale.m_z) < dFloat32 (1.0e-5f))) {
		dAssert (alignMatrix[0][0] == dFloat32(1.0f));
		dAssert (alignMatrix[1][1] == dFloat32(1.0f));
		dAssert (alignMatrix[2][2] == dFloat32(1.0f));

		// using general central theorem, is much faster and more accurate;
		//IImatrix = IIorigin + mass * [(displacemnet % displacemnet) * identityMatrix - transpose(displacement) * displacement)];
		dFloat32 mag2 = localScale.m_x * localScale.m_x;
		dMatrix inertia (dGetIdentityMatrix());
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
		
		dAssert (localScale.m_w == dFloat32 (0.0f));
		dVector origin (matrix.TransformVector (m_centerOfMass * localScale));

		origin.m_w = dFloat32 (0.0f);
		dFloat32 originMag2 = origin.DotProduct(origin).GetScalar();
		dMatrix Covariance(origin, origin);
		dMatrix parallel(dGetIdentityMatrix());
		for (dInt32 i = 0; i < 3; i++) {
			parallel[i][i] = originMag2;
			inertia[i] += (parallel[i] - Covariance[i]);
			dAssert(inertia[i][i] > dFloat32(0.0f));
		}

		inertia.m_posit = origin;
		inertia.m_posit.m_w = 1.0f;
		return inertia;
	} else {
		// for non uniform scale we need to the general divergence theorem
		dVector inertiaII;
		dVector crossInertia;
		dVector centerOfMass;
		dMatrix scaledMatrix(matrix);
		scaledMatrix[0] = scaledMatrix[0].Scale(localScale.m_x);
		scaledMatrix[1] = scaledMatrix[1].Scale(localScale.m_y);
		scaledMatrix[2] = scaledMatrix[2].Scale(localScale.m_z);
		scaledMatrix = alignMatrix * scaledMatrix;

		dFloat32 volume = CalculateMassProperties (scaledMatrix, inertiaII, crossInertia, centerOfMass);
		if (volume < D_MAX_MIN_VOLUME) {
			volume = D_MAX_MIN_VOLUME;
		}

		dFloat32 invVolume = dFloat32 (1.0f) / volume;
		centerOfMass = centerOfMass.Scale(invVolume);
		inertiaII = inertiaII.Scale (invVolume);
		crossInertia = crossInertia.Scale (invVolume);
		dMatrix inertia (dGetIdentityMatrix());
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


dFloat32 ndShapeConvex::GetVolume () const
{
	return m_centerOfMass.m_w;
}

dVector ndShapeConvex::CalculateVolumeIntegral (const dMatrix& globalMatrix, const dVector& globalPlane, const dShapeInstance& parentScale) const
{
	dPlane localPlane (globalMatrix.UntransformPlane (globalPlane));

	const dVector& scale = parentScale.m_scale;
	switch (parentScale.m_scaleType)
	{
		case dShapeInstance::m_unit:
			break;

		case dShapeInstance::m_uniform:
		{
			localPlane.m_w *= parentScale.m_invScale.m_x;
			break;
		}

		case dShapeInstance::m_nonUniform:
		{
			localPlane = localPlane * (scale | dVector::m_wOne);
			dFloat32 mag2 = localPlane.DotProduct(localPlane & dVector::m_triplexMask).GetScalar();
			localPlane = localPlane.Scale (dRsqrt(mag2));
			break;
		}

		default:
		{
			localPlane = localPlane * (scale | dVector::m_wOne);
			dFloat32 mag2 = localPlane.DotProduct(localPlane & dVector::m_triplexMask).GetScalar();
			localPlane = localPlane.Scale (dRsqrt(mag2));
			localPlane = parentScale.m_aligmentMatrix.UntransformPlane (localPlane);
		}
	}

	dVector cg (CalculateVolumeIntegral (localPlane));
	
	dFloat32 volume = cg.m_w * scale.m_x * scale.m_y * scale.m_z;
	cg = parentScale.m_aligmentMatrix.RotateVector (cg);
	cg = cg * scale;
	cg = globalMatrix.TransformVector (cg);
	cg.m_w = volume;
	return cg;
}

dVector ndShapeConvex::CalculateVolumeIntegral (const dPlane& plane) const 
{
	dInt8 mark[D_MAX_EDGE_COUNT];
	dFloat32 test[D_MAX_EDGE_COUNT];
	dVector faceVertex[D_MAX_EDGE_COUNT];

	dInt32 positive = 0;
	dInt32 negative = 0;
	for (dInt32 i = 0; i < m_vertexCount; i ++) {
		test[i] = plane.Evalue (m_vertex[i]);
		if (test[i] > dFloat32 (1.0e-5f)) {
			positive ++;
		} else if (test[i] < -dFloat32 (1.0e-5f)) {
			negative ++;
		} else {
			test[i] = dFloat32 (0.0f);
		}
	}

	if (positive == m_vertexCount) {
		return dVector::m_zero;
	}

	if (negative == m_vertexCount) {
		return m_centerOfMass;
	}

	dPolyhedraMassProperties localData;
	dConvexSimplexEdge* capEdge = nullptr;

	dVector cg (dVector::m_zero); 
	memset (mark, 0, sizeof (mark));
	for (dInt32 i = 0; i < m_edgeCount; i ++) {
		if (!mark[i]) {
			dConvexSimplexEdge* const face = &m_simplex[i];
			dConvexSimplexEdge* edge = face;
			dInt32 count = 0;
			dFloat32 size0 = test[edge->m_prev->m_vertex];
			do {
				//edge->m_mark = m_mark;
				mark[edge - m_simplex] = '1';
				dFloat32 size1 = test[edge->m_vertex];
				if (size0 <= dFloat32 (0.0f)) {
					faceVertex[count] = m_vertex[edge->m_prev->m_vertex];
					count ++;
					if (size1 > dFloat32 (0.0f)) {
						dVector dp (m_vertex[edge->m_vertex] - m_vertex[edge->m_prev->m_vertex]);
						dAssert (dp.m_w == dFloat32 (0.0f));
						faceVertex[count] = m_vertex[edge->m_prev->m_vertex] - dp.Scale (size0 / dp.DotProduct(plane).GetScalar());
						count ++;
					}
				} else if (size1 < dFloat32 (0.0f)) {
					dVector dp (m_vertex[edge->m_vertex] - m_vertex[edge->m_prev->m_vertex]);
					dAssert (dp.m_w == dFloat32 (0.0f));
					faceVertex[count] = m_vertex[edge->m_prev->m_vertex] - dp.Scale (size0 / dp.DotProduct(plane).GetScalar());
					count ++;
					dAssert (count < dInt32 (sizeof (faceVertex) / sizeof (faceVertex[0])));
				}

				if (!capEdge) {
					if ((size1 > dFloat32 (0.0f)) && (size0 < dFloat32 (0.0f))) {
						capEdge = edge->m_prev->m_twin;
					}
				}

				size0 = size1;
				edge = edge->m_next;
			} while (edge != face);

			if (count) {
				localData.AddCGFace(count, faceVertex);
			}
		}
	}

	if (capEdge) {
		dInt32 count = 0;
		dConvexSimplexEdge* edge = capEdge;
		dConvexSimplexEdge* ptr = nullptr;
		do {
			dVector dp (m_vertex[edge->m_twin->m_vertex] - m_vertex[edge->m_vertex]);
			dAssert (dp.m_w == dFloat32 (0.0f));
			faceVertex[count] = m_vertex[edge->m_vertex] - dp.Scale (test[edge->m_vertex] / dp.DotProduct(plane).GetScalar());
			count ++;
			if (count == 127) {
				// something is wrong return zero
				return dVector (dFloat32 (0.0f));
			}

			for (ptr = edge->m_next; ptr != edge; ptr = ptr->m_next) {
				dInt32 index0 = ptr->m_twin->m_vertex;
				if (test[index0] > dFloat32 (0.0f)) {
					index0 = ptr->m_vertex;
					if (test[index0] < dFloat32 (0.0f)) {
						break;
					}
				}
			}
			edge = ptr->m_twin;
		} while (edge != capEdge);
		localData.AddCGFace(count, faceVertex);
	}

	dVector inertia;
	dVector crossInertia;
	dFloat32 volume = localData.MassProperties (cg, inertia, crossInertia);
	cg = cg.Scale (dFloat32 (1.0f) / dMax (volume, dFloat32 (1.0e-6f)));
	cg.m_w = volume;
	return cg; 
}





dVector ndShapeConvex::SupportVertexSpecial (const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const 
{
	return SupportVertex(dir, vertexIndex);
}

dVector ndShapeConvex::SupportVertexSpecialProjectPoint (const dVector& point, const dVector& dir) const
 {
	return point;
 }

dInt32 ndShapeConvex::BuildCylinderCapPoly (dFloat32 radius, const dMatrix& transform, dVector* const vertexOut) const
{
/*
	dFloat32 h = 2.0;
	dInt32 n = 8;
	dFloat32 a0 = h * h * (dPi / n);

	dFloat32 h0 = h * dSin (0.5 * dPI2 / n);
	dFloat32 h1 = h * dCos (0.5 * dPI2 / n);
	dFloat32 a1 = h * h * (dSin (0.5 * dPI2 / n) * dCos (0.5 * dPI2 / n));

	dFloat32 a = h * h * (dPi / n - 0.5f * dSin (dPI2 / n));

	for (dInt32 i = 8; i < 16; i ++) {
		dFloat32 den = dPi / i - 0.5f * dSin (dPI2 / i);
		dFloat32 h1 = dSqrt (a / den);
		dFloat32 h2 = dSqrt (a / den);
	}
*/

	dInt32 count = (radius < dFloat32 (1.0f)) ? 8 : ((radius < dFloat32 (2.0f)) ? 12 : 16);

	dFloat32 angle = dPI2 / count;
	dVector r (dFloat32 (0.0f), dFloat32 (0.0f), radius, dFloat32 (0.0f));
	dMatrix rotation (dPitchMatrix(angle));

	for (dInt32 i = 0; i < count; i++) {
		vertexOut[i] = transform.TransformVector(r); 
		r = rotation.RotateVector(r);
	}

	return count;
}

#endif

ndShapeConvex::ndShapeConvex(ndShapeID id)
	:ndShape(id)
	,m_vertex(nullptr)
	,m_simplex(nullptr)
	,m_boxMinRadius(dFloat32(0.0f))
	,m_boxMaxRadius(dFloat32(0.0f))
	,m_simplexVolume(dFloat32(0.0f))
	,m_edgeCount(0)
	,m_vertexCount(0)
{
}

ndShapeConvex::~ndShapeConvex()
{
	if (m_vertex) 
	{
		dMemory::Free(m_vertex);
	}
	
	if (m_simplex) 
	{
		dMemory::Free(m_simplex);
	}
}

void ndShapeConvex::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
{
	dInt8 mark[D_MAX_EDGE_COUNT];
	dVector tmp[D_MAX_EDGE_COUNT];
	dVector vertex[D_MAX_EDGE_COUNT];

	matrix.TransformTriplex(&tmp[0].m_x, sizeof(dVector), &m_vertex[0].m_x, sizeof(dVector), m_vertexCount);

	memset(mark, 0, sizeof(mark));
	for (dInt32 i = 0; i < m_edgeCount; i++) 
	{
		if (!mark[i]) 
		{
			ndConvexSimplexEdge* const face = &m_simplex[i];
			ndConvexSimplexEdge* edge = face;
			dInt32 count = 0;
			do 
			{
				mark[edge - m_simplex] = '1';
				dInt32 index = edge->m_vertex;
				vertex[count] = tmp[index] & dVector::m_triplexMask;
				count++;
				edge = edge->m_next;
			} while (edge != face);
			debugCallback.DrawPolygon(count, vertex);
		}
	}
}

void ndShapeConvex::SetVolumeAndCG()
{
	dVector faceVertex[D_MAX_EDGE_COUNT];
	dInt8* const edgeMarks = dAlloca(dInt8, m_edgeCount + 32);
	memset(&edgeMarks[0], 0, sizeof(dInt8) * m_edgeCount);

	dPolyhedraMassProperties localData;
	for (dInt32 i = 0; i < m_edgeCount; i++) 
	{
		ndConvexSimplexEdge* const face = &m_simplex[i];
		if (!edgeMarks[i]) {
			ndConvexSimplexEdge* edge = face;
			dInt32 count = 0;
			do 
			{
				dAssert((edge - m_simplex) >= 0);
				edgeMarks[dInt32(edge - m_simplex)] = '1';
				faceVertex[count] = m_vertex[edge->m_vertex];
				count++;
				dAssert(count < dInt32(sizeof(faceVertex) / sizeof(faceVertex[0])));
				edge = edge->m_next;
			} while (edge != face);

			localData.AddCGFace(count, faceVertex);
		}
	}

	dVector origin;
	dVector inertia;
	dVector crossInertia;
	dFloat32 volume = localData.MassProperties(origin, inertia, crossInertia);
	m_simplexVolume = volume;

	// calculate the origin of the bound box of this primitive
	dVector p0(dVector::m_zero);
	dVector p1(dVector::m_zero);

	for (dInt32 i = 0; i < 3; i++) 
	{
		dVector dir(dFloat32(0.0f));
		dir[i] = dFloat32(-1.0f);
		p0[i] = SupportVertex(dir, nullptr)[i];

		dir[i] = dFloat32(1.0f);
		p1[i] = SupportVertex(dir, nullptr)[i];
	}

	dAssert(p0.m_w == dFloat32(0.0f));
	dAssert(p1.m_w == dFloat32(0.0f));
	m_boxSize = (p1 - p0) * dVector::m_half;
	m_boxOrigin = (p1 + p0) * dVector::m_half;
	m_boxMinRadius = dMin(m_boxSize.m_x, m_boxSize.m_y, m_boxSize.m_z);
	m_boxMaxRadius = dSqrt((m_boxSize.DotProduct(m_boxSize)).GetScalar());

	MassProperties();
}

void ndShapeConvex::MassProperties()
{
	dFloat32 volume = CalculateMassProperties(dGetIdentityMatrix(), m_inertia, m_crossInertia, m_centerOfMass);
	if (volume < D_MAX_MIN_VOLUME) 
	{
		volume = D_MAX_MIN_VOLUME;
	}
	dFloat32 invVolume = dFloat32(1.0f) / volume;
	m_inertia = m_inertia.Scale(invVolume);
	m_crossInertia = m_crossInertia.Scale(invVolume);
	m_centerOfMass = m_centerOfMass.Scale(invVolume);
	m_centerOfMass.m_w = volume;
	
	// complete the calculation 
	ndShape::MassProperties();
}

dFloat32 ndShapeConvex::CalculateMassProperties(const dMatrix& offset, dVector& inertia, dVector& crossInertia, dVector& centerOfMass) const
{
	class dMassPropertiesCalculator : public ndShapeDebugCallback
	{
		public:
		dMassPropertiesCalculator(const ndShapeConvex* const shape)
			:m_localData()
			,m_me(shape)
		{
		}

		virtual void DrawPolygon(dInt32 vertexCount, const dVector* const faceArray)
		{
			m_localData.AddInertiaAndCrossFace(vertexCount, faceArray);
		}

		dPolyhedraMassProperties m_localData;
		const ndShapeConvex* m_me;
	};
		
	dMassPropertiesCalculator massPropretiesCalculator(this);
		
	DebugShape(offset, massPropretiesCalculator);
	return massPropretiesCalculator.m_localData.MassProperties(centerOfMass, inertia, crossInertia);
}

dMatrix ndShapeConvex::CalculateInertiaAndCenterOfMass(const dMatrix& alignMatrix, const dVector& localScale, const dMatrix& matrix) const
{
	if ((dAbs(localScale.m_x - localScale.m_y) < dFloat32(1.0e-5f)) && 
		(dAbs(localScale.m_x - localScale.m_z) < dFloat32(1.0e-5f)) && 
		(dAbs(localScale.m_y - localScale.m_z) < dFloat32(1.0e-5f)))
	{
		dAssert(alignMatrix.TestIdentity());
	
		// using general central theorem, is much faster and more accurate;
		//IImatrix = IIorigin + mass * [(displacemnet % displacemnet) * identityMatrix - transpose(displacement) * displacement)];
		dFloat32 mag2 = localScale.m_x * localScale.m_x;
		dMatrix inertia(dGetIdentityMatrix());
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
	
		dAssert(localScale.m_w == dFloat32(0.0f));
		dVector origin(matrix.TransformVector(m_centerOfMass * localScale));
	
		origin.m_w = dFloat32(0.0f);
		dFloat32 originMag2 = origin.DotProduct(origin).GetScalar();
		dMatrix Covariance(origin, origin);
		dMatrix parallel(dGetIdentityMatrix());
		for (dInt32 i = 0; i < 3; i++) 
		{
			parallel[i][i] = originMag2;
			inertia[i] += (parallel[i] - Covariance[i]);
			dAssert(inertia[i][i] > dFloat32(0.0f));
		}
	
		inertia.m_posit = origin;
		inertia.m_posit.m_w = 1.0f;
		return inertia;
	}
	else 
	{
		// for non uniform scale we need to the general divergence theorem
		dVector inertiaII;
		dVector crossInertia;
		dVector centerOfMass;
		dMatrix scaledMatrix(matrix);
		scaledMatrix[0] = scaledMatrix[0].Scale(localScale.m_x);
		scaledMatrix[1] = scaledMatrix[1].Scale(localScale.m_y);
		scaledMatrix[2] = scaledMatrix[2].Scale(localScale.m_z);
		scaledMatrix = alignMatrix * scaledMatrix;
	
		dFloat32 volume = CalculateMassProperties(scaledMatrix, inertiaII, crossInertia, centerOfMass);
		if (volume < D_MAX_MIN_VOLUME) {
			volume = D_MAX_MIN_VOLUME;
		}
	
		dFloat32 invVolume = dFloat32(1.0f) / volume;
		centerOfMass = centerOfMass.Scale(invVolume);
		inertiaII = inertiaII.Scale(invVolume);
		crossInertia = crossInertia.Scale(invVolume);
		dMatrix inertia(dGetIdentityMatrix());
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

void ndShapeConvex::CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
{
	dVector origin(matrix.TransformVector(m_boxOrigin));
	dVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & dVector::m_triplexMask;
	p1 = (origin + size) & dVector::m_triplexMask;
}

dFloat32 ndShapeConvex::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
{
	dAssert(0);
	return 0;
	//ndBodyKinematic* const kinBody = ((ndBodyKinematic*)body)->GetAsBodyKinematic();
	//ndShapeInstance instance(kinBody->GetCollisionShape());
	//ndContactSolver rayCaster(&instance);
	//instance.m_material.m_userData = nullptr;
	//return rayCaster.RayCast(localP0, localP1, maxT, contactOut);
}

dVector ndShapeConvex::SupportVertex(const dVector& dir, dInt32* const vertexIndex) const
{
	dAssert(dir.m_w == dFloat32(0.0f));
	dAssert(dAbs(dir.DotProduct(dir).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-3f));

	dInt16 cache[16];
	memset(cache, -1, sizeof(cache));
	ndConvexSimplexEdge* edge = &m_simplex[0];

	dInt32 index = edge->m_vertex;
	dFloat32 side0 = m_vertex[index].DotProduct(dir).GetScalar();

	cache[index & (sizeof(cache) / sizeof(cache[0]) - 1)] = dInt16(index);
	ndConvexSimplexEdge* ptr = edge;
	dInt32 maxCount = 128;
	do 
	{
		dInt32 index1 = ptr->m_twin->m_vertex;
		if (cache[index1 & (sizeof(cache) / sizeof(cache[0]) - 1)] != index1) 
		{
			cache[index1 & (sizeof(cache) / sizeof(cache[0]) - 1)] = dInt16(index1);
			dFloat32 side1 = m_vertex[index1].DotProduct(dir).GetScalar();
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

bool ndShapeConvex::SanityCheck(dInt32 count, const dVector& normal, dVector* const contactsOut) const
{
	if (count > 1) 
	{
		dInt32 j = count - 1;
		for (dInt32 i = 0; i < count; i++) 
		{
			dVector error(contactsOut[i] - contactsOut[j]);
			dAssert(error.m_w == dFloat32(0.0f));
			if (error.DotProduct(error).GetScalar() <= dFloat32(1.0e-20f)) 
			{
				return false;
			}
			j = i;
		}

		if (count >= 3) 
		{
			dVector n(dFloat32(0.0f));
			dVector e0(contactsOut[1] - contactsOut[0]);
			for (dInt32 i = 2; i < count; i++) 
			{
				dVector e1(contactsOut[i] - contactsOut[0]);
				n += e0.CrossProduct(e1);
				e0 = e1;
			}
			dAssert(n.m_w == dFloat32(0.0f));
			dAssert(n.DotProduct(n).GetScalar() > dFloat32(0.0f));
			n = n.Scale(dRsqrt(n.DotProduct(n).GetScalar()));
			dFloat32 projection;
			projection = n.DotProduct(normal).GetScalar();
			dAssert(projection > dFloat32(0.9f));
			if (projection < dFloat32(0.9f)) 
			{
				return false;
			}

			e0 = contactsOut[count - 1] - contactsOut[count - 2];
			j = count - 1;
			for (dInt32 i = 0; i < count; i++) 
			{
				dVector e1(contactsOut[i] - contactsOut[j]);
				dVector n1(e0.CrossProduct(e1));
				dAssert(n1.m_w == dFloat32(0.0f));
				dFloat32 error = n1.DotProduct(normal).GetScalar();
				dAssert(error >= dFloat32(-1.0e-4f));
				if (error < dFloat32(-1.0e-4f)) 
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

dInt32 ndShapeConvex::RectifyConvexSlice(dInt32 count, const dVector& normal, dVector* const contactsOut) const
{
	class ndConvexFaceNode
	{
		public:
		dInt32 m_vertexIndex;
		ndConvexFaceNode* m_next;
		ndConvexFaceNode* m_prev;
		dInt32 m_mask;
	};

	ndConvexFaceNode convexHull[D_MAX_CONTATCS + 1];
	char buffer[D_MAX_CONTATCS * (sizeof(void*) + sizeof(dFloat32))];

	dInt32 start = count;
	dInt32 i0 = count - 1;
	for (dInt32 i = 0; i < count; i++) 
	{
		convexHull[i].m_vertexIndex = i;
		convexHull[i].m_next = &convexHull[i + 1];
		convexHull[i].m_prev = &convexHull[i0];
		i0 = i;
	}
	convexHull[count - 1].m_next = &convexHull[0];

	dVector hullArea(dVector::m_zero);
	dVector edge0(contactsOut[1] - contactsOut[0]);
	for (dInt32 i = 2; i < count; i++) 
	{
		dVector edge1(contactsOut[i] - contactsOut[0]);
		hullArea += edge1.CrossProduct(edge0);
		edge0 = edge1;
	}

	dFloat32 totalArea = dAbs(hullArea.DotProduct(normal).GetScalar());
	if (totalArea < dFloat32(1.0e-5f)) 
	{
		return 1;
	}
	ndConvexFaceNode* hullPoint = &convexHull[0];

	bool hasLinearCombination = true;
	dUpHeap<ndConvexFaceNode*, dFloat32> sortHeap(buffer, sizeof(buffer));
	while (hasLinearCombination) 
	{
		hasLinearCombination = false;
		sortHeap.Flush();
		ndConvexFaceNode* ptr = hullPoint;
		dVector e0(contactsOut[ptr->m_next->m_vertexIndex] - contactsOut[ptr->m_vertexIndex]);
		do 
		{
			dVector e1(contactsOut[ptr->m_next->m_next->m_vertexIndex] - contactsOut[ptr->m_next->m_vertexIndex]);
			dFloat32 area = dAbs(e0.CrossProduct(e1).DotProduct(normal).GetScalar());
			sortHeap.Push(ptr->m_next, area);
			e0 = e1;
			ptr->m_mask = 1;
			ptr = ptr->m_next;
		} while (ptr != hullPoint);

		while (sortHeap.GetCount() && (sortHeap.Value() * dFloat32(32.0f) < totalArea)) 
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

	const dInt32 maxVertexCount = D_MAX_VERTEX_CLIP_FACE;
	while (count > maxVertexCount) 
	{
		sortHeap.Flush();
		ndConvexFaceNode* ptr = hullPoint;
		dVector e0(contactsOut[ptr->m_next->m_vertexIndex] - contactsOut[ptr->m_vertexIndex]);
		do 
		{
			dVector e1(contactsOut[ptr->m_next->m_next->m_vertexIndex] - contactsOut[ptr->m_next->m_vertexIndex]);
			dFloat32 area = dAbs(e0.CrossProduct(e1).DotProduct(normal).GetScalar());
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

	dInt32 index = start;
	ndConvexFaceNode* ptr = hullPoint;
	do 
	{
		contactsOut[index] = contactsOut[ptr->m_vertexIndex];
		index++;
		ptr = ptr->m_next;
	} while (ptr != hullPoint);
	memcpy(contactsOut, &contactsOut[start], count * sizeof(dVector));

	dAssert(SanityCheck(count, normal, contactsOut));
	return count;
}

dInt32 ndShapeConvex::CalculatePlaneIntersection(const dVector& normal, const dVector& origin, dVector* const contactsOut) const
{
	dVector support[4];
	dInt32 featureCount = 3;
	const ndConvexSimplexEdge* edge = &m_simplex[0];
	const ndConvexSimplexEdge** const vertToEdgeMapping = GetVertexToEdgeMapping();
	dAssert(normal.m_w == dFloat32(0.0f));
	if (vertToEdgeMapping) 
	{
		dInt32 edgeIndex;
		featureCount = 1;
		support[0] = SupportVertex(normal, &edgeIndex);
		edge = vertToEdgeMapping[edgeIndex];

		// 5 degrees
		const dFloat32 tiltAngle = dFloat32(0.087f);
		const dFloat32 tiltAngle2 = tiltAngle * tiltAngle;
		dPlane testPlane(normal, -(normal.DotProduct(support[0]).GetScalar()));
		const ndConvexSimplexEdge* ptr = edge;
		do 
		{
			const dVector& p = m_vertex[ptr->m_twin->m_vertex];
			dFloat32 test = testPlane.Evalue(p);
			dVector dist(p - support[0]);
			dAssert(dist.m_w == dFloat32(0.0f));
			dFloat32 angle2 = test * test / (dist.DotProduct(dist).GetScalar());

			if (angle2 < tiltAngle2) 
			{
				support[featureCount] = p;
				featureCount++;
			}
			ptr = ptr->m_twin->m_next;
		} while ((ptr != edge) && (featureCount < 3));
	}

	dInt32 count = 0;
	dPlane plane(normal, -normal.DotProduct(origin).GetScalar());
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
			dFloat32 side0 = plane.Evalue(m_vertex[edge->m_vertex]);
			dFloat32 side1 = side0;
			const ndConvexSimplexEdge* firstEdge = nullptr;
			if (side0 > dFloat32(0.0f)) 
			{
				const ndConvexSimplexEdge* ptr = edge;
				do 
				{
					dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == dFloat32(0.0f));
					side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
					if (side1 < side0) 
					{
						if (side1 < dFloat32(0.0f)) 
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
					for (dInt32 i = 0; i < m_edgeCount; i++) 
					{
						ptr = &m_simplex[i];
						side0 = plane.Evalue(m_vertex[ptr->m_vertex]);
						side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
						if ((side1 < dFloat32(0.0f)) && (side0 > dFloat32(0.0f))) 
						{
							firstEdge = ptr;
							break;
						}
					}
				}

			}
			else if (side0 < dFloat32(0.0f)) 
			{
				const ndConvexSimplexEdge* ptr = edge;
				do 
				{
					dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == dFloat32(0.0f));
					side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
					if (side1 > side0) 
					{
						if (side1 >= dFloat32(0.0f)) 
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
					for (dInt32 i = 0; i < m_edgeCount; i++) 
					{
						ptr = &m_simplex[i];
						side0 = plane.Evalue(m_vertex[ptr->m_vertex]);
						//dFloat32 side1 = plane.Evalue (m_vertex[ptr->m_twin->m_vertex]);
						side1 = plane.Evalue(m_vertex[ptr->m_twin->m_vertex]);
						if ((side1 < dFloat32(0.0f)) && (side0 > dFloat32(0.0f))) 
						{
							firstEdge = ptr;
							break;
						}
					}
				}
			}

			if (firstEdge) 
			{
				dAssert(side0 >= dFloat32(0.0f));
				dAssert((side1 = plane.Evalue(m_vertex[firstEdge->m_vertex])) >= dFloat32(0.0f));
				dAssert((side1 = plane.Evalue(m_vertex[firstEdge->m_twin->m_vertex])) < dFloat32(0.0f));
				dAssert(dAbs(side0 - plane.Evalue(m_vertex[firstEdge->m_vertex])) < dFloat32(1.0e-5f));

				dInt32 maxCount = 0;
				const ndConvexSimplexEdge* ptr = firstEdge;
				do 
				{
					if (side0 > dFloat32(0.0f)) 
					{
						dAssert(plane.Evalue(m_vertex[ptr->m_vertex]) > dFloat32(0.0f));
						dAssert(plane.Evalue(m_vertex[ptr->m_twin->m_vertex]) < dFloat32(0.0f));

						dVector dp(m_vertex[ptr->m_twin->m_vertex] - m_vertex[ptr->m_vertex]);
						dAssert(dp.m_w == dFloat32(0.0f));
						dFloat32 t = plane.DotProduct(dp).GetScalar();
						if (t >= dFloat32(-1.e-24f)) 
						{
							t = dFloat32(0.0f);
						}
						else 
						{
							t = side0 / t;
							if (t > dFloat32(0.0f)) 
							{
								t = dFloat32(0.0f);
							}
							if (t < dFloat32(-1.0f)) 
							{
								t = dFloat32(-1.0f);
							}
						}

						dAssert(t <= dFloat32(0.01f));
						dAssert(t >= dFloat32(-1.05f));
						contactsOut[count] = m_vertex[ptr->m_vertex] - dp.Scale(t);

						ndConvexSimplexEdge* ptr1 = ptr->m_next;
						for (; ptr1 != ptr; ptr1 = ptr1->m_next) 
						{
							dAssert(m_vertex[ptr->m_twin->m_vertex].m_w == dFloat32(0.0f));
							side0 = plane.Evalue(m_vertex[ptr1->m_twin->m_vertex]);
							if (side0 >= dFloat32(0.0f)) 
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
							dAssert(m_vertex[ptr1->m_twin->m_vertex].m_w == dFloat32(0.0f));
							side0 = plane.Evalue(m_vertex[ptr1->m_twin->m_vertex]);
							if (side0 >= dFloat32(0.0f)) 
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

dFloat32 ndShapeConvex::GetBoxMinRadius() const
{
	return m_boxMinRadius;
}

dFloat32 ndShapeConvex::GetBoxMaxRadius() const
{
	return m_boxMaxRadius;
}

bool ndShapeConvex::SanityCheck(dPolyhedra& hull) const
{
	dPolyhedra::Iterator iter(hull);
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) 
		{
			return false;
		}
		dEdge* ptr = edge;
		dVector p0(m_vertex[edge->m_incidentVertex]);
		ptr = ptr->m_next;
		dVector p1(m_vertex[ptr->m_incidentVertex]);
		dVector e1(p1 - p0);
		dVector n0(dFloat32(0.0f));
		for (ptr = ptr->m_next; ptr != edge; ptr = ptr->m_next) 
		{
			dVector p2(m_vertex[ptr->m_incidentVertex]);
			dVector e2(p2 - p0);
			n0 += e1.CrossProduct(e2);
			e1 = e2;
		}

		dAssert(n0.m_w == dFloat32(0.0f));
		ptr = edge;
		do 
		{
			dVector q0(m_vertex[ptr->m_twin->m_incidentVertex]);
			for (dEdge* neiborg = ptr->m_twin->m_next->m_next; neiborg != ptr->m_twin; neiborg = neiborg->m_next) 
			{
				dVector q1(m_vertex[neiborg->m_incidentVertex]);
				dVector q1q0(q1 - q0);
				dFloat32 project = q1q0.DotProduct(n0).GetScalar();
				if (project > dFloat32(1.0e-5f)) 
				{
					return false;
				}
			}

			ptr = ptr->m_next;
		} while (ptr != edge);
	}

	return true;
}
