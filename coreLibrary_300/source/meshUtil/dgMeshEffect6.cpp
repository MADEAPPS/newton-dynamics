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

// algorithm from paper: Non-Distorted Texture Mapping Using Angle Based Flattening
// by: A. Sheffer and E. de Sturler
// http://www.math.vt.edu/people/sturler/Publications/UIUCDCS-R-2001-2257.pdf
// 
// also improvement from paper: ABF++: Fast and Robust Angle Based Flattening
// http://hal.archives-ouvertes.fr/docs/00/10/56/89/PDF/abf_plus_plus_temp.pdf
// by: Alla Sheffer, Bruno Lévy, Inria Lorraine, Maxim Mogilnitsky and Alexander Bogomyakov

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"
#include "dgMeshEffect.h"

#define dgABF_MAX_ITERATIONS	10
#define dgABF_TOL				dgFloat64 (1.0e-3f)
#define dgABF_TOL2				dgABF_TOL * dgABF_TOL
#define dgABF_PI				dgFloat64 (3.1415926535)


class TestSolver_xxxxxxx: public SymmetricBiconjugateGradientSolve
{
	public:
	dgFloat64 a[4][4];

	TestSolver_xxxxxxx()
		:SymmetricBiconjugateGradientSolve()
	{
		dgFloat64 b[] = {1, 2, 3, 4};
		dgFloat64 x[] = {0, 0, 0, 0};
		dgFloat64 c[4];

		memset (a, 0, sizeof (a));
		a[0][0] = 2;
		a[1][1] = 3;
		a[2][2] = 4;
		a[0][3] = 1;
		a[1][3] = 1;
		a[2][3] = 1;
		a[3][0] = 1;
		a[3][1] = 1;
		a[3][2] = 1;


		Solve (4, 4, dgFloat64  (1.0e-10f), x, b);

		MatrixTimeVector (c, x);
		MatrixTimeVector (c, x);
	}

	void MatrixTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		out[0] = a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2] + a[0][3] * v[3];
		out[1] = a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2] + a[1][3] * v[3];
		out[2] = a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2] + a[2][3] * v[3];
		out[3] = a[3][0] * v[0] + a[3][1] * v[1] + a[3][2] * v[2] + a[3][3] * v[3];
	}

	void InversePrecoditionerTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		out[0] = v[0]/a[0][0];
		out[1] = v[1]/a[1][1];
		out[2] = v[2]/a[2][2];
		out[3] = v[3];
	}
};



class dgAngleBasedFlatteningMapping: public SymmetricBiconjugateGradientSolve
{
	public: 
	dgAngleBasedFlatteningMapping (dgMeshEffect* const mesh, dgInt32 material)
		:m_mesh(mesh)
		,m_material(material)
	{
		m_mesh->Triangulate();
		CalculateNumberOfVariables();

		dgInt32 vertexCount = m_mesh->GetVertexCount();
		m_betaEdge = (dgEdge**) m_mesh->GetAllocator()->MallocLow(m_anglesCount * sizeof (dgEdge*));
		m_interiorIndirectMap = (dgInt32*) m_mesh->GetAllocator()->MallocLow (vertexCount * sizeof (dgInt32));

		m_beta = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_weight= (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));

		m_sinTable = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_cosTable = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_variables = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_totalVariablesCount * sizeof (dgFloat64));
		m_gradients = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_totalVariablesCount * sizeof (dgFloat64));
		m_deltaVariable = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_totalVariablesCount * sizeof (dgFloat64));
		m_uv = (dgTriplex*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgTriplex));

		InitEdgeVector();
		CalculateInitialAngles ();

		// if there are no interior vertex then the mesh can be projected to a flat plane
		LagrangeOptimization();
//		GenerateUVCoordinates ();

	}

	~dgAngleBasedFlatteningMapping()
	{
		m_mesh->GetAllocator()->FreeLow (m_betaEdge);
		m_mesh->GetAllocator()->FreeLow (m_interiorIndirectMap);
		m_mesh->GetAllocator()->FreeLow (m_sinTable);
		m_mesh->GetAllocator()->FreeLow (m_cosTable);
		m_mesh->GetAllocator()->FreeLow (m_beta);
		m_mesh->GetAllocator()->FreeLow (m_weight);
		m_mesh->GetAllocator()->FreeLow (m_variables);
		m_mesh->GetAllocator()->FreeLow (m_gradients);
		m_mesh->GetAllocator()->FreeLow (m_deltaVariable);
		m_mesh->GetAllocator()->FreeLow (m_uv);
	}
	
	// the Hessian matrix is compose of these second partial derivatives
	// these derivatives are too complex and make the solver to spend too much time, 
	// [0][0]  2/b0^2 - W2 Sin[x0] Sin[x5] Sin[x9], 0, 0, 0, 0, W2 Cos[x0] Cos[x5] Sin[x9], 0, 0, 0, W2 Cos[x0] Cos[x9] Sin[x5], 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, Cos[x0] Sin[x5] Sin[x9], 0, 
	// [0][[1] {0, 2/b1^2 + W2 Sin[x1] Sin[x10] Sin[x3], 0, -W2 Cos[x1] Cos[x3] Sin[x10], 0, 0, 0, 0, 0, 0, -W2 Cos[x1] Cos[x10] Sin[x3], 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -Cos[x1] Sin[x10] Sin[x3], 0}, 
	// ...

	// the optimize version of the algorithms assume that the second derivatives are linear, therefore all sine terms are neglected, I will do the same
	// [ 0][0-n]  2/b0^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,  Cos[x0] Sin[x5] Sin[x9], 0 
	// [ 1][0-n]  0, 2/b1^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -Cos[x1] Sin[x10] Sin[x3], 0 
	// [ 2][0-n]  0, 0, 2/b2^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 
	// [ 3][0-n]  0, 0, 0, 2/b3^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -Cos[x3] Sin[x1] Sin[x10], Cos[x3] Sin[x11] Sin[x12] Sin[x8] 
	// [ 4][0-n]  0, 0, 0, 0, 2/b4^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -Cos[x4] Sin[x13] Sin[x6] Sin[x9]}	
	// [ 5][0-n]  0, 0, 0, 0, 0, 2/b5^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, Cos[x5] Sin[x0] Sin[x9], 0 
	// [ 6][0-n]  0, 0, 0, 0, 0, 0, 2/b6^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, -Cos[x6] Sin[x13] Sin[x4] Sin[x9] 
	// [ 7][0-n]  0, 0, 0, 0, 0, 0, 0, 2/b7^2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0
	// [ 8][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 2/b8^2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, Cos[x8] Sin[x11] Sin[x12] Sin[x3] 
	// [ 9][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 2/b9^2,  0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, Cos[x9] Sin[x0] Sin[x5], -Cos[x9] Sin[x13] Sin[x4] Sin[x6]
	// [10][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2/b10^2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, -Cos[x10] Sin[x1] Sin[x3], 0}, 
	// [11][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2/b11^2, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, Cos[x11] Sin[x12] Sin[x3] Sin[x8]
	// [12][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2/b12^2, 0, 0, 0, 0, 0, 0, 1,	0, 0, 0, Cos[x12] Sin[x11] Sin[x3] Sin[x8]
	// [13][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2/b13^2, 0, 0, 0, 0, 0, 1, 0, 0, 0, -Cos[x13] Sin[x4] Sin[x6] Sin[x9]
	// [14][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2/b14^2, 0, 0, 0, 0, 1, 0, 1, 0, 0

	// [15][0-n]  1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// [16][0-n]  0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// [17][0-n]  0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// [18][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// [19][0-n]  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0

	// [20][0-n]  0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// [21][0-n]  0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0

	// [22][0-n]  Cos[x0] Sin[x5] Sin[x9], -Cos[x1] Sin[x10] Sin[x3], 0, -Cos[x3] Sin[x1] Sin[x10], 0, Cos[x5] Sin[x0] Sin[x9], 0, 0, 0, Cos[x9] Sin[x0] Sin[x5], -Cos[x10] Sin[x1] Sin[x3], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	// [23][0-n]  0, 0, 0, Cos[x3] Sin[x11] Sin[x12] Sin[x8], -Cos[x4] Sin[x13] Sin[x6] Sin[x9], 0, -Cos[x6] Sin[x13] Sin[x4] Sin[x9], 0, Cos[x8] Sin[x11] Sin[x12] Sin[x3], -Cos[x9] Sin[x13] Sin[x4] Sin[x6], 0, Cos[x11] Sin[x12] Sin[x3] Sin[x8], Cos[x12] Sin[x11] Sin[x3] Sin[x8], -Cos[x13] Sin[x4] Sin[x6] Sin[x9], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

	void MatrixTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		dgAssert (0);
	}

	void InversePrecoditionerTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		dgAssert (0);
	}

	void GenerateUVCoordinates ()
	{
		dgAssert(0);
/*
		memset (m_uv, 0, m_anglesCount * sizeof (m_uv[0]));
		dgInt32 mark = m_mesh->IncLRU();
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgEdge* const face = m_betaEdge[i];
			if ((face->m_twin->m_incidentFace > 0) && face->m_mark != mark) {

				face->m_mark = mark;
				face->m_next->m_mark = mark;
				face->m_prev->m_mark = mark;

				dgInt32 edgeIndex0 = GetAlphaLandaIndex (face);
				dgInt32 edgeIndex1 = GetAlphaLandaIndex (face->m_next);
				dgInt32 edgeIndex2 = GetAlphaLandaIndex (face->m_prev);

				const dgBigVector& p0 = m_mesh->GetVertex(face->m_incidentVertex);
				const dgBigVector& p1 = m_mesh->GetVertex(face->m_next->m_incidentVertex);
				dgBigVector p10 (p1 - p0);
				dgFloat64 e0length = sqrt (p10 % p10);

				m_uv[edgeIndex0].m_x = dgFloat32 (0.0f);
				m_uv[edgeIndex0].m_y = dgFloat32 (0.0f);

				m_uv[edgeIndex1].m_x = dgFloat32 (e0length);
				m_uv[edgeIndex1].m_y = dgFloat32 (0.0f);

				dgFloat64 e2length = e0length * sin (m_alphaLambda[edgeIndex1]) / sin (m_alphaLambda[edgeIndex2]);
				dgFloat64 du (m_uv[edgeIndex1].m_x - m_uv[edgeIndex0].m_x);
				dgFloat64 dv (m_uv[edgeIndex1].m_y - m_uv[edgeIndex0].m_y);
				dgFloat64 refAngle = atan2 (dv, du);

				m_uv[edgeIndex2].m_x = m_uv[edgeIndex0].m_x + dgFloat32 (e2length * cos (m_alphaLambda[edgeIndex0] + refAngle));
				m_uv[edgeIndex2].m_y = m_uv[edgeIndex0].m_y + dgFloat32 (e2length * sin (m_alphaLambda[edgeIndex0] + refAngle));

				dgList<dgEdge*> stack(m_mesh->GetAllocator());
				if (face->m_twin->m_incidentFace > 0) {
					stack.Append(face->m_twin);
				}
				if (face->m_next->m_twin->m_incidentFace > 0) {
					stack.Append(face->m_next->m_twin);
				}

				if (face->m_prev->m_twin->m_incidentFace > 0) {
					stack.Append(face->m_prev->m_twin);
				}

				while (stack.GetCount()) {
					dgEdge* const edge = stack.GetLast()->GetInfo();
					stack.Remove (stack.GetLast());
					if (edge->m_mark != mark) {
						dgAssert (edge->m_incidentFace > 0);

						dgEdge* const next = edge->m_next;
						dgEdge* const prev = edge->m_prev;

						edge->m_mark = mark;
						next->m_mark = mark;
						prev->m_mark = mark;

						dgInt32 edgeIndex0 = GetAlphaLandaIndex (edge);
						dgInt32 edgeIndex1 = GetAlphaLandaIndex (next);
						dgInt32 edgeIndex2 = GetAlphaLandaIndex (prev);

						m_uv[edgeIndex0].m_x = m_uv[GetAlphaLandaIndex (edge->m_twin->m_next)].m_x;
						m_uv[edgeIndex0].m_y = m_uv[GetAlphaLandaIndex (edge->m_twin->m_next)].m_y;

						m_uv[edgeIndex1].m_x = m_uv[GetAlphaLandaIndex (edge->m_twin)].m_x;
						m_uv[edgeIndex1].m_y = m_uv[GetAlphaLandaIndex (edge->m_twin)].m_y;

						const dgBigVector& p0 = m_mesh->GetVertex(edge->m_incidentVertex);
						const dgBigVector& p1 = m_mesh->GetVertex(next->m_incidentVertex);
						dgBigVector p10 (p1 - p0);

						dgFloat64 e0length = sqrt (p10 % p10);
						dgFloat64 e2length = e0length * sin (m_alphaLambda[edgeIndex1]) / sin (m_alphaLambda[edgeIndex2]);

						dgFloat64 du (m_uv[edgeIndex1].m_x - m_uv[edgeIndex0].m_x);
						dgFloat64 dv (m_uv[edgeIndex1].m_y - m_uv[edgeIndex0].m_y);
						dgFloat64 refAngle = atan2 (dv, du);

						m_uv[edgeIndex2].m_x = m_uv[edgeIndex0].m_x + dgFloat32 (e2length * cos (m_alphaLambda[edgeIndex0] + refAngle));
						m_uv[edgeIndex2].m_y = m_uv[edgeIndex0].m_y + dgFloat32 (e2length * sin (m_alphaLambda[edgeIndex0] + refAngle));

						if (next->m_twin->m_incidentFace > 0) {
							stack.Append(next->m_twin);
						}
						if (prev->m_twin->m_incidentFace > 0) {
							stack.Append(prev->m_twin);
						}
					}
				} 
			}
		}
*/
	}

	void CalculateNumberOfVariables()
	{
		//m_mesh->SaveOFF("xxx.off");
		m_anglesCount = 0;
		m_triangleCount = 0;
		m_interiorVertexCount = 0;

		dgInt32 mark = m_mesh->IncLRU();
		dgMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				dgEdge *ptr = edge;
				do {
					m_anglesCount ++;
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != edge);
				m_triangleCount ++;
				dgAssert (edge->m_next->m_next->m_next == edge);
			}
		}

		mark = m_mesh->IncLRU();
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				bool isInterior = true;
				dgEdge *ptr = edge;
				do {
					isInterior &= (ptr->m_incidentFace > 0);
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				m_interiorVertexCount += isInterior ? 1 : 0;
			}
		}
		m_totalVariablesCount = m_anglesCount + m_triangleCount + 2 * m_interiorVertexCount;
	}

	void InitEdgeVector()
	{
		dgInt32 count = 0;
		dgInt32 mark = m_mesh->IncLRU();
		dgMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				dgEdge *ptr = edge;
				do {
					ptr->m_mark = mark;
					m_betaEdge[count] = ptr;
					ptr->m_incidentFace = count + 1;
					count ++;
					dgAssert (count <= m_anglesCount);
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}

		count = 0;
		mark = m_mesh->IncLRU();		
		memset (m_interiorIndirectMap, -1, m_mesh->GetVertexCount() * sizeof (m_interiorIndirectMap[0]));
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				bool isInterior = true;
				dgEdge* ptr = edge;
				do {
					isInterior &= (ptr->m_incidentFace > 0);
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				if (isInterior) {
					m_interiorIndirectMap[edge->m_incidentVertex] = m_anglesCount + m_triangleCount + count;
					count ++;
				}
			}
		}
	}



	dgInt32 GetAlphaLandaIndex (const dgEdge* const edge) const
	{
		return edge->m_incidentFace - 1;
	}

	dgInt32 GetTriangleIndex (const dgInt32 alphaIndex) const
	{
		return alphaIndex / 3 + m_anglesCount;
	}

	dgInt32 GetTriangleIndex (const dgEdge* const edge) const
	{
		return GetAlphaLandaIndex(edge) / 3 + m_anglesCount;
	}

	dgInt32 GetInteriorVertex(const dgEdge* const edge) const
	{
		return m_interiorIndirectMap[edge->m_incidentVertex];
	}

	void CalculateInitialAngles ()
	{
		// calculate initial beta angle for each triangle
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgEdge* const edge = m_betaEdge[i];

			const dgBigVector& p0 = m_mesh->GetVertex(edge->m_incidentVertex);
			const dgBigVector& p1 = m_mesh->GetVertex(edge->m_next->m_incidentVertex);
			const dgBigVector& p2 = m_mesh->GetVertex(edge->m_prev->m_incidentVertex);

			dgBigVector e10 (p1 - p0);
			dgBigVector e20 (p2 - p0);

			e10 = e10.Scale3 (dgFloat64 (1.0) / sqrt (e10 % e10));
			e20 = e20.Scale3 (dgFloat64 (1.0) / sqrt (e20 % e20));

			m_beta[i] = acos (dgClamp(e10 % e20, dgFloat64 (-1.0f), dgFloat64 (1.0f)));
			dgAssert (m_beta[i] > dgFloat64 (0.0f));
		}

		#ifdef _DEBUG
		for (dgInt32 i = 0; i < m_triangleCount; i ++) {
			dgInt32 i0 = i * 3 + 0;
			dgInt32 i1 = i * 3 + 1;
			dgInt32 i2 = i * 3 + 2;
			dgAssert (fabs (m_beta[i0] + m_beta[i1] + m_beta[i2] - dgABF_PI) < dgFloat64 (1.0e-6f));
		}
		#endif

		// for each interior vertex apply the scale factor
		dgInt32 mark = m_mesh->IncLRU();
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgEdge* const edge = m_betaEdge[i];
			if ((edge->m_mark != mark) && (GetInteriorVertex(edge) >= 0)) {
				dgFloat64 scale = dgFloat64 (0.0f);
				dgEdge* ptr = edge; 
				do {
					dgInt32 index = GetAlphaLandaIndex (ptr);
					scale += m_beta[index];
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				dgAssert (scale > dgFloat32 (0.0f));

				scale = dgFloat64 (2.0f) * dgABF_PI / scale;
				ptr = edge;
				do {
					dgInt32 index = GetAlphaLandaIndex (ptr);
					m_beta[index] *= scale;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}

		// initialized each alpha lambda to the beta angle and also calcual ethe derivatoe coeficent (2.0 / (betai * betai)) 
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgAssert (m_beta[i] > dgFloat64 (0.0f));
			m_variables[i] = m_beta[i];
			m_weight[i] = dgFloat64 (2.0f) / (m_beta[i] * m_beta[i]);
		}
	}

	// angular derivative component
	// (wi * (xi - bi) + T0
	// where wi = 2.0 / (bi ^ 2)
	dgFloat64 CalculateAngularGradientDerivative (dgInt32 alphaIndex) const
	{
		return (m_variables[alphaIndex] - m_beta[alphaIndex]) * m_weight[alphaIndex] + m_variables[GetTriangleIndex(alphaIndex)];
	}

	// Vi if the the edge is an interior vertex
	dgFloat64 CalculateInteriorVertexGradient (dgInt32 alphaIndex) const
	{
		dgInt32 index = GetInteriorVertex(m_betaEdge[alphaIndex]);
		dgFloat64 gradient = (index != -1) ? m_variables[index] : dgFloat32 (0.0f);
		return gradient;
	}

	// Wj * cos(alpha) * sum (alphai) for eadh previsu or next interior incdent vertex
	dgFloat64 CalculatePlanarityGradient (dgInt32 alphaIndex) const
	{
		dgFloat64 gradient = dgFloat64 (0.0f);

		dgEdge* const incidentEdge = m_betaEdge[alphaIndex];

		if (GetInteriorVertex (incidentEdge->m_next) != -1) {
			dgEdge* const edge = m_betaEdge[GetAlphaLandaIndex(incidentEdge->m_next)];
			dgFloat64 product = m_cosTable[GetAlphaLandaIndex(edge->m_prev)];
			dgEdge* ptr = edge->m_twin->m_next;
			do {
				product *= m_sinTable[GetAlphaLandaIndex(ptr->m_prev)];
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			dgInt32 interiorVertexIndex = GetInteriorVertex (incidentEdge->m_next) + m_interiorVertexCount;
			gradient -= m_variables[interiorVertexIndex] * product;
		}

		if (GetInteriorVertex (incidentEdge->m_prev) != -1) {
			dgEdge* const edge = m_betaEdge[GetAlphaLandaIndex(incidentEdge->m_prev)];
			dgFloat64 product = m_cosTable[GetAlphaLandaIndex(edge->m_next)];
			dgEdge* ptr = edge->m_twin->m_next;
			do {
				product *= m_sinTable[GetAlphaLandaIndex(ptr->m_next)];
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			dgInt32 interiorVertexIndex = GetInteriorVertex (incidentEdge->m_prev) + m_interiorVertexCount;
			gradient += m_variables[interiorVertexIndex] * product;
		}

		return gradient;
	}

	// sample of the Gradient Vector according to Mathematic to a generic mesh, this can be generalize for and arbitrary mesh topology
	// x0 - x14 are the planar angles in 2d
	// b0 - b14 are the mesh angles in 3d.
	// T0 - T4 are the triangle lambdas
	// V0 - V1 interior vertex lambdas
	// W0 - W1 interior vertex wheel lambdas 
	//
    // Gradient derivatives: 
	//0   (2 (-b0 + x0))/b0^2 + T0 + W2 Cos[x0] Sin[x5] Sin[x9] 
	//1   (2 (-b1 + x1))/b1^2 + T0 - W2 Cos[x1] Sin[x10] Sin[x3] 
	//2   (2 (-b2 + x2))/b2^2 + T0 + V2  
	//3   (2 (-b3 + x3))/b3^2 + T1 - W2 Cos[x3] Sin[x1] Sin[x10] + W3 Cos[x3] Sin[x11] Sin[x12] Sin[x8] 
	//4   (2 (-b4 + x4))/b4^2 + T1 + V2 - W3 Cos[x4] Sin[x13] Sin[x6] Sin[x9] 
	//5   (2 (-b5 + x5))/b5^2 + T1 + V3 + W2 Cos[x5] Sin[x0] Sin[x9] 
	//6   (2 (-b6 + x6))/b6^2 + T2 - W3 Cos[x6] Sin[x13] Sin[x4] Sin[x9] 
	//7   (2 (-b7 + x7))/b7^2 + T2 + V3 
	//8   (2 (-b8 + x8))/b8^2 + T2 + W3 Cos[x8] Sin[x11] Sin[x12] Sin[x3] 
	//9   (2 (-b09 + x09))/b09^2 + T3 + W2 Cos[x9] Sin[x0] Sin[x5] - W3 Cos[x9] Sin[x13] Sin[x4] Sin[x6] 
	//10  (2 (-b10 + x10))/b10^2 + T3 + V3 - W2 Cos[x10] Sin[x1] Sin[x3] 
	//11  (2 (-b11 + x11))/b11^2 + T3 + V2 + W3 Cos[x11] Sin[x12] Sin[x3] Sin[x8] 
	//12  (2 (-b12 + x12))/b12^2 + T4 + W3 Cos[x12] Sin[x11] Sin[x3] Sin[x8] 
	//13  (2 (-b13 + x13))/b13^2 + T4 - W3 Cos[x13] Sin[x4] Sin[x6] Sin[x9] 
	//14  (2 (-b14 + x14))/b14^2 + T4 + V3 
	//
	//15  x0 + x1 + x2 - pi
	//16  x3 + x4 + x5 - pi 
	//17  x6 + x7 + x8 - pi  
	//18  x10 + x11 + x9 - pi 
	//19  x12 + x13 + x14 - pi 
	//
	//20  x11 + x2 + x4 - 2 pi 
	//21  x10 + x14 + x5 + x7 - 2 pi 
	//
	//22  Sin[x0] Sin[x5] Sin[x9] - Sin[x1] Sin[x10] Sin[x3] 
	//23  Sin[x11] Sin[x12] Sin[x3] Sin[x8] - Sin[x13] Sin[x4] Sin[x6] Sin[x9]
	dgFloat64 CalculateGradientVector ()
	{
		// pre-compute sin cos tables
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			m_sinTable[i] = sin (m_variables[i]);
			m_cosTable[i] = cos (m_variables[i]);
		}

		dgFloat64 error2 = dgFloat64 (0.0f);

		// calculate gradients due to the difference between a matching edge angle and it projected angle msu be mminimal Wei * (Xei - Bei) ^ e = minimal
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgFloat64 gradient = CalculateAngularGradientDerivative (i) + CalculateInteriorVertexGradient (i) + CalculatePlanarityGradient (i);
			m_gradients[i] = -gradient;
			error2 += gradient * gradient;
		}

		// calculate gradient due to the equality that the sum on the internal angle of a triangle must add to 180 degree. (Xt0 + Xt1 + Xt2 - pi) = 0
		for (dgInt32 i = 0; i < m_triangleCount; i ++) {
			dgFloat64 gradient = m_gradients[i * 3 + 0] + m_gradients[i * 3 + 1] + m_gradients[i * 3 + 2] - dgABF_PI;
			m_gradients[i] = -gradient;
			error2 += gradient * gradient;
		}

		// calculate the gradient due to the equality that the sum of all the angle incident to and interior vertex must be 3060 degree sum (Xvi) - 2 * pi = 0 
		dgInt32 mark = m_mesh->IncLRU();
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgEdge* const edge = m_betaEdge[i];

			if ((edge->m_mark != mark) && GetInteriorVertex(edge) != -1) {
				dgInt32 vertexIndex = GetInteriorVertex(edge);
				dgFloat64 gradient = - dgFloat64 (2.0f) * dgABF_PI;

				dgEdge* ptr = edge; 
				do {
					dgInt32 index = GetAlphaLandaIndex(ptr);
					gradient += m_variables[index];
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				m_gradients[vertexIndex] = - gradient;
				error2 += gradient * gradient;
			}
		}

		// calculate the gradient due to the equality that the difference of the product of the sin of the angle to the
		// incident to an interior vertex must be zero product (sin (Xvi + 1) - product (sin (Xvi - 1)  = 0 
		mark = m_mesh->IncLRU();
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgEdge* const edge = m_betaEdge[i];

			dgInt32 vertexIndex = GetInteriorVertex(edge);
			if ((edge->m_mark != mark) && (vertexIndex != -1)) {
				vertexIndex += m_interiorVertexCount;
				dgFloat64 partialProdut0 =  dgFloat64 (1.0f);
				dgFloat64 partialProdut1 =  dgFloat64 (1.0f);
				dgEdge* ptr = edge; 
				do {
					dgInt32 index0 = GetAlphaLandaIndex(ptr->m_next);
					dgInt32 index1 = GetAlphaLandaIndex(ptr->m_prev);
					partialProdut0 *= m_sinTable[index0];
					partialProdut1 *= m_sinTable[index1];
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				dgFloat64 gradient = partialProdut0 - partialProdut1;
				m_gradients[vertexIndex] = - gradient;
				error2 += gradient * gradient;
			}
		}

		return error2;
	}

	void LagrangeOptimization()
	{
		memset (&m_variables[m_anglesCount], 0, (m_totalVariablesCount - m_anglesCount) * sizeof (dgFloat64));	
		dgFloat64 error2 = CalculateGradientVector ();

		const dgInt32 solverIter = dgMax (dgInt32 (sqrt (dgFloat64(m_totalVariablesCount))), 10);
		const dgFloat64 solverTolerance = dgABF_TOL2;

		for (dgInt32 iter = 0; (iter < dgABF_MAX_ITERATIONS) && (error2 > dgABF_TOL2); iter++) {
			Solve(m_totalVariablesCount, solverIter, solverTolerance, m_deltaVariable, m_gradients);
			for (dgInt32 i = 0; i < m_totalVariablesCount; i ++) {
				m_variables[i] += m_deltaVariable[i];
			}
		}

	}

	dgMeshEffect* m_mesh;
	
	dgEdge** m_betaEdge;
	dgInt32* m_interiorIndirectMap;

	dgFloat64* m_beta;
	dgFloat64* m_weight;
	dgFloat64* m_sinTable;
	dgFloat64* m_cosTable;
	dgFloat64* m_variables;
	dgFloat64* m_gradients;
	dgFloat64* m_deltaVariable;

	dgTriplex* m_uv;

	dgInt32 m_material;
	dgInt32 m_anglesCount;
	dgInt32 m_triangleCount;
	dgInt32 m_interiorVertexCount;
	dgInt32 m_totalVariablesCount;
};	

dgBigVector dgMeshEffect::GetOrigin ()const
{
    dgBigVector origin (dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f), dgFloat64 (0.0f));	
    for (dgInt32 i = 0; i < m_pointCount; i ++) {
        origin += m_points[i];
    }	
    return origin.Scale3 (dgFloat64 (1.0f) / m_pointCount);
}


dgInt32 dgMeshEffect::EnumerateAttributeArray (dgVertexAtribute* const attib)
{
    dgInt32 index = 0;
    dgPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        dgAssert (index < GetCount());
        if (edge->m_incidentFace > 0) {
            attib[index] = m_attrib[dgInt32 (edge->m_userData)];
            edge->m_userData = dgUnsigned64 (index);
            index ++;
        }
    }
    return index;
}

void dgMeshEffect::ClearAttributeArray ()
{
    dgStack<dgVertexAtribute>attribArray (m_pointCount);

    memset (&attribArray[0], 0, m_pointCount * sizeof (dgVertexAtribute));
    dgInt32 mark = IncLRU();
    dgPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        if (edge->m_mark < mark){
            dgEdge* ptr = edge;

            dgInt32 index = ptr->m_incidentVertex;
            dgVertexAtribute& attrib = attribArray[index];
            attrib.m_vertex = m_points[index];
            do {
                ptr->m_mark = mark;
                ptr->m_userData = index;
                ptr = ptr->m_twin->m_next;
            } while (ptr !=  edge);

        }
    }
    ApplyAttributeArray (&attribArray[0], m_pointCount);
}


void dgMeshEffect::ApplyAttributeArray (dgVertexAtribute* const attib, dgInt32 maxCount)
{
    dgStack<dgInt32>indexMap (dgMax (GetCount(), maxCount));

    m_atribCount = dgVertexListToIndexList (&attib[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof(dgFloat64), maxCount, &indexMap[0], DG_VERTEXLIST_INDEXLIST_TOL);
    m_maxAtribCount = m_atribCount;

    GetAllocator()->FreeLow (m_attrib);

    m_attrib = (dgVertexAtribute*) GetAllocator()->MallocLow(dgInt32 (m_atribCount * sizeof(dgVertexAtribute)));
    memcpy (m_attrib, attib, m_atribCount * sizeof(dgVertexAtribute));

    dgPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        if (edge->m_incidentFace > 0) {
            dgInt32 index = indexMap[dgInt32 (edge->m_userData)];
            dgAssert (index >= 0);
            dgAssert (index < m_atribCount);
            edge->m_userData = dgUnsigned64 (index);
        }
    }
}


void dgMeshEffect::CalculateNormals (dgFloat64 angleInRadians)
{
    dgEdge* edgeBuffer[256];
    dgBigVector faceNormal[256];
    dgVertexAtribute tmpAttributes[256];
    dgStack<dgVertexAtribute> attibutes(m_atribCount);
    memcpy (&attibutes[0], &m_attrib[0], m_atribCount * sizeof (dgVertexAtribute));

    m_atribCount = 0;
    dgInt32 mark = IncLRU();
    dgPolyhedra::Iterator iter (*this);	

    dgFloat32 smoothValue = dgCos (angleInRadians); 
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
            dgTree<dgInt32, dgEdge*> normalsMap(GetAllocator()) ;
            dgInt32 edgeIndex = 0;
            dgEdge* ptr = edge;
            do {
                dgBigVector normal (FaceNormal (ptr, &m_points[0].m_x, sizeof (m_points[0])));
                normal = normal.Scale3 (dgFloat32 (1.0f) / (sqrt(normal % normal) + dgFloat32(1.0e-16f)));
                faceNormal[edgeIndex] = normal;
                normalsMap.Insert(edgeIndex, ptr);
                edgeIndex ++;
                ptr = ptr->m_twin->m_next;
            } while (ptr != edge);


            dgEdge* startEdge = edge;
            dgBigVector normal0 (faceNormal[normalsMap.Find(startEdge)->GetInfo()]);
            for (dgEdge* ptr = edge->m_prev->m_twin ; (ptr != edge) && (ptr->m_incidentFace > 0); ptr = ptr->m_prev->m_twin) {
                const dgBigVector& normal1 (faceNormal[normalsMap.Find(ptr)->GetInfo()]);
                dgFloat64 dot = normal0 % normal1;
                if (dot < smoothValue) {
                    break;
                }
                startEdge = ptr;
                normal0 = normal1;
            }


            dgInt32 attribCount = 1;
            edgeBuffer[0] = startEdge;
            tmpAttributes[0] = attibutes[dgInt32 (startEdge->m_userData)];
            normal0 = faceNormal[normalsMap.Find(startEdge)->GetInfo()];
            dgBigVector normal (normal0);
            for (dgEdge* ptr = startEdge->m_twin->m_next; (ptr != startEdge) && (ptr->m_incidentFace > 0); ptr = ptr->m_twin->m_next) { 
                const dgBigVector& normal1 (faceNormal[normalsMap.Find(ptr)->GetInfo()]);
                dgFloat64 dot = normal0 % normal1;
                if (dot < smoothValue)  {
                    break;
                }
                edgeBuffer[attribCount] = ptr;
                tmpAttributes[attribCount] = attibutes[dgInt32 (ptr->m_userData)];
                attribCount ++;

                normal += normal1;
                normal0 = normal1;
            } 
            normal = normal.Scale3 (dgFloat32 (1.0f) / (sqrt(normal % normal) + dgFloat32(1.0e-16f)));
            for (dgInt32 i = 0; i < attribCount; i ++) {
                tmpAttributes[i].m_normal_x = normal.m_x;
                tmpAttributes[i].m_normal_y = normal.m_y;
                tmpAttributes[i].m_normal_z = normal.m_z;
            }
            if (attribCount == 1) {
                edgeBuffer[0]->m_mark = mark;
                edgeBuffer[0]->m_userData = m_atribCount;
                AddAtribute(tmpAttributes[0]);
            } else {
                dgInt32 indexArray[256];
                dgInt32 count = dgVertexListToIndexList (&tmpAttributes[0].m_vertex.m_x, sizeof (dgVertexAtribute), sizeof (dgVertexAtribute) / sizeof(dgFloat64), attribCount, &indexArray[0], DG_VERTEXLIST_INDEXLIST_TOL);

                for (dgInt32 i = 0; i < attribCount; i ++) {
                    edgeBuffer[i]->m_mark = mark;
                    edgeBuffer[i]->m_userData = m_atribCount + indexArray[i];
                }
                for (dgInt32 i = 0; i < count; i ++) {
                    AddAtribute(tmpAttributes[i]);
                }
            }
        }
    }
    PackVertexArrays();
}

void dgMeshEffect::SphericalMapping (dgInt32 material)
{
    dgBigVector origin (GetOrigin());

    dgStack<dgBigVector>sphere (m_pointCount);
    for (dgInt32 i = 0; i < m_pointCount; i ++) {
        dgBigVector point (m_points[i] - origin);
        dgAssert ((point % point) > dgFloat32 (0.0f));
        point = point.Scale3 (dgFloat64 (1.0f) / sqrt (point % point));

        dgFloat64 u = dgAsin (dgClamp (point.m_y, dgFloat64 (-1.0f + 1.0e-6f), dgFloat64 (1.0f - 1.0e-6f)));
        dgFloat64 v = dgAtan2 (point.m_x, point.m_z);

        u = dgFloat32 (1.0f) -(dgFloat64 (3.141592f/2.0f) - u) / dgFloat64 (3.141592f);
        dgAssert (u >= dgFloat32 (0.0f));
        dgAssert (u <= dgFloat32 (1.0f));

        v = (dgFloat64 (3.141592f) - v) / dgFloat64 (2.0f * 3.141592f);

        sphere[i].m_x = v;
        sphere[i].m_y = u;
    }


    dgStack<dgVertexAtribute>attribArray (GetCount());
    dgInt32 count = EnumerateAttributeArray (&attribArray[0]);

    dgPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        dgVertexAtribute& attrib = attribArray[dgInt32 (edge->m_userData)];
        attrib.m_u0 = sphere[edge->m_incidentVertex].m_x;
        attrib.m_v0 = sphere[edge->m_incidentVertex].m_y;
        attrib.m_u1 = sphere[edge->m_incidentVertex].m_x;
        attrib.m_v1 = sphere[edge->m_incidentVertex].m_y;
        attrib.m_material = material;
    }

    dgInt32 mark = IncLRU ();
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
            dgBigVector normal(0.0f); 
            edge->m_mark = mark;
            edge->m_next->m_mark = mark;
            dgVertexAtribute& attrib0 = attribArray[dgInt32 (edge->m_userData)];
            dgVertexAtribute& attrib1 = attribArray[dgInt32 (edge->m_next->m_userData)];
            dgBigVector p0 (attrib0.m_u0, attrib0.m_v0, dgFloat64 (0.0f), dgFloat64 (0.0f));
            dgBigVector p1 (attrib1.m_u0, attrib1.m_v0, dgFloat64 (0.0f), dgFloat64 (0.0f));
            dgBigVector e0 (p1 - p0);
            dgEdge* ptr = edge->m_next->m_next;
            do {
                ptr->m_mark = mark;
                dgVertexAtribute& attrib2 = attribArray[dgInt32 (ptr->m_userData)];
                dgBigVector p2 (attrib2.m_u0, attrib2.m_v0, dgFloat64 (0.0f), dgFloat64 (0.0f));
                dgBigVector e1 (p2 - p0);
                normal += e1 * e0;
                ptr = ptr->m_next;
            } while (ptr != edge);

            if (normal.m_z < dgFloat32 (0.0f)) {
                dgEdge* ptr = edge;
                do {
                    dgVertexAtribute& attrib = attribArray[dgInt32 (ptr->m_userData)];
                    if (attrib.m_u0 < dgFloat32(0.5f)) {
                        attrib.m_u0 += dgFloat32(1.0f);
                        attrib.m_u1 = attrib.m_u0;
                    }
                    ptr = ptr->m_next;
                } while (ptr != edge);
            }
        }
    }

    ApplyAttributeArray (&attribArray[0], count);
}


void dgMeshEffect::CylindricalMapping (dgInt32 cylinderMaterial, dgInt32 capMaterial)
{
    dgBigVector origin (GetOrigin());
    dgStack<dgBigVector>cylinder (m_pointCount);

    dgBigVector pMin (dgFloat64 (1.0e10f), dgFloat64 (1.0e10f), dgFloat64 (1.0e10f), dgFloat64 (0.0f));
    dgBigVector pMax (dgFloat64 (-1.0e10f), dgFloat64 (-1.0e10f), dgFloat64 (-1.0e10f), dgFloat64 (0.0f));
    for (dgInt32 i = 0; i < m_pointCount; i ++) {
        dgBigVector tmp (m_points[i] - origin);
        pMin.m_x = dgMin (pMin.m_x, tmp.m_x);
        pMax.m_x = dgMax (pMax.m_x, tmp.m_x);
        pMin.m_y = dgMin (pMin.m_y, tmp.m_y);
        pMax.m_y = dgMax (pMax.m_y, tmp.m_y);
        pMin.m_z = dgMin (pMin.m_z, tmp.m_z);
        pMax.m_z = dgMax (pMax.m_z, tmp.m_z);
    }

    dgBigVector scale (dgFloat64 (1.0f)/ (pMax.m_x - pMin.m_x), dgFloat64 (1.0f)/ (pMax.m_y - pMin.m_y), dgFloat64 (1.0f)/ (pMax.m_z - pMin.m_z), dgFloat64 (0.0f));
    for (dgInt32 i = 0; i < m_pointCount; i ++) {
        dgBigVector point (m_points[i] - origin);
        dgFloat64 u = (point.m_x - pMin.m_x) * scale.m_x;

        dgAssert ((point % point) > dgFloat32 (0.0f));
        point = point.Scale3 (dgFloat64 (1.0f) / sqrt (point % point));
        dgFloat64 v = dgAtan2 (point.m_y, point.m_z);

        v = (v - dgFloat64 (3.141592f)) / dgFloat64 (2.0f * 3.141592f) + dgFloat64 (1.0f);
        cylinder[i].m_x = u;
        cylinder[i].m_y = v;
    }


    dgStack<dgVertexAtribute>attribArray (GetCount());
    dgInt32 count = EnumerateAttributeArray (&attribArray[0]);

    dgPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        dgVertexAtribute& attrib = attribArray[dgInt32 (edge->m_userData)];
        attrib.m_u0 = cylinder[edge->m_incidentVertex].m_x;
        attrib.m_v0 = cylinder[edge->m_incidentVertex].m_y;
        attrib.m_u1 = cylinder[edge->m_incidentVertex].m_x;
        attrib.m_v1 = cylinder[edge->m_incidentVertex].m_y;
        attrib.m_material = cylinderMaterial;
    }

    dgInt32 mark = IncLRU ();
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
            dgBigVector normal(0.0f); 
            edge->m_mark = mark;
            edge->m_next->m_mark = mark;
            dgVertexAtribute& attrib0 = attribArray[dgInt32 (edge->m_userData)];
            dgVertexAtribute& attrib1 = attribArray[dgInt32 (edge->m_next->m_userData)];
            dgBigVector p0 (attrib0.m_u0, attrib0.m_v0, dgFloat64 (0.0f), dgFloat64 (0.0f));
            dgBigVector p1 (attrib1.m_u0, attrib1.m_v0, dgFloat64 (0.0f), dgFloat64 (0.0f));
            dgBigVector e0 (p1 - p0);
            dgEdge* ptr = edge->m_next->m_next;
            do {
                ptr->m_mark = mark;
                dgVertexAtribute& attrib2 = attribArray[dgInt32 (ptr->m_userData)];
                dgBigVector p2 (attrib2.m_u0, attrib2.m_v0, dgFloat64 (0.0f), dgFloat64 (0.0f));
                dgBigVector e1 (p2 - p0);
                normal += e0 * e1;
                ptr = ptr->m_next;
            } while (ptr != edge);

            if (normal.m_z < dgFloat32 (0.0f)) {
                dgEdge* ptr = edge;
                do {
                    dgVertexAtribute& attrib = attribArray[dgInt32 (ptr->m_userData)];
                    if (attrib.m_v0 < dgFloat32(0.5f)) {
                        attrib.m_v0 += dgFloat32(1.0f);
                        attrib.m_v1 = attrib.m_v0;
                    }
                    ptr = ptr->m_next;
                } while (ptr != edge);
            }
        }
    }


    // apply cap mapping
    mark = IncLRU ();
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        //if (edge->m_mark < mark){
        if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
            const dgVector& p0 = m_points[edge->m_incidentVertex];
            const dgVector& p1 = m_points[edge->m_next->m_incidentVertex];
            const dgVector& p2 = m_points[edge->m_prev->m_incidentVertex];

            edge->m_mark = mark;
            edge->m_next->m_mark = mark;
            edge->m_prev->m_mark = mark;

            dgVector e0 (p1 - p0);
            dgVector e1 (p2 - p0);
            dgVector n (e0 * e1);
            if ((n.m_x * n.m_x) > (dgFloat32 (0.99f) * (n % n))) {
                dgEdge* ptr = edge;
                do {
                    dgVertexAtribute& attrib = attribArray[dgInt32 (ptr->m_userData)];
                    dgVector p (m_points[ptr->m_incidentVertex] - origin);
                    dgFloat64 u = (p.m_y - pMin.m_y) * scale.m_y;
                    dgFloat64 v = (p.m_z - pMin.m_z) * scale.m_z;

                    attrib.m_u0 = u;
                    attrib.m_v0 = v;
                    attrib.m_u1 = u;
                    attrib.m_v1 = v;
                    attrib.m_material = capMaterial;

                    ptr = ptr->m_next;
                }while (ptr !=  edge);
            }
        }
    }

    ApplyAttributeArray (&attribArray[0], count);
}



void dgMeshEffect::BoxMapping (dgInt32 front, dgInt32 side, dgInt32 top)
{
    dgBigVector minVal;
    dgBigVector maxVal;
    dgInt32 materialArray[3];

    GetMinMax (minVal, maxVal, &m_points[0][0], m_pointCount, sizeof (dgBigVector));
    dgBigVector dist (maxVal - minVal);
    dist[0] = dgMax (dgFloat64 (1.0e-3f), dist[0]);
    dist[1] = dgMax (dgFloat64 (1.0e-3f), dist[1]);
    dist[2] = dgMax (dgFloat64 (1.0e-3f), dist[2]);
    dgBigVector scale (dgFloat64 (1.0f)/ dist[0], dgFloat64 (1.0f)/ dist[1], dgFloat64 (1.0f)/ dist[2], dgFloat64 (0.0f));

    dgStack<dgVertexAtribute>attribArray (GetCount());
    dgInt32 count = EnumerateAttributeArray (&attribArray[0]);

    materialArray[0] = front;
    materialArray[1] = side;
    materialArray[2] = top;

    dgInt32 mark = IncLRU();
    dgPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        dgEdge* const edge = &(*iter);
        if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
            const dgBigVector& p0 = m_points[edge->m_incidentVertex];
            const dgBigVector& p1 = m_points[edge->m_next->m_incidentVertex];
            const dgBigVector& p2 = m_points[edge->m_prev->m_incidentVertex];

            edge->m_mark = mark;
            edge->m_next->m_mark = mark;
            edge->m_prev->m_mark = mark;

            dgBigVector e0 (p1 - p0);
            dgBigVector e1 (p2 - p0);
            dgBigVector n (e0 * e1);

            dgInt32 index = 0;
            dgFloat64 maxProjection = dgFloat32 (0.0f);

            for (dgInt32 i = 0; i < 3; i ++) {
                dgFloat64 proj = fabs (n[i]);
                if (proj > maxProjection) {
                    index = i;
                    maxProjection = proj;
                }
            }

            dgInt32 u = (index + 1) % 3;
            dgInt32 v = (u + 1) % 3;
            if (index == 1) {
                dgSwap (u, v);
            }
            dgEdge* ptr = edge;
            do {
                dgVertexAtribute& attrib = attribArray[dgInt32 (ptr->m_userData)];
                dgBigVector p (scale.CompProduct3(m_points[ptr->m_incidentVertex] - minVal));
                attrib.m_u0 = p[u];
                attrib.m_v0 = p[v];
                attrib.m_u1 = dgFloat64(0.0f);
                attrib.m_v1 = dgFloat64(0.0f);
                attrib.m_material = materialArray[index];

                ptr = ptr->m_next;
            }while (ptr !=  edge);
        }
    }

    ApplyAttributeArray (&attribArray[0], count);
}


void dgMeshEffect::UniformBoxMapping (dgInt32 material, const dgMatrix& textureMatrix)
{
    dgStack<dgVertexAtribute>attribArray (GetCount());
    dgInt32 count = EnumerateAttributeArray (&attribArray[0]);

    dgInt32 mark = IncLRU();
    for (dgInt32 i = 0; i < 3; i ++) {
        dgMatrix rotationMatrix (dgGetIdentityMatrix());
        if (i == 1) {
            rotationMatrix = dgYawMatrix(dgFloat32 (90.0f * 3.141592f / 180.0f));
        } else if (i == 2) {
            rotationMatrix = dgPitchMatrix(dgFloat32 (90.0f * 3.141592f / 180.0f));
        }

        dgPolyhedra::Iterator iter (*this);	

        for(iter.Begin(); iter; iter ++){
            dgEdge* const edge = &(*iter);
            if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) {
                dgBigVector n (FaceNormal(edge, &m_points[0].m_x, sizeof (dgBigVector)));
                dgVector normal (rotationMatrix.RotateVector(dgVector (n.Scale3 (dgFloat64 (1.0f) / sqrt (n % n)))));
                normal.m_x = dgAbsf (normal.m_x);
                normal.m_y = dgAbsf (normal.m_y);
                normal.m_z = dgAbsf (normal.m_z);
                if ((normal.m_z >= (normal.m_x - dgFloat32 (1.0e-4f))) && (normal.m_z >= (normal.m_y - dgFloat32 (1.0e-4f)))) {
                    dgEdge* ptr = edge;
                    do {
                        ptr->m_mark = mark;
                        dgVertexAtribute& attrib = attribArray[dgInt32 (ptr->m_userData)];
                        dgVector p (textureMatrix.TransformVector(rotationMatrix.RotateVector(m_points[ptr->m_incidentVertex])));
                        attrib.m_u0 = p.m_x;
                        attrib.m_v0 = p.m_y;
                        attrib.m_u1 = dgFloat32 (0.0f);
                        attrib.m_v1 = dgFloat32 (0.0f);
                        attrib.m_material = material;
                        ptr = ptr->m_next;
                    }while (ptr !=  edge);
                }
            }
        }
    }

    ApplyAttributeArray (&attribArray[0], count);
}



void dgMeshEffect::AngleBaseFlatteningMapping (dgInt32 material)
{

TestSolver_xxxxxxx xx;


	dgAngleBasedFlatteningMapping angleBadedFlattening (this, material);
}
