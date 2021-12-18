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

// algorithm from paper: Non-Distorted Texture Mapping Using Angle Based Flattening
// by: A. Sheffer and E. de Sturler
// http://www.math.vt.edu/people/sturler/Publications/UIUCDCS-R-2001-2257.pdf
// 
// also improvement from paper: ABF++: Fast and Robust Angle Based Flattening
// http://hal.archives-ouvertes.fr/docs/00/10/56/89/PDF/abf_plus_plus_temp.pdf
// by: Alla Sheffer, Bruno Lévy, Inria Lorraine, Maxim Mogilnitsky and Alexander Bogomyakov
//
// also looking at paper
// Least Squares Conformal Maps for Automatic Texture Atlas Generation
// http://www.cs.jhu.edu/~misha/Fall09/Levy02.pdf
// by Bruno Lévy Sylvain Petitjean Nicolas Ray Jérome Maillot
// for automatic seam and atlas generation 

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndStack.h"
#include "ndMatrix.h"
#include "ndMeshEffect.h"
//#include "dgWorld.h"
//#include "ndMeshEffect.h"

#if 0
#define dgABF_MAX_ITERATIONS		5
#define dgABF_TOL2					ndFloat64 (1.0e-12)
#define dgABF_LINEAR_SOLVER_TOL		ndFloat64 (1.0e-14)
#define dgABF_PI					ndFloat64 (3.1415926535)

#define dgABF_UV_TOL2				ndFloat64 (1.0e-8)

#if 1
	#define DG_DEBUG_UV	dTrace 
#else
	#define DG_DEBUG_UV	
#endif


class dgTriangleAnglesToUV: public dgSymmetricConjugateGradientSolver<ndFloat64>
{
	public:
	dgTriangleAnglesToUV (ndMeshEffect* const mesh, ndInt32 material, dgReportProgress progressReportCallback, void* const userData, const ndFloat64* const pinnedPoint, ndFloat64* const triangleAnglesVector = nullptr)
		:m_hessianCoLumnIndex (mesh->GetAllocator())
		,m_hessianCoLumnValue(mesh->GetAllocator())
		,m_mesh(mesh)
		,m_triangleAngles(triangleAnglesVector)
		,m_pinnedPoints(pinnedPoint)
		,m_trianglesCount(0)
		,m_matrixElementCount(0)
		,m_allocated(false)
	{
		ndInt32 mark = m_mesh->IncLRU();
		ndMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				ndEdge *ptr = edge;
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != edge);
				m_trianglesCount ++;
				dAssert (edge->m_next->m_next->m_next == edge);
			}
		}

		m_triangles = (ndEdge**) m_mesh->GetAllocator()->MallocLow (m_trianglesCount * sizeof (ndEdge*));

		ndInt32 count = 0;
		mark = m_mesh->IncLRU();
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				ndEdge *ptr = edge;
				do {
					ptr->m_incidentFace = count + 1;
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != edge);
				m_triangles[count] = ptr;
				count ++;
				dAssert (count <= m_trianglesCount);
			}
		}

		if (!m_triangleAngles) {
			dAssert (0);
			AnglesFromUV ();
		}
		
		m_uvArray = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (2 * m_mesh->GetVertexCount() * sizeof (ndFloat64));
		mark = m_mesh->IncLRU();
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				ndEdge* ptr = edge;
				ndEdge* uvEdge = edge;
				do {
					if ((uvEdge->m_incidentFace < 0) && (ptr->m_incidentFace > 0)) {
						uvEdge = ptr;
					}
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				dAssert (0);
/*
				ndInt32 index = ndInt32 (uvEdge->m_userData);
				ndMeshEffect::dgVertexAtribute& attribute = m_mesh->GetAttribute (index);
				m_uvArray[index * 2 + 0] = attribute.m_u0;
				m_uvArray[index * 2 + 1] = attribute.m_v0;
*/
			}
		}

		m_sinTable = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (3 * m_trianglesCount * sizeof (ndFloat64));
		m_cosTable = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (3 * m_trianglesCount * sizeof (ndFloat64));

		// pre-compute sin cos tables
		for (ndInt32 i = 0; i < m_trianglesCount * 3; i ++) {
			m_sinTable[i] = sin (m_triangleAngles[i]);
			m_cosTable[i] = cos (m_triangleAngles[i]);
		}

		m_vertexEdge = (ndEdge**) m_mesh->GetAllocator()->MallocLow (m_mesh->GetVertexCount() * sizeof (ndEdge*));
		mark = m_mesh->IncLRU();
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const vertex = &iter.GetNode()->GetInfo();
			if (vertex->m_mark != mark) {
				ndInt32 index = vertex->m_incidentVertex;
				m_vertexEdge[index] = vertex;
				ndEdge* ptr = vertex;
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next					;
				} while (ptr != vertex);
			}
		}

		m_gradients = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (2 * m_mesh->GetVertexCount() * sizeof (ndFloat64));
		m_diagonal = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (2 * m_mesh->GetVertexCount() * sizeof (ndFloat64));

		LagrangeOptimization();

		dAssert (0);
/*
		ndStack<ndMeshEffect::dgVertexAtribute>attribArray (m_mesh->GetCount());
//		ndInt32 attribCount = m_mesh->EnumerateAttributeArray (&attribArray[0]);
		mark = m_mesh->IncLRU();
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if (edge->m_mark != mark) {
				ndInt32 vertexIndex = edge->m_incidentVertex;
				ndEdge* const vertexEdge = m_vertexEdge[vertexIndex];
				ndFloat64 u = m_uvArray[vertexIndex * 2 + 0];
				ndFloat64 v = m_uvArray[vertexIndex * 2 + 1];
				ndEdge* ptr = vertexEdge;
				do {
					if (ptr->m_incidentFace > 0) {
						ndInt32 index = ndInt32 (ptr->m_userData);
						ndMeshEffect::dgVertexAtribute& attribute = m_mesh->GetAttribute (index);
						attribute.m_u0 = u;
						attribute.m_v0 = v;
						attribute.m_material = material;
					}
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != vertexEdge);
			}
		}
*/
		dAssert (0);
		//m_mesh->ApplyAttributeArray(&attribArray[0], attribCount);
	}

	~dgTriangleAnglesToUV()
	{
		m_mesh->GetAllocator()->FreeLow (m_diagonal);
		m_mesh->GetAllocator()->FreeLow (m_gradients);
		m_mesh->GetAllocator()->FreeLow (m_vertexEdge);
		m_mesh->GetAllocator()->FreeLow (m_sinTable);
		m_mesh->GetAllocator()->FreeLow (m_cosTable);
		m_mesh->GetAllocator()->FreeLow (m_uvArray);
		m_mesh->GetAllocator()->FreeLow (m_triangles);

		if (m_allocated) {
			m_mesh->GetAllocator()->FreeLow (m_triangleAngles);
		}
	}

/*
	void GenerateUVCoordinates ()
	{
		m_mesh->SaveOFF("xxx.off");

		ndStack<dInt8> attibuteUsed (m_attibuteCount);
		memset (&attibuteUsed[0], 0, attibuteUsed.GetSizeInBytes());
		ndInt32 mark = m_mesh->IncLRU();
		for (ndInt32 i = 0; i < m_triangleCount; i ++) {
			ndEdge* const face = m_betaEdge[i * 3];
			if (face->m_mark != mark) {
				ndEdge* ptr = face;
				do {
					if (ptr->m_incidentFace > 0) {
						ndInt32 index = ndInt32 (ptr->m_userData);
						attibuteUsed[index] = 1;
						m_uvArray[index].m_u0 = ndFloat32 (0.0f);
						m_uvArray[index].m_v0 = ndFloat32 (0.0f);
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != face);

				ndEdge* const twinFace = face->m_twin;
				const ndBigVector& p0 = m_mesh->GetVertex(face->m_incidentVertex);
				const ndBigVector& p1 = m_mesh->GetVertex(twinFace->m_incidentVertex);
				ndBigVector p10 (p1 - p0);
				ndFloat64 e0length = sqrt (p10 % p10);

				ptr = twinFace;
				do {
					if (ptr->m_incidentFace > 0) {
						ndInt32 index = ndInt32 (ptr->m_userData);
						attibuteUsed[index] = 1;
						m_uvArray[index].m_u0 = e0length;
						m_uvArray[index].m_v0 = ndFloat32 (0.0f);
					}
					ptr = ptr->m_twin->m_next;
				} while (ptr != twinFace);

				dList<ndEdge*> stack(m_mesh->GetAllocator());
				stack.Append(face);
				while (stack.GetCount()) {
					dList<ndEdge*>::dNode* const node = stack.GetFirst();
					ndEdge* const face = node->GetInfo();
					stack.Remove (node);
					if (face->m_mark != mark) {
						ndInt32 uvIndex2 = ndInt32 (face->m_prev->m_userData);
						if (!attibuteUsed[uvIndex2]) {

							ndInt32 uvIndex0 = ndInt32 (face->m_userData);
							ndInt32 uvIndex1 = ndInt32 (face->m_next->m_userData);

							ndInt32 edgeIndex0 = GetAlphaLandaIndex (face);
							ndInt32 edgeIndex1 = GetAlphaLandaIndex (face->m_next);
							ndInt32 edgeIndex2 = GetAlphaLandaIndex (face->m_prev);

							ndFloat64 refAngleCos = cos (m_variables[edgeIndex0]);
							ndFloat64 refAngleSin = sin (m_variables[edgeIndex0]);
							ndFloat64 scale = sin (m_variables[edgeIndex1]) / sin (m_variables[edgeIndex2]);

							ndFloat64 du = (m_uvArray[uvIndex1].m_u0 - m_uvArray[uvIndex0].m_u0) * scale;
							ndFloat64 dv = (m_uvArray[uvIndex1].m_v0 - m_uvArray[uvIndex0].m_v0) * scale;
							ndFloat64 u = m_uvArray[uvIndex0].m_u0 + du * refAngleCos - dv * refAngleSin; 
							ndFloat64 v = m_uvArray[uvIndex0].m_v0 + du * refAngleSin + dv * refAngleCos; 

							ndEdge* ptr = face->m_prev;
							do {
								if (ptr->m_incidentFace > 0) {
									ndInt32 index = ndInt32 (ptr->m_userData);
									attibuteUsed[index] = 1;
									m_uvArray[index].m_u0 = u;
									m_uvArray[index].m_v0 = v;
								}
								ptr = ptr->m_twin->m_next;
							} while (ptr != face->m_prev);
						}

						face->m_mark = mark;
						face->m_next->m_mark = mark;
						face->m_prev->m_mark = mark;

						if (face->m_next->m_twin->m_incidentFace > 0) {
							stack.Append(face->m_next->m_twin);
						}

						if (face->m_prev->m_twin->m_incidentFace > 0) {
							stack.Append(face->m_prev->m_twin);
						}
					}
				}
			}
		}
	}
*/

	ndInt32 GetAlphaLandaIndex (const ndEdge* const edge) const
	{
		return edge->m_incidentFace - 1;
	}

	void AnglesFromUV ()
	{
		m_allocated = true;
		m_triangleAngles = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (3 * m_trianglesCount * sizeof (ndFloat64));

		// calculate initial beta angle for each triangle
		for (ndInt32 i = 0; i < m_trianglesCount; i ++) {
			ndEdge* const edge = m_triangles[i];

			const ndBigVector& p0 = m_mesh->GetVertex(edge->m_incidentVertex);
			const ndBigVector& p1 = m_mesh->GetVertex(edge->m_next->m_incidentVertex);
			const ndBigVector& p2 = m_mesh->GetVertex(edge->m_prev->m_incidentVertex);

			ndBigVector e10 (p1 - p0);
			ndBigVector e20 (p2 - p0);
			ndBigVector e12 (p2 - p1);
			dAssert(e10.m_w == ndFloat32(0.0f));
			dAssert(e20.m_w == ndFloat32(0.0f));
			dAssert(e12.m_w == ndFloat32(0.0f));

			e10 = e10.Scale (ndFloat64 (1.0) / sqrt (e10.DotProduct(e10).GetScalar()));
			e20 = e20.Scale (ndFloat64 (1.0) / sqrt (e20.DotProduct(e20).GetScalar()));
			e12 = e20.Scale (ndFloat64 (1.0) / sqrt (e12.DotProduct(e12).GetScalar()));

			m_triangleAngles[i * 3 + 0] = acos (dClamp(e10.DotProduct(e20).GetScalar(), ndFloat64 (-1.0f), ndFloat64 (1.0f)));
			m_triangleAngles[i * 3 + 1] = acos (dClamp(e10.DotProduct(e20).GetScalar(), ndFloat64 (-1.0f), ndFloat64 (1.0f)));
			m_triangleAngles[i * 3 + 2] = dgABF_PI - m_triangleAngles[i * 3 + 0] - m_triangleAngles[i * 3 + 1];
		}
	}

/*
	// objective function
	f[u2_,v2_,u3_,v3_,u4_,v4_,u5_,v5_,u6_,v6_] := 
		(cos[a0] * sin[b0] * (u1 - u0) + sin[a0] * sin[b0] * (v1 - v0) - sin[c0] * (u6 - u0)) ^ 2 +
		(cos[a0] * sin[b0] * (v1 - v0) + sin[a0] * sin[b0] * (u1 - u0) - sin[c0] * (v6 - v0)) ^ 2 + 
		(cos[a1] * sin[b1] * (u2 - u0) + sin[a1] * sin[b1] * (v2 - v0) - sin[c1] * (u1 - u0)) ^ 2 +
		(cos[a1] * sin[b1] * (v2 - v0) + sin[a1] * sin[b1] * (u2 - u0) - sin[c1] * (v1 - v0)) ^ 2 + 
		(cos[a2] * sin[b2] * (u5 - u0) + sin[a2] * sin[b2] * (v5 - v0) - sin[c2] * (u2 - u0)) ^ 2 +
		(cos[a2] * sin[b2] * (v5 - v0) + sin[a2] * sin[b2] * (u5 - u0) - sin[c2] * (v2 - v0)) ^ 2 + 
		(cos[a3] * sin[b3] * (u2 - u1) + sin[a3] * sin[b3] * (v2 - v1) - sin[c3] * (u3 - u1)) ^ 2 +
		(cos[a3] * sin[b3] * (v2 - v1) + sin[a3] * sin[b3] * (u2 - u1) - sin[c3] * (v3 - v1)) ^ 2 + 
		(cos[a4] * sin[b4] * (u3 - u1) + sin[a4] * sin[b4] * (v3 - v1) - sin[c4] * (u4 - u1)) ^ 2 +
		(cos[a4] * sin[b4] * (v3 - v1) + sin[a4] * sin[b4] * (u3 - u1) - sin[c4] * (v4 - v1)) ^ 2 + 
		(cos[a5] * sin[b5] * (u4 - u1) + sin[a5] * sin[b5] * (v4 - v1) - sin[c5] * (u6 - u1)) ^ 2 +
		(cos[a5] * sin[b5] * (v4 - v1) + sin[a5] * sin[b5] * (u4 - u1) - sin[c5] * (v6 - v1)) ^ 2 + 
		(cos[a6] * sin[b6] * (u4 - u2) + sin[a6] * sin[b6] * (v4 - v2) - sin[c6] * (u3 - u2)) ^ 2 +
		(cos[a6] * sin[b6] * (v4 - v2) + sin[a6] * sin[b6] * (u4 - u2) - sin[c6] * (v3 - v2)) ^ 2 + 
		(cos[a7] * sin[b7] * (u5 - u2) + sin[a7] * sin[b7] * (v5 - v2) - sin[c7] * (u4 - u2)) ^ 2 +
		(cos[a7] * sin[b7] * (v5 - v2) + sin[a7] * sin[b7] * (u5 - u2) - sin[c7] * (v4 - v2)) ^ 2 + 
		(cos[a8] * sin[b8] * (u5 - u4) + sin[a8] * sin[b8] * (v5 - v4) - sin[c8] * (u6 - u4)) ^ 2 +
		(cos[a8] * sin[b8] * (v5 - v4) + sin[a8] * sin[b8] * (u5 - u4) - sin[c8] * (v6 - v4)) ^ 2
*/
	void TraceObjectiveFunction() const
	{
		DG_DEBUG_UV (("f["));
		for (ndInt32 i = 2; i < m_mesh->GetVertexCount(); i ++) {
			DG_DEBUG_UV (("u%d_,v%d_", i, i));
			if (i != (m_mesh->GetVertexCount() - 1)) {
				DG_DEBUG_UV ((","));
			}
		}
		DG_DEBUG_UV (("] := \n"));

		for (ndInt32 i = 0; i < m_trianglesCount; i ++) {
			ndEdge* const face = m_triangles[i];

			ndInt32 v0 = face->m_incidentVertex;
			ndInt32 v1 = face->m_next->m_incidentVertex;
			ndInt32 v2 = face->m_prev->m_incidentVertex;
			(void)(v0);
			(void)(v1);
			(void)(v2);
			DG_DEBUG_UV (("(cos[a%d] * sin[b%d] * (u%d - u%d) + sin[a%d] * sin[b%d] * (v%d - v%d) - sin[c%d] * (u%d - u%d)) ^ 2 +\n", i, i, v1, v0, i, i, v1, v0, i, v2, v0));
			DG_DEBUG_UV (("(cos[a%d] * sin[b%d] * (v%d - v%d) + sin[a%d] * sin[b%d] * (u%d - u%d) - sin[c%d] * (v%d - v%d)) ^ 2", i, i, v1, v0, i, i, v1, v0, i, v2, v0));
			if (i != (m_trianglesCount - 1)) {
				DG_DEBUG_UV ((" + \n"));
			} else {
				DG_DEBUG_UV (("\n"));
			}
		}
	}

	ndFloat64 CalculateExpression_U_face (const ndEdge* const face) const
	{
		ndInt32 faceIndex = GetAlphaLandaIndex (face);
		ndEdge* const faceStartEdge = m_triangles[faceIndex];

		ndInt32 uvIndex0 = ndInt32 (faceStartEdge->m_incidentVertex);
		ndInt32 uvIndex1 = ndInt32 (faceStartEdge->m_next->m_incidentVertex);
		ndInt32 uvIndex2 = ndInt32 (faceStartEdge->m_prev->m_incidentVertex);

		ndInt32 alphaIndex0 = faceIndex * 3;
		ndInt32 alphaIndex1 = faceIndex * 3 + 1;
		ndInt32 alphaIndex2 = faceIndex * 3 + 2;

		DG_DEBUG_UV (("("));
		DG_DEBUG_UV (("cos(a%d) * sin(b%d) * (u%d - u%d) + ", faceIndex, faceIndex, uvIndex1, uvIndex0));
		DG_DEBUG_UV (("sin(a%d) * sin(b%d) * (v%d - v%d) + ", faceIndex, faceIndex, uvIndex1, uvIndex0));
		DG_DEBUG_UV (("sin(c%d) * (u%d - u%d)", faceIndex, uvIndex2, uvIndex0));
		DG_DEBUG_UV ((")"));

		ndFloat64 gradient = m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1] * (m_uvArray[uvIndex1 * 2] - m_uvArray[uvIndex0 * 2]) + 
							 m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1] * (m_uvArray[uvIndex1 * 2 + 1] - m_uvArray[uvIndex0 * 2 + 1]) +
							 m_sinTable[alphaIndex2] * (m_uvArray[uvIndex2 * 2] - m_uvArray[uvIndex0 * 2]);
		return gradient;
	}

	ndFloat64 CalculateExpression_V_face (const ndEdge* const face) const
	{
		ndInt32 faceIndex = GetAlphaLandaIndex (face);
		ndEdge* const faceStartEdge = m_triangles[faceIndex];

		ndInt32 uvIndex0 = ndInt32 (faceStartEdge->m_incidentVertex);
		ndInt32 uvIndex1 = ndInt32 (faceStartEdge->m_next->m_incidentVertex);
		ndInt32 uvIndex2 = ndInt32 (faceStartEdge->m_prev->m_incidentVertex);

		ndInt32 alphaIndex0 = faceIndex * 3;
		ndInt32 alphaIndex1 = faceIndex * 3 + 1;
		ndInt32 alphaIndex2 = faceIndex * 3 + 2;

		DG_DEBUG_UV (("("));
		DG_DEBUG_UV (("cos(a%d) * sin(b%d) * (v%d - v%d) + ", faceIndex, faceIndex, uvIndex1, uvIndex0));
		DG_DEBUG_UV (("sin(a%d) * sin(b%d) * (u%d - u%d) + ", faceIndex, faceIndex, uvIndex1, uvIndex0));
		DG_DEBUG_UV (("sin(c%d) * (v%d - v%d)", faceIndex, uvIndex2, uvIndex0));
		DG_DEBUG_UV ((")"));

		ndFloat64 gradient = m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1] * (m_uvArray[uvIndex1 * 2 + 1] - m_uvArray[uvIndex0 * 2 + 1]) + 
							 m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1] * (m_uvArray[uvIndex1 * 2] - m_uvArray[uvIndex0 * 2]) +
							 m_sinTable[alphaIndex2] * (m_uvArray[uvIndex2 * 2 + 1] - m_uvArray[uvIndex0 * 2 + 1]);
		return gradient;
	}


	ndFloat64 CalculateGradient_U_Coefficent (const ndEdge* const edge, bool u) const
	{
		DG_DEBUG_UV (("("));
		ndInt32 faceIndex = GetAlphaLandaIndex (edge);
		ndEdge* const faceStartEdge = m_triangles[faceIndex];

		ndFloat64 gradient = ndFloat64 (0.0f);

		ndInt32 alphaIndex0 = faceIndex * 3;
		ndInt32 alphaIndex1 = faceIndex * 3 + 1;
		ndInt32 alphaIndex2 = faceIndex * 3 + 2;
		if (faceStartEdge == edge) {
			if (u) {
				DG_DEBUG_UV ((" - cos(a%d) * sin(b%d) - sin(c%d)", faceIndex, faceIndex, faceIndex));
				gradient = - m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1] - m_sinTable[alphaIndex2];
			} else {
				DG_DEBUG_UV ((" - sin(a%d) * sin(b%d) - sin(c%d)", faceIndex, faceIndex, faceIndex));
				gradient = - m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1] - m_sinTable[alphaIndex2];
			}
		} else if (faceStartEdge->m_next == edge) {
			if (u) {
				DG_DEBUG_UV (("cos(a%d) * sin(b%d)", faceIndex, faceIndex));
				gradient = m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1];
			} else {
				DG_DEBUG_UV (("sin(a%d) * sin(b%d)", faceIndex, faceIndex));
				gradient = m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1];
			}
		} else {
			dAssert (faceStartEdge->m_prev == edge);
			if (u) {
				DG_DEBUG_UV ((" - sin(c%d)", faceIndex));
				gradient = -m_sinTable[alphaIndex2];
			} else {
				DG_DEBUG_UV (("0"));
			}
		}
		DG_DEBUG_UV ((")"));
		return gradient;
	}

	ndFloat64 CalculateGradient_V_Coefficent (const ndEdge* const edge, bool u) const
	{
		DG_DEBUG_UV (("("));
		ndInt32 faceIndex = GetAlphaLandaIndex (edge);
		ndEdge* const faceStartEdge = m_triangles[faceIndex];

		ndInt32 alphaIndex0 = faceIndex * 3;
		ndInt32 alphaIndex1 = faceIndex * 3 + 1;
		ndInt32 alphaIndex2 = faceIndex * 3 + 2;

		ndFloat64 gradient = ndFloat64 (0.0f);
		if (faceStartEdge == edge) {
			if (!u) {
				DG_DEBUG_UV ((" - cos(a%d) * sin(b%d) - sin(c%d)", faceIndex, faceIndex, faceIndex));
				gradient = - m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1] - m_sinTable[alphaIndex2];
			} else {
				DG_DEBUG_UV ((" - sin(a%d) * sin(b%d) - sin(c%d)", faceIndex, faceIndex, faceIndex));
				gradient = - m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1] - m_sinTable[alphaIndex2];
			}
		} else if (faceStartEdge->m_next == edge) {
			if (!u) {
				DG_DEBUG_UV (("cos(a%d) * sin(b%d)", faceIndex, faceIndex));
				gradient = m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1];
			} else {
				DG_DEBUG_UV (("sin(a%d) * sin(b%d)", faceIndex, faceIndex));
				gradient = m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1];
			}
		} else {
			dAssert (faceStartEdge->m_prev == edge);
			if (!u) {
				DG_DEBUG_UV ((" - sin(c%d)", faceIndex));
				gradient = -m_sinTable[alphaIndex2];
			} else {
				DG_DEBUG_UV (("0"));
			}
		}
		DG_DEBUG_UV ((")"));
		return gradient;
	}


	ndFloat64 CalculateHessianExpression_U_V (const ndEdge* const face) const
	{
		ndInt32 faceIndex = GetAlphaLandaIndex (face);
		//ndEdge* const faceStartEdge = m_triangles[faceIndex];
		//ndInt32 uvIndex0 = ndInt32 (faceStartEdge->m_incidentVertex);
		//ndInt32 uvIndex1 = ndInt32 (faceStartEdge->m_next->m_incidentVertex);
		//ndInt32 uvIndex2 = ndInt32 (faceStartEdge->m_prev->m_incidentVertex);
		ndInt32 alphaIndex0 = faceIndex * 3;
		ndInt32 alphaIndex1 = faceIndex * 3 + 1;
		//ndInt32 alphaIndex2 = faceIndex * 3 + 2;
		DG_DEBUG_UV (("( - sin(a%d) * sin(b%d))", faceIndex, faceIndex));
		return - m_sinTable[alphaIndex0] * m_sinTable[alphaIndex1];
	}


	ndFloat64 CalculateHessianExpression_V_V (const ndEdge* const face) const
	{
		ndInt32 faceIndex = GetAlphaLandaIndex (face);
		//ndEdge* const faceStartEdge = m_triangles[faceIndex];
		//ndInt32 uvIndex0 = ndInt32 (faceStartEdge->m_incidentVertex);
		//ndInt32 uvIndex1 = ndInt32 (faceStartEdge->m_next->m_incidentVertex);
		//ndInt32 uvIndex2 = ndInt32 (faceStartEdge->m_prev->m_incidentVertex);

		ndInt32 alphaIndex0 = faceIndex * 3;
		ndInt32 alphaIndex1 = faceIndex * 3 + 1;
		ndInt32 alphaIndex2 = faceIndex * 3 + 2;
		DG_DEBUG_UV (("(- cos(a%d) * sin(b%d) - sin(c%d))", faceIndex, faceIndex, faceIndex));
		return - m_cosTable[alphaIndex0] * m_sinTable[alphaIndex1] - m_sinTable[alphaIndex2];
	}


	void CalculateGradientU (ndInt32 vertexIndex)
	{
		// calculate U Gradient derivative
		const ndEdge* const vertex = m_vertexEdge[vertexIndex];
		DG_DEBUG_UV (("du%d =\n", vertexIndex));
		ndFloat64 gradient = ndFloat64 (0.0f);
		const ndEdge* ptr = vertex;
		do {
			if (ptr->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_U_Coefficent (ptr, true) ;
				DG_DEBUG_UV ((" * "));
				gradient += a * CalculateExpression_U_face (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_U_Coefficent (ptr, false);
				DG_DEBUG_UV ((" * "));
				gradient += a * CalculateExpression_V_face (ptr);
				DG_DEBUG_UV ((" +\n"));
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);
		m_gradients[2 * vertexIndex] = - gradient;
		DG_DEBUG_UV (("\n"));

		// calculate diagonal derivative
		DG_DEBUG_UV (("H(u%d,u%d) =\n", vertexIndex, vertexIndex));
		ndFloat64 diagonal = ndFloat64 (0.0f);
		ptr = vertex;
		do {
			if (ptr->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 diag = CalculateGradient_U_Coefficent (ptr, true);
				diagonal += diag * diag;
				DG_DEBUG_UV (("^2 +\n"));

				DG_DEBUG_UV (("2 * "));
				diag = CalculateGradient_U_Coefficent (ptr, false);
				diagonal += diag * diag;
				DG_DEBUG_UV (("^2 +\n"));
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);
		dAssert (diagonal > ndFloat32 (0.0f));

		m_hessianCoLumnValue[m_matrixElementCount] = diagonal;
		m_hessianCoLumnIndex[m_matrixElementCount] = vertexIndex * 2 + 0;
		m_matrixElementCount ++;
		m_diagonal[2 * vertexIndex] = diagonal;
		DG_DEBUG_UV (("\n"));

		// calculate of diagonal UiVi derivative
		DG_DEBUG_UV (("H(u%d,v%d) =\n", vertexIndex, vertexIndex));
		ndFloat64 hessianUV = ndFloat64 (0.0);
		ptr = vertex;
		do {
			if (ptr->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_U_Coefficent (ptr, true);
				DG_DEBUG_UV ((" * "));
				hessianUV += a * CalculateHessianExpression_U_V (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_U_Coefficent (ptr, false);
				DG_DEBUG_UV ((" * "));
				hessianUV += a * CalculateHessianExpression_V_V (ptr);
				DG_DEBUG_UV ((" +\n"));
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);

		m_hessianCoLumnValue[m_matrixElementCount] = hessianUV;
		m_hessianCoLumnIndex[m_matrixElementCount] = vertexIndex * 2 + 1;
		m_matrixElementCount ++;
		DG_DEBUG_UV (("\n"));


/*
		// calculate off diagonal partial derivatives
		ptr = vertex;
		do {
			// derivative respect to U(i, j)
			ndInt32 vertexIndex2 = ptr->m_twin->m_incidentVertex;
			DG_DEBUG_UV (("H(u%d,u%d) =\n", vertexIndex, vertexIndex2));
			if (ptr->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_U_Coefficent (ptr, true);
				DG_DEBUG_UV ((" * "));
//				gradient += diag * TraceExpression_U_face (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_U_Coefficent (ptr, false);
				DG_DEBUG_UV ((" * "));
//				gradient += diag * TraceExpression_V_face (ptr);
				DG_DEBUG_UV ((" +\n"));
			}

			if (ptr->m_twin->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_U_Coefficent (ptr->m_twin->m_next, true);
				DG_DEBUG_UV ((" * "));
				//				gradient += diag * TraceExpression_U_face (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_U_Coefficent (ptr->m_twin->m_next, false);
				DG_DEBUG_UV ((" * "));
				//				gradient += diag * TraceExpression_V_face (ptr);
				DG_DEBUG_UV ((" +\n"));
			}

			// derivative respect to V(i, j)
			DG_DEBUG_UV (("H(u%d,v%d) =\n", vertexIndex, vertexIndex2));
			if (ptr->m_incidentFace > 0) {

				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_U_Coefficent (ptr, true);
				DG_DEBUG_UV ((" * "));
				//				gradient += diag * TraceExpression_U_face (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_U_Coefficent (ptr, false);
				DG_DEBUG_UV ((" * "));
				//				gradient += diag * TraceExpression_V_face (ptr);
				DG_DEBUG_UV ((" +\n"));

			}

			if (ptr->m_twin->m_incidentFace > 0) {

				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_U_Coefficent (ptr->m_twin->m_next, true);
				DG_DEBUG_UV ((" * "));
				//				gradient += diag * TraceExpression_U_face (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_U_Coefficent (ptr->m_twin->m_next, false);
				DG_DEBUG_UV ((" * "));
				//				gradient += diag * TraceExpression_V_face (ptr);
				DG_DEBUG_UV ((" +\n"));

			}

			DG_DEBUG_UV (("\n"));
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);
*/
	}


	void CalculateGradientV (ndInt32 vertexIndex)
	{
		// calculate U Gradient derivative
		const ndEdge* const vertex = m_vertexEdge[vertexIndex];
		DG_DEBUG_UV (("dv%d =\n", vertexIndex));
			
		ndFloat64 gradient = ndFloat64 (0.0f);
		const ndEdge* ptr = vertex;
		do {
			if (ptr->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 a = CalculateGradient_V_Coefficent (ptr, true);
				DG_DEBUG_UV ((" * "));
				gradient += a * CalculateExpression_U_face (ptr);
				DG_DEBUG_UV ((" +\n"));

				DG_DEBUG_UV (("2 * "));
				a = CalculateGradient_V_Coefficent (ptr, false);
				DG_DEBUG_UV ((" * "));
				gradient += a * CalculateExpression_V_face (ptr);
				DG_DEBUG_UV ((" +\n"));
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);
		m_gradients[2 * vertexIndex + 1] = - gradient;
		DG_DEBUG_UV (("\n"));


		// calculate diagonal derivative
		DG_DEBUG_UV (("H(v%d,v%d) =\n", vertexIndex, vertexIndex));
		ndFloat64 diagonal = ndFloat64 (0.0f);
		ptr = vertex;
		do {
			if (ptr->m_incidentFace > 0) {
				DG_DEBUG_UV (("2 * "));
				dAssert (ptr->m_incidentVertex == vertexIndex);
				ndFloat64 diag = CalculateGradient_V_Coefficent (ptr, true);
				diagonal += diag * diag;
				DG_DEBUG_UV (("^2 +\n"));

				DG_DEBUG_UV (("2 * "));
				diag = CalculateGradient_V_Coefficent (ptr, false);
				diagonal += diag * diag;
				DG_DEBUG_UV (("^2 +\n"));
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != vertex);
		dAssert (diagonal > ndFloat32 (0.0f));

		m_hessianCoLumnValue[m_matrixElementCount] = diagonal;
		m_hessianCoLumnIndex[m_matrixElementCount] = vertexIndex * 2 + 1;
		m_matrixElementCount ++;
		m_diagonal[2 * vertexIndex + 1] = diagonal;
		DG_DEBUG_UV (("\n"));





	}



	void CalculateGradientVectorAndHessianMatrix ()
	{
		// trace objective function
//		TraceObjectiveFunction();

		// trace gradients
		DG_DEBUG_UV (("\n"));
		ndInt32 count = m_mesh->GetVertexCount();
		for (ndInt32 i = 0; i < count; i ++) {
			CalculateGradientU (i);
			CalculateGradientV (i);
		}
		DG_DEBUG_UV (("\n"));
	}

	void InversePrecoditionerTimeVector (ndFloat64* const out, const ndFloat64* const v) const
	{
		const ndInt32 count = m_mesh->GetVertexCount();
		for (ndInt32 i = 0; i < count; i ++) {
			out[2 * i + 0] = m_pinnedPoints[i] * v[i * 2 + 0] / m_diagonal[2 * i + 0];
			out[2 * i + 1] = m_pinnedPoints[i] * v[i * 2 + 1] / m_diagonal[2 * i + 1];
		}
	}

	void MatrixTimeVector (ndFloat64* const out, const ndFloat64* const v) const
	{
/*
		const ndInt32 count = m_mesh->GetVertexCount();
		for (ndInt32 i = 0; i < count; i ++) {
			ndEdge* const vertex = m_vertexEdge[i];
			dAssert (vertex->m_incidentVertex == i);
			out[i * 2 + 0] = m_diagonal[2 * i + 0] * v[2 * i + 0];
			out[i * 2 + 1] = m_diagonal[2 * i + 1] * v[2 * i + 1];
		}
		DG_DEBUG_UV (("\n"));
		ndInt32 count = m_mesh->GetVertexCount();
		for (ndInt32 i = 0; count; i ++) {
			CalculateHessianDiagonalUU (i);
		}
		DG_DEBUG_UV (("\n"));
*/
	}

	void LagrangeOptimization()
	{
		CalculateGradientVectorAndHessianMatrix ();
		ndStack<ndFloat64> r0(2 * m_mesh->GetVertexCount());
		ndStack<ndFloat64> z0(2 * m_mesh->GetVertexCount());
		ndStack<ndFloat64> p0(2 * m_mesh->GetVertexCount());
		ndStack<ndFloat64> q0(2 * m_mesh->GetVertexCount());
		SetBuffers(&r0[0], &z0[0], &p0[0], &q0[0]);
		Solve(2 * m_mesh->GetVertexCount(), dgABF_UV_TOL2, m_uvArray, m_gradients);
		SetBuffers(nullptr, nullptr, nullptr, nullptr);
	}

	dgArray<ndInt32> m_hessianCoLumnIndex;
	dgArray<ndFloat64> m_hessianCoLumnValue;
	ndMeshEffect* m_mesh;
	ndEdge** m_triangles;
	ndEdge** m_vertexEdge;
	ndFloat64* m_uvArray;
	ndFloat64* m_sinTable;
	ndFloat64* m_cosTable;
	ndFloat64* m_gradients;
	ndFloat64* m_diagonal;
	ndFloat64* m_triangleAngles;
	const ndFloat64* m_pinnedPoints;

	ndInt32 m_trianglesCount;
	ndInt32 m_matrixElementCount;
	bool m_allocated;
};

class dgAngleBasedFlatteningMapping: public dgSymmetricConjugateGradientSolver<ndFloat64>
{
	public: 
	dgAngleBasedFlatteningMapping (ndMeshEffect* const mesh, ndInt32 material, dgReportProgress progressReportCallback, void* const userData)
		:m_mesh(mesh)
		,m_progressReportUserData(userData)
		,m_progressReportCallback(progressReportCallback)
	{
dAssert (0);
/*
		AllocVectors();
		InitEdgeVector();
		CalculateInitialAngles ();
		LagrangeOptimization();

		ndEdge* const face = m_betaEdge[0];
		ndEdge* ptr = face;
		do {
			if (ptr->m_incidentFace > 0) {
				ndInt32 index = ndInt32 (ptr->m_userData);
				ndMeshEffect::dgVertexAtribute& attribute = m_mesh->GetAttribute (index);
				attribute.m_u0 = ndFloat32 (0.0f);
				attribute.m_v0 = ndFloat32 (0.0f);
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != face);

		ndEdge* const twinFace = face->m_twin;
		const ndBigVector& p0 = m_mesh->GetVertex(face->m_incidentVertex);
		const ndBigVector& p1 = m_mesh->GetVertex(twinFace->m_incidentVertex);
		ndBigVector p10 (p1 - p0);
		dAssert(p10.m_w == ndFloat32(0.0f));
		ndFloat64 e0length = sqrt (p10.DotProduct(p10).GetScalar());

		ptr = twinFace;
		do {
			if (ptr->m_incidentFace > 0) {
				ndInt32 index = ndInt32 (ptr->m_userData);
				ndMeshEffect::dgVertexAtribute& attribute = m_mesh->GetAttribute (index);
				attribute.m_u0 = e0length;
				attribute.m_v0 = ndFloat32 (0.0f);
			}
			ptr = ptr->m_twin->m_next;
		} while (ptr != twinFace);

		DeleteAuxiliaryVectors();

		m_deltaVariables[0] = 0.0f;
		m_deltaVariables[1] = 0.0f;
		for (ndInt32 i = 2; i < m_totalVariablesCount; i ++) {
			m_deltaVariables[i] = 1.0f;
		}
		dgTriangleAnglesToUV anglesToUV (mesh, material, progressReportCallback, userData, m_deltaVariables, m_variables);
*/
	}

	~dgAngleBasedFlatteningMapping()
	{
		m_mesh->GetAllocator()->FreeLow (m_variables);
		m_mesh->GetAllocator()->FreeLow (m_deltaVariables);
	}


	void AllocVectors()
	{
		CalculateNumberOfVariables();
		ndInt32 vertexCount = m_mesh->GetVertexCount();

		// alloc intermediate vectors
		m_betaEdge = (ndEdge**) m_mesh->GetAllocator()->MallocLow(m_anglesCount * sizeof (ndEdge*));
		m_interiorIndirectMap = (ndInt32*) m_mesh->GetAllocator()->MallocLow (vertexCount * sizeof (ndInt32));
		m_beta = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (ndFloat64));
		m_weight= (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (ndFloat64));
		m_sinTable = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (ndFloat64));
		m_cosTable = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (ndFloat64));
		m_gradients = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_totalVariablesCount * sizeof (ndFloat64));
		
		// allocate angle and internal vertex vector
		m_variables = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_totalVariablesCount * sizeof (ndFloat64));
		m_deltaVariables = (ndFloat64*) m_mesh->GetAllocator()->MallocLow (m_totalVariablesCount * sizeof (ndFloat64));
	}

	void DeleteAuxiliaryVectors()
	{
		// delete intermediate vectors
		m_mesh->GetAllocator()->FreeLow (m_betaEdge);
		m_mesh->GetAllocator()->FreeLow (m_interiorIndirectMap);
		m_mesh->GetAllocator()->FreeLow (m_sinTable);
		m_mesh->GetAllocator()->FreeLow (m_cosTable);
		m_mesh->GetAllocator()->FreeLow (m_beta);
		m_mesh->GetAllocator()->FreeLow (m_weight);
		m_mesh->GetAllocator()->FreeLow (m_gradients);
		
		m_beta = nullptr;
		m_weight = nullptr;
		m_betaEdge = nullptr;
		m_sinTable = nullptr;
		m_cosTable = nullptr;
		m_gradients = nullptr;
		m_interiorIndirectMap = nullptr;
	}


	void CalculateNumberOfVariables()
	{
		//m_mesh->SaveOFF("xxx.off");
		m_anglesCount = 0;
		m_triangleCount = 0;
		m_interiorVertexCount = 0;

		ndInt32 mark = m_mesh->IncLRU();
		ndMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				ndEdge *ptr = edge;
				do {
					m_anglesCount ++;
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != edge);
				m_triangleCount ++;
				dAssert (edge->m_next->m_next->m_next == edge);
			}
		}

		mark = m_mesh->IncLRU();
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				bool isInterior = true;
				ndEdge *ptr = edge;
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
		ndInt32 count = 0;
		ndInt32 mark = m_mesh->IncLRU();
		ndMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
				ndEdge *ptr = edge;
				do {
					ptr->m_mark = mark;
					m_betaEdge[count] = ptr;
					ptr->m_incidentFace = count + 1;
					count ++;
					dAssert (count <= m_anglesCount);
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}

		count = 0;
		mark = m_mesh->IncLRU();		
		memset (m_interiorIndirectMap, -1, m_mesh->GetVertexCount() * sizeof (m_interiorIndirectMap[0]));
		for (iter.Begin(); iter; iter ++) {
			ndEdge* const edge = &iter.GetNode()->GetInfo();
			if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {

				bool isInterior = true;
				ndEdge* ptr = edge;
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

	ndInt32 GetAlphaLandaIndex (const ndEdge* const edge) const
	{
		return edge->m_incidentFace - 1;
	}

	ndInt32 GetTriangleIndex (const ndInt32 alphaIndex) const
	{
		return alphaIndex / 3 + m_anglesCount;
	}

	ndInt32 GetTriangleIndex (const ndEdge* const edge) const
	{
		return GetAlphaLandaIndex(edge) / 3 + m_anglesCount;
	}

	ndInt32 GetInteriorVertex(const ndEdge* const edge) const
	{
		return m_interiorIndirectMap[edge->m_incidentVertex];
	}

	void CalculateInitialAngles ()
	{
		// calculate initial beta angle for each triangle
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			ndEdge* const edge = m_betaEdge[i];

			const ndBigVector& p0 = m_mesh->GetVertex(edge->m_incidentVertex);
			const ndBigVector& p1 = m_mesh->GetVertex(edge->m_next->m_incidentVertex);
			const ndBigVector& p2 = m_mesh->GetVertex(edge->m_prev->m_incidentVertex);

			ndBigVector e10 (p1 - p0);
			ndBigVector e20 (p2 - p0);

			e10 = e10.Scale (ndFloat64 (1.0) / sqrt (e10.DotProduct(e10).GetScalar()));
			e20 = e20.Scale (ndFloat64 (1.0) / sqrt (e20.DotProduct(e20).GetScalar()));
			dAssert(e10.m_w == ndFloat32(0.0f));
			dAssert(e20.m_w == ndFloat32(0.0f));

			m_beta[i] = acos (dClamp(e10.DotProduct(e20).GetScalar(), ndFloat64 (-1.0f), ndFloat64 (1.0f)));
			dAssert (m_beta[i] > ndFloat64 (0.0f));
		}

		#ifdef _DEBUG
		for (ndInt32 i = 0; i < m_triangleCount; i ++) {
			ndInt32 i0 = i * 3 + 0;
			ndInt32 i1 = i * 3 + 1;
			ndInt32 i2 = i * 3 + 2;
			dAssert (fabs (m_beta[i0] + m_beta[i1] + m_beta[i2] - dgABF_PI) < ndFloat64 (1.0e-6f));
		}
		#endif

		// for each interior vertex apply the scale factor
		ndInt32 mark = m_mesh->IncLRU();
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			ndEdge* const edge = m_betaEdge[i];
			if ((edge->m_mark != mark) && (GetInteriorVertex(edge) >= 0)) {
				ndFloat64 scale = ndFloat64 (0.0f);
				ndEdge* ptr = edge; 
				do {
					ndInt32 index = GetAlphaLandaIndex (ptr);
					dAssert (index >= 0);
					dAssert (index <= m_anglesCount);
					scale += m_beta[index];
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				dAssert (scale > ndFloat32 (0.0f));

				scale = ndFloat64 (2.0f) * dgABF_PI / scale;
				ptr = edge;
				do {
					ndInt32 index = GetAlphaLandaIndex (ptr);
					dAssert (index >= 0);
					dAssert (index <= m_anglesCount);
					m_beta[index] *= scale;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}

		// initialized each alpha lambda to the beta angle and also calcual ethe derivatoe coeficent (2.0 / (betai * betai)) 
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			dAssert (m_beta[i] > ndFloat64 (0.0f));
			m_variables[i] = m_beta[i];
			m_weight[i] = ndFloat64 (2.0f) / (m_beta[i] * m_beta[i]);
		}
	}

	// angular derivative component
	// (wi * (xi - bi) + T0
	// where wi = 2.0 / (bi ^ 2)
	ndFloat64 CalculateAngularGradientDerivative (ndInt32 alphaIndex) const
	{
		ndFloat64 gradient = (m_variables[alphaIndex] - m_beta[alphaIndex]) * m_weight[alphaIndex] + m_variables[GetTriangleIndex(alphaIndex)];
		dAssert (fabs(gradient) < ndFloat64(1.0e10f));
		return gradient;
	}

	// Vi if the the edge is an interior vertex
	ndFloat64 CalculateInteriorVertexGradient (ndInt32 alphaIndex) const
	{
		ndInt32 index = GetInteriorVertex(m_betaEdge[alphaIndex]);
		ndFloat64 gradient = (index != -1) ? m_variables[index] : ndFloat32 (0.0f);
		dAssert (fabs(gradient) < ndFloat64(1.0e10f));
		return gradient;
	}

	// Wj * cos(alpha) * sum (alphai) for eadh previsu or next interior incdent vertex
	ndFloat64 CalculatePlanarityGradient (ndInt32 alphaIndex) const
	{
		ndFloat64 gradient = ndFloat64 (0.0f);

		ndEdge* const incidentEdge = m_betaEdge[alphaIndex];

		if (GetInteriorVertex (incidentEdge->m_next) != -1) {
			ndEdge* const edge = m_betaEdge[GetAlphaLandaIndex(incidentEdge->m_next)];
			ndFloat64 product = m_cosTable[GetAlphaLandaIndex(edge->m_prev)];
			ndEdge* ptr = edge->m_twin->m_next;
			do {
				product *= m_sinTable[GetAlphaLandaIndex(ptr->m_prev)];
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			ndInt32 interiorVertexIndex = GetInteriorVertex (incidentEdge->m_next) + m_interiorVertexCount;
			gradient -= m_variables[interiorVertexIndex] * product;
		}

		if (GetInteriorVertex (incidentEdge->m_prev) != -1) {
			ndEdge* const edge = m_betaEdge[GetAlphaLandaIndex(incidentEdge->m_prev)];
			ndFloat64 product = m_cosTable[GetAlphaLandaIndex(edge->m_next)];
			ndEdge* ptr = edge->m_twin->m_next;
			do {
				product *= m_sinTable[GetAlphaLandaIndex(ptr->m_next)];
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			ndInt32 interiorVertexIndex = GetInteriorVertex (incidentEdge->m_prev) + m_interiorVertexCount;
			gradient += m_variables[interiorVertexIndex] * product;
		}
		dAssert (fabs(gradient) < ndFloat64(1.0e10f));
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
	ndFloat64 CalculateGradientVector ()
	{
		// pre-compute sin cos tables
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			m_sinTable[i] = sin (m_variables[i]);
			m_cosTable[i] = cos (m_variables[i]);
		}

		ndFloat64 gradientNorm = ndFloat64 (0.0f);

		// calculate gradients due to the difference between a matching edge angle and it projected angle msu be mminimal Wei * (Xei - Bei) ^ e = minimal
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			ndFloat64 gradient = CalculateAngularGradientDerivative (i) + CalculateInteriorVertexGradient (i) + CalculatePlanarityGradient (i);
			m_gradients[i] = -gradient;
			gradientNorm += gradient * gradient;
		}

		// calculate gradient due to the equality that the sum on the internal angle of a triangle must add to 180 degree. (Xt0 + Xt1 + Xt2 - pi) = 0
		for (ndInt32 i = 0; i < m_triangleCount; i ++) {
			ndFloat64 gradient = m_variables[i * 3 + 0] + m_variables[i * 3 + 1] + m_variables[i * 3 + 2] - dgABF_PI;
			m_gradients[m_anglesCount + i] = -gradient;
			gradientNorm += gradient * gradient;
		}

		// calculate the gradient due to the equality that the sum of all the angle incident to and interior vertex must be 3060 degree sum (Xvi) - 2 * pi = 0 
		ndInt32 mark = m_mesh->IncLRU();
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			ndEdge* const edge = m_betaEdge[i];

			if ((edge->m_mark != mark) && GetInteriorVertex(edge) != -1) {
				ndInt32 vertexIndex = GetInteriorVertex(edge);
				ndFloat64 gradient = - ndFloat64 (2.0f) * dgABF_PI;

				ndEdge* ptr = edge; 
				do {
					ndInt32 index = GetAlphaLandaIndex(ptr);
					gradient += m_variables[index];
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				m_gradients[vertexIndex] = - gradient;
				gradientNorm += gradient * gradient;
			}
		}

		// calculate the gradient due to the equality that the difference of the product of the sin of the angle to the
		// incident to an interior vertex must be zero product (sin (Xvi + 1) - product (sin (Xvi - 1)  = 0 
		mark = m_mesh->IncLRU();
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			ndEdge* const edge = m_betaEdge[i];

			ndInt32 vertexIndex = GetInteriorVertex(edge);
			if ((edge->m_mark != mark) && (vertexIndex != -1)) {
				vertexIndex += m_interiorVertexCount;
				ndFloat64 partialProdut0 =  ndFloat64 (1.0f);
				ndFloat64 partialProdut1 =  ndFloat64 (1.0f);
				ndEdge* ptr = edge; 
				do {
					ndInt32 index0 = GetAlphaLandaIndex(ptr->m_next);
					ndInt32 index1 = GetAlphaLandaIndex(ptr->m_prev);
					partialProdut0 *= m_sinTable[index0];
					partialProdut1 *= m_sinTable[index1];
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				ndFloat64 gradient = partialProdut0 - partialProdut1;
				m_gradients[vertexIndex] = - gradient;
				gradientNorm += gradient * gradient;
			}
		}

		return gradientNorm;
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
	void MatrixTimeVector (ndFloat64* const out, const ndFloat64* const v) const
	{
		for (ndInt32 i = 0; i < m_interiorVertexCount; i ++) {
			out[i + m_anglesCount + m_triangleCount] = ndFloat64 (0.0f);
			out[i + m_anglesCount + m_triangleCount + m_interiorVertexCount] = ndFloat64 (0.0f);
		}

		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			out[i] = m_weight[i] * v[i];

			ndEdge* const edge = m_betaEdge[i];
			ndInt32 vertexIndex = GetInteriorVertex(edge);
			if (vertexIndex >= 0) {
				out[i] += v[vertexIndex];
				out[vertexIndex] += v[i];
			}
		}

		for (ndInt32 i = 0; i < m_triangleCount; i ++) {
			ndInt32 j = i * 3;
			out[j + 0] += v[i + m_anglesCount];
			out[j + 1] += v[i + m_anglesCount];
			out[j + 2] += v[i + m_anglesCount];
			out[i + m_anglesCount] = v[j + 0] + v[j + 1] +  v[j + 2];
		}

		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			{
				ndEdge* const edge = m_betaEdge[i]->m_prev;
				ndInt32 vertexIndex = GetInteriorVertex(edge);
				if (vertexIndex >= 0) {
					ndInt32 index = GetAlphaLandaIndex(edge->m_next);
					ndFloat64 product = m_cosTable[index];
					ndEdge* ptr = edge->m_twin->m_next; 
					do {
						ndInt32 m = GetAlphaLandaIndex(ptr->m_next);
						product *= m_sinTable[m];
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);
					out[i] += v[vertexIndex + m_interiorVertexCount] * product;
					out[vertexIndex + m_interiorVertexCount] += product * v[i];
				}
			}

			{
				ndEdge* const edge = m_betaEdge[i]->m_next;
				ndInt32 vertexIndex = GetInteriorVertex(edge);
				if (vertexIndex >= 0) {
					ndInt32 index = GetAlphaLandaIndex(edge->m_prev);
					ndFloat64 product = m_cosTable[index];
					ndEdge* ptr = edge->m_twin->m_next; 
					do {
						ndInt32 m = GetAlphaLandaIndex(ptr->m_prev);
						product *= m_sinTable[m];
						ptr = ptr->m_twin->m_next;
					} while (ptr != edge);
					out[i] -= v[vertexIndex + m_interiorVertexCount] * product;
					out[vertexIndex + m_interiorVertexCount] -= product * v[i];
				}
			}
		}
	}

	void InversePrecoditionerTimeVector (ndFloat64* const out, const ndFloat64* const v) const
	{
		for (ndInt32 i = 0; i < m_anglesCount; i ++) {
			out[i] = v[i] / m_weight[i];
		}
		for (ndInt32 i = 0; i < m_triangleCount; i ++) {
			out[i + m_anglesCount] = v[i + m_anglesCount];
		}

		for (ndInt32 i = 0; i < m_interiorVertexCount; i ++) {
			out[i + m_anglesCount + m_triangleCount] = v[i + m_anglesCount + m_triangleCount];
			out[i + m_anglesCount + m_triangleCount + m_interiorVertexCount] = v[i + m_anglesCount + m_triangleCount + m_interiorVertexCount];
		}

		m_progressNum ++;
		if (m_progressReportCallback) {
			if ((m_progressNum & 127) == 127) {
				m_continueExecution = m_progressReportCallback (dMin (ndFloat32 (m_progressNum) / m_progressDen, ndFloat32 (1.0f)), m_progressReportUserData);
			}
		}
	}

	void LagrangeOptimization()
	{
		memset (m_deltaVariables, 0, m_totalVariablesCount * sizeof (ndFloat64));
		memset (&m_variables[m_anglesCount], 0, m_triangleCount * sizeof (ndFloat64));	

		for (ndInt32 i = 0; i < m_interiorVertexCount; i ++) {
			m_variables[i + m_anglesCount + m_triangleCount] = ndFloat32 (1.0f);
			m_variables[i + m_anglesCount + m_triangleCount + m_interiorVertexCount] = ndFloat32 (1.0f);
		}
		
		m_progressNum = 0;
		m_continueExecution = true;

/*
		ndStack<ndFloat64> r0(2 * m_mesh->GetVertexCount());
		ndStack<ndFloat64> z0(2 * m_mesh->GetVertexCount());
		ndStack<ndFloat64> p0(2 * m_mesh->GetVertexCount());
		ndStack<ndFloat64> q0(2 * m_mesh->GetVertexCount());
		SetBuffers(&r0[0], &z0[0], &p0[0], &q0[0]);
		ndFloat64 gradientNorm = CalculateGradientVector ();
		for (ndInt32 iter = 0; (iter < dgABF_MAX_ITERATIONS) && (gradientNorm > dgABF_TOL2) && m_continueExecution; iter++) {
			m_progressDen = m_progressNum + m_totalVariablesCount;
			Solve(m_totalVariablesCount, dgABF_LINEAR_SOLVER_TOL, m_deltaVariables, m_gradients);
			for (ndInt32 i = 0; i < m_totalVariablesCount; i ++) {
				m_variables[i] += m_deltaVariables[i];
			}
			gradientNorm = CalculateGradientVector ();
		}
		SetBuffers(nullptr, nullptr, nullptr, nullptr);
*/

#ifdef _DEBUG
		// calculate gradient due to the equality that the sum on the internal angle of a triangle must add to 180 degree. (Xt0 + Xt1 + Xt2 - pi) = 0
//		for (ndInt32 i = 0; i < m_triangleCount; i ++) {
//			ndFloat64 gradient = m_variables[i * 3 + 0] + m_variables[i * 3 + 1] + m_variables[i * 3 + 2] - dgABF_PI;
//			dAssert (fabs (gradient) < ndFloat64 (1.0e-2f));
//		}
#endif
	}

	ndMeshEffect* m_mesh;
	ndEdge** m_betaEdge;
	ndInt32* m_interiorIndirectMap;

	ndFloat64* m_beta;
	ndFloat64* m_weight;
	ndFloat64* m_sinTable;
	ndFloat64* m_cosTable;
	ndFloat64* m_variables;
	ndFloat64* m_gradients;
	ndFloat64* m_deltaVariables;

	ndInt32 m_anglesCount;
	ndInt32 m_triangleCount;
	ndInt32 m_interiorVertexCount;
	ndInt32 m_totalVariablesCount;

	void* m_progressReportUserData;
	dgReportProgress m_progressReportCallback;
	mutable ndInt32 m_progressNum;
	mutable ndInt32 m_progressDen;
	mutable bool m_continueExecution;
};	


/*
void ndMeshEffect::ClearAttributeArray ()
{
	dAssert(0);

    ndStack<dgVertexAtribute>attribArray (m_pointCount);

    memset (&attribArray[0], 0, m_pointCount * sizeof (dgVertexAtribute));
    ndInt32 mark = IncLRU();
    dPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        ndEdge* const edge = &(*iter);
        if (edge->m_mark < mark){
            ndEdge* ptr = edge;

            ndInt32 index = ptr->m_incidentVertex;
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
*/



void ndMeshEffect::CylindricalMapping (ndInt32 cylinderMaterial, ndInt32 capMaterial, const ndMatrix& uvAligment)
{
	ndBigVector origin (GetOrigin());
	ndStack<ndBigVector> buffer(m_points.m_vertex.m_count);
	ndBigVector pMin(ndFloat64(1.0e10f), ndFloat64(1.0e10f), ndFloat64(1.0e10f), ndFloat64(0.0f));
	ndBigVector pMax(ndFloat64(-1.0e10f), ndFloat64(-1.0e10f), ndFloat64(-1.0e10f), ndFloat64(0.0f));

	for (ndInt32 i = 0; i < m_points.m_vertex.m_count; i ++) {
		buffer[i] = uvAligment.RotateVector (m_points.m_vertex[i] - origin);
		const ndBigVector& tmp = buffer[i];
		pMin.m_x = dMin (pMin.m_x, tmp.m_x);
		pMax.m_x = dMax (pMax.m_x, tmp.m_x);
		pMin.m_y = dMin (pMin.m_y, tmp.m_y);
		pMax.m_y = dMax (pMax.m_y, tmp.m_y);
		pMin.m_z = dMin (pMin.m_z, tmp.m_z);
		pMax.m_z = dMax (pMax.m_z, tmp.m_z);
	}

	ndStack<ndBigVector>cylinder (m_points.m_vertex.m_count);
    ndBigVector scale (ndFloat64 (1.0f)/ (pMax.m_x - pMin.m_x), ndFloat64 (1.0f)/ (pMax.m_y - pMin.m_y), ndFloat64 (1.0f)/ (pMax.m_z - pMin.m_z), ndFloat64 (0.0f));
    for (ndInt32 i = 0; i < m_points.m_vertex.m_count; i ++) {
		ndBigVector point (buffer[i]);
		ndFloat64 u = (point.m_x - pMin.m_x) * scale.m_x;

		dAssert(point.m_w == ndFloat32(0.0f));
		dAssert(point.DotProduct(point).GetScalar() > ndFloat32 (0.0f));
		point = point.Normalize();
		ndFloat64 v = ndAtan2 (point.m_y, point.m_z);

		v = v + ndPi;
		cylinder[i].m_x = u;
		cylinder[i].m_y = v;
    }

	UnpackAttibuteData();
	m_attrib.m_uv0Channel.Reserve(m_attrib.m_pointChannel.m_count);
	m_attrib.m_materialChannel.Reserve(m_attrib.m_pointChannel.m_count);

    ndPolyhedra::Iterator iter (*this);	
    for(iter.Begin(); iter; iter ++){
        ndEdge* const edge = &(*iter);
		dAttibutFormat::dgUV uv;
		uv.m_u = ndFloat32(cylinder[edge->m_incidentVertex].m_x);
		uv.m_v = ndFloat32(cylinder[edge->m_incidentVertex].m_y);
		m_attrib.m_uv0Channel[ndInt32(edge->m_userData)] = uv;
		m_attrib.m_materialChannel[ndInt32(edge->m_userData)] = cylinderMaterial;
    }

    ndInt32 mark = IncLRU ();
    for(iter.Begin(); iter; iter ++){
        ndEdge* const edge = &(*iter);
        if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
			dAttibutFormat::dgUV uvRef(m_attrib.m_uv0Channel[ndInt32(edge->m_userData)]);
			ndFloat32 UVrefSin = ndSin(uvRef.m_v);
			ndFloat32 UVrefCos = ndCos(uvRef.m_v);
			ndEdge* ptr = edge;
            do {
				ptr->m_mark = mark;
				dAttibutFormat::dgUV uv(m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)]);
				ndFloat32 sinAngle = UVrefCos * ndSin(uv.m_v) - UVrefSin * ndCos(uv.m_v);
				ndFloat32 cosAngle = UVrefCos * ndCos(uv.m_v) + UVrefSin * ndSin(uv.m_v);
				ndFloat32 deltaAngle = ndAtan2(sinAngle, cosAngle);
				uv.m_v = (uvRef.m_v + deltaAngle) / dPi2;
				m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)] = uv;
                ptr = ptr->m_next;
            } while (ptr != edge);
        }
    }

    // apply cap mapping
    mark = IncLRU ();
    for(iter.Begin(); iter; iter ++){
        ndEdge* const edge = &(*iter);
		if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) {
			ndVector p0(buffer[edge->m_incidentVertex]);
			ndVector p1(buffer[edge->m_next->m_incidentVertex]);

			ndVector e1(p1 - p0);
			ndBigVector normal(ndFloat32(0.0f));
			for (ndEdge* ptr = edge->m_next; ptr != edge; ptr = ptr->m_next) {
				ndVector p2(buffer[ptr->m_next->m_incidentVertex]);
				ndBigVector e2(p2 - p0);
				normal += e1.CrossProduct(e2);
				e1 = e2;
			}
			normal = normal.Normalize();
			if (dAbs(normal.m_x) > ndFloat32 (0.99f)) {
				ndEdge* ptr = edge;
				do {
					dAttibutFormat::dgUV uv;
					ndVector p(buffer[ptr->m_incidentVertex]);
					uv.m_u = ndFloat32((p.m_y - pMin.m_y) * scale.m_y);
					uv.m_v = ndFloat32((p.m_z - pMin.m_z) * scale.m_z);
					m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)] = uv;
					m_attrib.m_materialChannel[ndInt32(ptr->m_userData)] = capMaterial;
					ptr = ptr->m_next;
				} while (ptr != edge);
			}

			ndEdge* ptr = edge;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != edge);

		}
    }
	PackAttibuteData();
}

void ndMeshEffect::AngleBaseFlatteningMapping (ndInt32 material, dgReportProgress progressReportCallback, void* const userData)
{
	dgSetPrecisionDouble presicion;

	ndMeshEffect tmp (*this);

	ndBigVector minBox;
	ndBigVector maxBox;
	tmp.CalculateAABB(minBox, maxBox);

	ndBigVector size (maxBox - minBox);
	ndFloat32 scale = ndFloat32 (1.0 / dMax (size.m_x, size.m_y, size.m_z));

	ndMatrix matrix (dGetIdentityMatrix());
	matrix[0][0] = scale;
	matrix[1][1] = scale;
	matrix[2][2] = scale;
	tmp.ApplyTransform(matrix);

	dgAngleBasedFlatteningMapping angleBadedFlattening (&tmp, material, progressReportCallback, userData);
}

#endif


void ndMeshEffect::CalculateNormals(ndFloat64 angleInRadians)
{
	ndEdge* edgeBuffer[256];
	ndBigVector faceNormal[256];

	UnpackAttibuteData();
	m_attrib.m_normalChannel.Resize(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_normalChannel.SetCount(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_normalChannel.m_isValid = true;

	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	ndFloat32 smoothValue = ndCos(angleInRadians);

	ndTree<ndInt32, ndEdge*> normalsMap;
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) 
		{
			ndInt32 edgeIndex = 0;
			normalsMap.RemoveAll();
			ndEdge* edgePtr = edge;
			do 
			{
				ndVector normal(FaceNormal(edgePtr, &m_points.m_vertex[0].m_x, sizeof(ndBigVector)));
				dAssert(normal.m_w == ndFloat32(0.0f));
				normal = normal.Scale(ndFloat32(1.0f) / ndFloat32(sqrt(normal.DotProduct(normal).GetScalar()) + ndFloat32(1.0e-16f)));
				faceNormal[edgeIndex] = normal;
				normalsMap.Insert(edgeIndex, edgePtr);
				edgeIndex++;
				edgePtr = edgePtr->m_twin->m_next;
			} while (edgePtr != edge);

			ndEdge* startEdge = edge;
			ndVector normal0(faceNormal[normalsMap.Find(startEdge)->GetInfo()]);
			for (ndEdge* ptr = edge->m_prev->m_twin; (ptr->m_mark != mark) && (ptr != edge) && (ptr->m_incidentFace > 0); ptr = ptr->m_prev->m_twin) 
			{
				const ndVector& normal1(faceNormal[normalsMap.Find(ptr)->GetInfo()]);
				dAssert(normal0.m_w == ndFloat32(0.0f));
				ndFloat32 dot = normal0.DotProduct(normal1).GetScalar();
				if (dot < smoothValue) 
				{
					break;
				}
				startEdge = ptr;
				normal0 = normal1;
			}

			ndInt32 attribCount = 1;
			edgeBuffer[0] = startEdge;
			normal0 = faceNormal[normalsMap.Find(startEdge)->GetInfo()];
			ndVector normal(normal0);
			for (ndEdge* ptr = startEdge->m_twin->m_next; (ptr->m_mark != mark) && (ptr != startEdge) && (ptr->m_incidentFace > 0); ptr = ptr->m_twin->m_next) 
			{
				const ndVector& normal1(faceNormal[normalsMap.Find(ptr)->GetInfo()]);
				dAssert(normal0.m_w == ndFloat32(0.0f));
				ndFloat32 dot = normal0.DotProduct(normal1).GetScalar();
				if (dot < smoothValue) 
				{
					break;
				}
				edgeBuffer[attribCount] = ptr;
				attribCount++;
				normal += normal1;
				normal0 = normal1;
			}

			dAssert(normal.m_w == ndFloat32(0.0f));
			normal = normal.Scale(ndFloat32(1.0f) / ndFloat32(sqrt(normal.DotProduct(normal).GetScalar()) + ndFloat32(1.0e-16f)));
			ndTriplex n;
			n.m_x = normal.m_x;
			n.m_y = normal.m_y;
			n.m_z = normal.m_z;
			for (ndInt32 i = 0; i < attribCount; i++) 
			{
				edgeBuffer[i]->m_mark = mark;
				ndInt32 index = ndInt32(edgeBuffer[i]->m_userData);
				m_attrib.m_normalChannel[index] = n;
			}
		}
	}
	PackAttibuteData();
}

ndBigVector ndMeshEffect::GetOrigin()const
{
	ndBigVector origin(ndFloat64(0.0f), ndFloat64(0.0f), ndFloat64(0.0f), ndFloat64(0.0f));
	for (ndInt32 i = 0; i < m_points.m_vertex.GetCount(); i++) 
	{
		origin += m_points.m_vertex[i];
	}
	return origin.Scale(ndFloat64(1.0f) / m_points.m_vertex.GetCount());
}

void ndMeshEffect::BoxMapping(ndInt32 front, ndInt32 side, ndInt32 top, const ndMatrix& textureMatrix)
{
	ndBigVector origin(GetOrigin());
	ndStack<ndBigVector> buffer(m_points.m_vertex.GetCount());
	ndBigVector pMin(ndFloat64(1.0e10f), ndFloat64(1.0e10f), ndFloat64(1.0e10f), ndFloat64(0.0f));
	ndBigVector pMax(ndFloat64(-1.0e10f), ndFloat64(-1.0e10f), ndFloat64(-1.0e10f), ndFloat64(0.0f));

	for (ndInt32 i = 0; i < m_points.m_vertex.GetCount(); i++) 
	{
		buffer[i] = textureMatrix.RotateVector(m_points.m_vertex[i] - origin);
		const ndBigVector& tmp = buffer[i];
		pMin.m_x = dMin(pMin.m_x, tmp.m_x);
		pMax.m_x = dMax(pMax.m_x, tmp.m_x);
		pMin.m_y = dMin(pMin.m_y, tmp.m_y);
		pMax.m_y = dMax(pMax.m_y, tmp.m_y);
		pMin.m_z = dMin(pMin.m_z, tmp.m_z);
		pMax.m_z = dMax(pMax.m_z, tmp.m_z);
	}
	ndInt32 materialArray[3];

	ndBigVector dist(pMax);
	dist[0] = dMax(ndFloat64(1.0e-3f), dist[0]);
	dist[1] = dMax(ndFloat64(1.0e-3f), dist[1]);
	dist[2] = dMax(ndFloat64(1.0e-3f), dist[2]);
	ndBigVector scale(ndFloat64(0.5f) / dist[0], ndFloat64(0.5f) / dist[1], ndFloat64(0.5f) / dist[2], ndFloat64(0.0f));

	UnpackAttibuteData();
	m_attrib.m_uv0Channel.SetCount(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_materialChannel.SetCount(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_uv0Channel.m_isValid = true;
	m_attrib.m_materialChannel.m_isValid = true;

	materialArray[0] = front;
	materialArray[1] = side;
	materialArray[2] = top;

	ndInt32 mark = IncLRU();
	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) 
		{
			const ndBigVector& p0 = buffer[edge->m_incidentVertex];
			const ndBigVector& p1 = buffer[edge->m_next->m_incidentVertex];
			const ndBigVector& p2 = buffer[edge->m_prev->m_incidentVertex];

			edge->m_mark = mark;
			edge->m_next->m_mark = mark;
			edge->m_prev->m_mark = mark;

			ndBigVector e0(p1 - p0);
			ndBigVector e1(p2 - p0);
			ndBigVector n(e0.CrossProduct(e1));

			ndInt32 index = 0;
			ndFloat64 maxProjection = ndFloat32(0.0f);

			for (ndInt32 i = 0; i < 3; i++) 
			{
				ndFloat64 proj = fabs(n[i]);
				if (proj > maxProjection) 
				{
					index = i;
					maxProjection = proj;
				}
			}

			ndInt32 u = (index + 1) % 3;
			ndInt32 v = (u + 1) % 3;
			if (index == 1) 
			{
				dSwap(u, v);
			}
			ndEdge* ptr = edge;
			do 
			{
				dAttibutFormat::dgUV uv;
				ndBigVector p(scale * buffer[ptr->m_incidentVertex] - ndFloat32(0.5f));
				uv.m_u = ndFloat32(p[u]);
				uv.m_v = ndFloat32(p[v]);
				m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)] = uv;
				m_attrib.m_materialChannel[ndInt32(ptr->m_userData)] = materialArray[index];
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
	PackAttibuteData();
}

void ndMeshEffect::UniformBoxMapping(ndInt32 material, const ndMatrix& textureMatrix)
{
	UnpackAttibuteData();

	m_attrib.m_uv0Channel.Resize(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_uv0Channel.SetCount(m_attrib.m_pointChannel.GetCount());

	m_attrib.m_materialChannel.Resize(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_materialChannel.SetCount(m_attrib.m_pointChannel.GetCount());

	m_attrib.m_uv0Channel.m_isValid = true;
	m_attrib.m_materialChannel.m_isValid = true;

	ndInt32 mark = IncLRU();
	for (ndInt32 i = 0; i < 3; i++) 
	{
		ndMatrix rotationMatrix(dGetIdentityMatrix());
		if (i == 1) 
		{
			rotationMatrix = dYawMatrix(ndFloat32(90.0f * ndDegreeToRad));
		}
		else if (i == 2) 
		{
			rotationMatrix = dPitchMatrix(ndFloat32(90.0f * ndDegreeToRad));
		}

		ndPolyhedra::Iterator iter(*this);

		for (iter.Begin(); iter; iter++) 
		{
			ndEdge* const edge = &(*iter);
			if ((edge->m_mark < mark) && (edge->m_incidentFace > 0)) 
			{
				ndBigVector n(FaceNormal(edge, &m_points.m_vertex[0].m_x, sizeof(ndBigVector)));
				ndVector normal(rotationMatrix.RotateVector(ndVector(n.Normalize())));
				normal.m_x = dAbs(normal.m_x);
				normal.m_y = dAbs(normal.m_y);
				normal.m_z = dAbs(normal.m_z);
				if ((normal.m_z >= (normal.m_x - ndFloat32(1.0e-4f))) && (normal.m_z >= (normal.m_y - ndFloat32(1.0e-4f)))) 
				{
					ndEdge* ptr = edge;
					do 
					{
						ptr->m_mark = mark;
						dAttibutFormat::dgUV uv;
						ndVector point(rotationMatrix.RotateVector(m_points.m_vertex[ptr->m_incidentVertex]));
						ndVector p(textureMatrix.TransformVector(point));
						uv.m_u = p.m_x;
						uv.m_v = p.m_y;
						m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)] = uv;
						m_attrib.m_materialChannel[ndInt32(ptr->m_userData)] = material;

						ptr = ptr->m_next;
					} while (ptr != edge);
				}
			}
		}
	}

	PackAttibuteData();
}

void ndMeshEffect::SphericalMapping(ndInt32 material, const ndMatrix& textureMatrix)
{
	ndBigVector origin(GetOrigin());
	ndStack<ndBigVector>sphere(m_points.m_vertex.GetCount());
	for (ndInt32 i = 0; i < m_points.m_vertex.GetCount(); i++)
	{
		ndBigVector geoPoint(m_points.m_vertex[i] - origin);
		dAssert(geoPoint.m_w == ndFloat32(0.0f));
		ndVector point(textureMatrix.RotateVector(geoPoint.Normalize()));
		point.m_x = dClamp(point.m_x, ndFloat32(-1.0f + 1.0e-6f), ndFloat32(1.0f - 1.0e-6f));

		ndFloat64 v = dClamp(ndAsin(point.m_x) / ndPi + ndFloat64(0.5f), ndFloat64(0.0f), ndFloat64(1.0f));
		ndFloat64 u = dClamp(ndAtan2(point.m_y, point.m_z) / (ndFloat32(2.0f) * ndPi) + ndFloat64(0.5f), ndFloat64(0.0f), ndFloat64(1.0f));
		sphere[i].m_x = u;
		sphere[i].m_y = v;
	}

	UnpackAttibuteData();

	m_attrib.m_uv0Channel.Resize(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_uv0Channel.SetCount(m_attrib.m_pointChannel.GetCount());

	m_attrib.m_materialChannel.Resize(m_attrib.m_pointChannel.GetCount());
	m_attrib.m_materialChannel.SetCount(m_attrib.m_pointChannel.GetCount());

	m_attrib.m_uv0Channel.m_isValid = true;
	m_attrib.m_materialChannel.m_isValid = true;

	ndPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		dAttibutFormat::dgUV uv;
		uv.m_u = ndFloat32(sphere[edge->m_incidentVertex].m_x);
		uv.m_v = ndFloat32(sphere[edge->m_incidentVertex].m_y);
		m_attrib.m_uv0Channel[ndInt32(edge->m_userData)] = uv;
		m_attrib.m_materialChannel[ndInt32(edge->m_userData)] = material;
	}

	ndInt32 mark = IncLRU();
	for (iter.Begin(); iter; iter++) 
	{
		ndEdge* const edge = &(*iter);
		if ((edge->m_incidentFace > 0) && (edge->m_mark != mark)) 
		{
			dAttibutFormat::dgUV uvRef(m_attrib.m_uv0Channel[ndInt32(edge->m_userData)]);
			ndFloat32 angle0 = uvRef.m_u * ndPi * ndFloat32(2.0f);
			ndFloat32 UVrefSin = ndSin(angle0);
			ndFloat32 UVrefCos = ndCos(angle0);
			ndEdge* ptr = edge;
			do 
			{
				ptr->m_mark = mark;
				dAttibutFormat::dgUV uv(m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)]);
				ndFloat32 angle1 = uv.m_u * ndPi * ndFloat32(2.0f);
				ndFloat32 sinAngle = ndSin(angle1) * UVrefCos - ndCos(angle1) * UVrefSin;
				ndFloat32 cosAngle = ndCos(angle1) * UVrefCos + ndSin(angle1) * UVrefSin;
				ndFloat32 deltaAngle = ndAtan2(sinAngle, cosAngle);
				uv.m_u = uvRef.m_u + deltaAngle / (ndFloat32 (2.0f) * ndPi);
				m_attrib.m_uv0Channel[ndInt32(ptr->m_userData)] = uv;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	PackAttibuteData();
}
