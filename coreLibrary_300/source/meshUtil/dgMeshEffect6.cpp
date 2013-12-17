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

#define dgABF_MAX_ITERATIONS	100
#define dgABF_TOL				dgFloat64 (1.0e-3f)
#define dgABF_TOL2				dgABF_TOL * dgABF_TOL
#define dgABF_PI				dgFloat64 (3.1415926535)


class TestSolver_xxxxxxx: public SymmetricBiconjugateGradientSolve
{
	public:
	dgFloat64 a[3][3];

	TestSolver_xxxxxxx()
		:SymmetricBiconjugateGradientSolve()
	{
		dgFloat64 b[] = {1, 2, 3};
		dgFloat64 x[] = {0, 0, 0};
		dgFloat64 c[3];

		dgMatrix aaa (dgPitchMatrix(30.0f * 3.141592f / 180.0f) * dgYawMatrix(30.0f * 3.141592f / 180.0f));
		dgMatrix bbb = aaa.Transpose();
		bbb[1] = bbb[1].Scale3 (2.0f);
		bbb[2] = bbb[2].Scale3 (2.0f);
		aaa = aaa * bbb;

		for (int i = 0; i < 3; i ++) {
			for (int j = 0; j < 3; j ++) {
				a[i][j] = aaa[i][j];
			}
		}

		Solve (3, 3, dgFloat64  (1.0e-10f), x, b);

		MatrixTimeVector (c, x);
		MatrixTimeVector (c, x);
	}

	void MatrixTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		out[0] = a[0][0] * v[0] +  a[0][1] * v[1] +  a[0][2] * v[2];
		out[1] = a[1][0] * v[0] +  a[1][1] * v[1] +  a[1][2] * v[2];
		out[2] = a[2][0] * v[0] +  a[2][1] * v[1] +  a[2][2] * v[2];
	}

	void InversePrecoditionerTimeVector (dgFloat64* const out, const dgFloat64* const v) const
	{
		out[0] = v[0] / a[0][0];
		out[1] = v[1] / a[1][1];
		out[2] = v[2] / a[2][2];
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
		InitMutipliersCount();

		dgInt32 vertexCount = m_mesh->GetVertexCount();

		m_betaEdge = (dgEdge**) m_mesh->GetAllocator()->MallocLow(m_anglesCount * sizeof (dgEdge*));
		m_interiorIndirectMap = (dgInt32*) m_mesh->GetAllocator()->MallocLow (vertexCount * sizeof (dgInt32));

		m_beta = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_weight= (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_alphaLambda = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_uv = (dgTriplex*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgTriplex));

		InitEdgeVector();
		CalculateInitialAngles ();

		// if there are no interior vertex then the mesh can be projected to a flat plane
		if (m_interiorVertexCount) {
			LagrangeOptimization();
		}
		GenerateUVCoordinates ();
	}

	~dgAngleBasedFlatteningMapping()
	{
		if (m_interiorVertexCount) {
			m_mesh->GetAllocator()->FreeLow (m_triangleLanbdaGradients);
			m_mesh->GetAllocator()->FreeLow (m_alphaLambdaGradients);
			m_mesh->GetAllocator()->FreeLow (m_interiorVertexPlanarityLambda);
			m_mesh->GetAllocator()->FreeLow (m_interiorVertexWheelLambda);
			m_mesh->GetAllocator()->FreeLow (m_triangleLambda);
			m_mesh->GetAllocator()->FreeLow (m_cosTable);
			m_mesh->GetAllocator()->FreeLow (m_sinTable);
		}

		m_mesh->GetAllocator()->FreeLow (m_uv);
		m_mesh->GetAllocator()->FreeLow (m_alphaLambda);
		m_mesh->GetAllocator()->FreeLow (m_weight);
		m_mesh->GetAllocator()->FreeLow (m_beta);
		m_mesh->GetAllocator()->FreeLow (m_interiorIndirectMap);
		m_mesh->GetAllocator()->FreeLow (m_betaEdge);

	}

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
	}

	void InitMutipliersCount()
	{
		m_anglesCount = 0;;
		m_triangleCount = 0;
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
		m_interiorVertexCount = 0;
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
	}


	dgInt32 GetAlphaLandaIndex (const dgEdge* const edge) const
	{
		return edge->m_incidentFace - 1;
	}

	dgInt32 GetTriangleIndex (const dgInt32 alphaIndex) const
	{
		return alphaIndex / 3;
	}

	dgInt32 GetTriangleIndex (const dgEdge* const edge) const
	{
		return GetAlphaLandaIndex(edge) / 3;
	}

	dgInt32 GetInteriorVertex(const dgEdge* const edge) const
	{
		return m_interiorIndirectMap[edge->m_incidentVertex];
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
					m_interiorIndirectMap[edge->m_incidentVertex] = count;
					count ++;
				}
			}
		}
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

			m_beta[i] = acos (dgClamp(e10 % e20, dgFloat64 (0.0), dgABF_PI));
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
			m_alphaLambda[i] = m_beta[i];
			m_weight[i] = dgFloat64 (2.0f) / (m_beta[i] * m_beta[i]);
		}
	}



	dgFloat64 CalculateAlphaLambdaGradient (dgInt32 alphaIndex) const
	{
		return (m_alphaLambda[alphaIndex] - m_beta[alphaIndex]) * m_weight[alphaIndex] + m_triangleLambda[GetTriangleIndex(alphaIndex)];
	}

	dgFloat64 CalculateInteriorVertexGradient (dgInt32 alphaIndex) const
	{
		dgInt32 index = GetInteriorVertex(m_betaEdge[alphaIndex]);
		return (index >= 0) ? m_interiorVertexPlanarityLambda[index] : dgFloat32 (0.0f);
	}

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
			gradient += product;
		}

		if (GetInteriorVertex (incidentEdge->m_prev) != -1) {
			dgEdge* const edge = m_betaEdge[GetAlphaLandaIndex(incidentEdge->m_prev)];
			dgFloat64 product = m_cosTable[GetAlphaLandaIndex(edge->m_next)];
			dgEdge* ptr = edge->m_twin->m_next;
			do {
				product *= m_sinTable[GetAlphaLandaIndex(ptr->m_next)];
				ptr = ptr->m_twin->m_next;
			} while (ptr != edge);
			gradient -= product;
		}
		return gradient;
	}


	dgFloat64 CalculateGradientVector ()
	{
		dgFloat64 error2 = dgFloat64 (0.0f);
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			m_sinTable[i] = sin (m_alphaLambda[i]);
			m_cosTable[i] = cos (m_alphaLambda[i]);
		}

		// calculate the new face angular gradients
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgFloat64 gradient = CalculateAlphaLambdaGradient (i) + CalculateInteriorVertexGradient(i) + CalculatePlanarityGradient (i);
			m_alphaLambdaGradients[i] = -gradient;
			error2 += gradient * gradient;
		}

		for (dgInt32 i = 0; i < m_triangleCount; i ++) {
			dgFloat64 gradient = m_alphaLambdaGradients[i * 3 + 0] + m_alphaLambdaGradients[i * 3 + 1] + m_alphaLambdaGradients[i * 3 + 2] - dgABF_PI;
			m_triangleLanbdaGradients[i] = gradient;
			error2 += gradient * gradient;
		}

		dgInt32 mark = m_mesh->IncLRU();
		for (dgInt32 i = 0; i < m_anglesCount; i ++) {
			dgEdge* const edge = m_betaEdge[i];
			if ((edge->m_mark != mark) && GetInteriorVertex(edge) != -1) {
				dgAssert (0);
				dgEdge* ptr = edge; 
				do {
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
			}
		}

		return error2;
	}


	void LagrangeOptimization()
	{
		m_sinTable = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_cosTable = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_triangleLambda = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_triangleCount * sizeof (dgFloat64));
		m_interiorVertexWheelLambda = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_interiorVertexCount * sizeof (dgFloat64));
		m_interiorVertexPlanarityLambda = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_interiorVertexCount * sizeof (dgFloat64));

		m_alphaLambdaGradients = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_anglesCount * sizeof (dgFloat64));
		m_triangleLanbdaGradients = (dgFloat64*) m_mesh->GetAllocator()->MallocLow (m_triangleCount * sizeof (dgFloat64));

		memset (m_triangleLambda, 0, m_triangleCount * sizeof (dgFloat64));	
		memset (m_interiorVertexWheelLambda, 0, m_interiorVertexCount * sizeof (dgFloat64));	
		memset (m_interiorVertexPlanarityLambda, 0, m_interiorVertexCount * sizeof (dgFloat64));	

		dgFloat64 error2 = CalculateGradientVector ();
		for (dgInt32 i = 0; (i < dgABF_MAX_ITERATIONS) && (error2 > dgABF_TOL2); i++) {
			//	dgAssert (0);
			//Solve();
		}
	}


	dgMeshEffect* m_mesh;
	
	dgEdge** m_betaEdge;
	dgInt32* m_interiorIndirectMap;

	dgFloat64* m_beta;
	dgFloat64* m_weight;
	

	dgFloat64* m_sinTable;
	dgFloat64* m_cosTable;
	dgFloat64* m_alphaLambda;
	dgFloat64* m_triangleLambda;
	dgFloat64* m_interiorVertexWheelLambda;
	dgFloat64* m_interiorVertexPlanarityLambda;

	dgFloat64* m_alphaLambdaGradients;
	dgFloat64* m_triangleLanbdaGradients;

	dgTriplex* m_uv;

	dgInt32 m_material;
	dgInt32 m_anglesCount;
	dgInt32 m_triangleCount;
	dgInt32 m_interiorVertexCount;
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
