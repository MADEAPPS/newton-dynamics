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

#include "dgPhysicsStdafx.h"
#include "dgWorld.h"
#include "dgMeshEffect.h"


#define dgABF_PI dgFloat64 (3.1415926535)



class dgAngleBasedFlatteningMapping
{
	enum dgABF_Type
	{
		m_interiorVertex = 0,
		m_exteriorVertex,
	};

	class dgVertex
	{
		public:
		dgEdge* m_incidentEdge;
		dgABF_Type m_type;
	};

	class dgFace
	{
		public:
		void Init (const dgMeshEffect* const mesh, dgEdge* const edge, dgInt32 enumerationID)
		{
			m_incidenEdge = edge;
			m_incidenEdge->m_incidentFace = enumerationID + 1;

			const dgBigVector& p0 = mesh->GetVertex(edge->m_prev->m_incidentVertex);
			const dgBigVector& p1 = mesh->GetVertex(edge->m_incidentVertex);
			const dgBigVector& p2 = mesh->GetVertex(edge->m_next->m_incidentVertex);

			dgBigVector e10 (p1 - p0);
			dgBigVector e20 (p2 - p0);

			e10 = e10.Scale3 (dgFloat64 (1.0) / sqrt (e10 % e10));
			e20 = e20.Scale3 (dgFloat64 (1.0) / sqrt (e20 % e20));
			m_beta = acos (dgClamp(e10 % e20, dgFloat64 (0.0), dgABF_PI));
			m_alpha = m_beta;
		}

		dgEdge* m_incidenEdge;
		dgFloat64 m_alpha;
		dgFloat64 m_beta;
		dgFloat64 m_u;
		dgFloat64 m_v;
	};

	public: 
	dgAngleBasedFlatteningMapping (dgMeshEffect* const mesh, dgInt32 material)
		:m_faceArray(1024, mesh->GetAllocator())
		,m_vertexArray(1024, mesh->GetAllocator())
		,m_mesh(mesh)
		,m_material(material)
	{
		m_mesh->Triangulate();
		m_vertexCount = m_mesh->GetVertexCount();
		m_edgeCount = m_mesh->GetFaceCount() * 3;
		m_interiorVertexCount = 0;

		InitializeVertexTypeVector ();
		InitializeFaceVector ();
		if (m_interiorVertexCount) {
			OptimizePlanrAngles();
		}
		CalculateUV ();
	}

	~dgAngleBasedFlatteningMapping()
	{
	}

	void InitializeVertexTypeVector ()
	{
		dgInt32 count = 0;

		dgInt32 mark = m_mesh->IncLRU();
		dgMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &iter.GetNode()->GetInfo();

			dgABF_Type vertexType = m_interiorVertex;
			if (edge->m_mark != mark) {
				dgEdge* ptr = edge; 
				do {
					if (ptr->m_incidentFace < 0) {
						vertexType = m_exteriorVertex;
					}

					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != edge);
				m_interiorVertexCount += (vertexType == m_interiorVertex) ? 1 : 0;

				m_vertexArray[count].m_type = vertexType;
				m_vertexArray[count].m_incidentEdge = edge;
				count ++;
				dgAssert (count <= m_vertexCount);
			}
		}
		dgAssert (count == m_vertexCount);
	}

	void InitializeFaceVector  ()
	{
		dgInt32 count = 0;
		dgInt32 mark = m_mesh->IncLRU();

		// calculate initial angles;
		dgMeshEffect::Iterator iter (*m_mesh);
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const face = &iter.GetNode()->GetInfo();
			if ((face->m_incidentFace > 0) && (face->m_mark != mark)) {
				dgAssert (face->m_next->m_next->m_next == face);

				dgEdge* ptr = face; 
				do {
					// save the vertex index plus one in the face 
					m_faceArray[count].Init(m_mesh, ptr, count);
					count ++;
					ptr->m_mark = mark;
					ptr = ptr->m_next;
				} while (ptr != face);
				dgAssert (count <= m_edgeCount);
			}
		}
		dgAssert (count == m_edgeCount);

		// calculate weight factor
		for (dgInt32 i = 0; i < m_vertexCount; i ++) {
			if (m_vertexArray[i].m_type == m_interiorVertex) {
				dgAssert (0);
				dgVertex& vertex = m_vertexArray[i];
				dgFloat64 sum = dgFloat64 (0.0f);

				dgEdge* ptr = vertex.m_incidentEdge; 
				do {
					dgInt32 faceIndex = ptr->m_incidentFace - 1;
					sum += m_faceArray[faceIndex].m_beta;
					ptr->m_mark = mark;
					ptr = ptr->m_twin->m_next;
				} while (ptr != vertex.m_incidentEdge);
			}
		}
	}

	void OptimizePlanrAngles()
	{
		dgAssert (0);
	}

	void CalculateUV ()
	{
		
		dgInt32 mark = m_mesh->IncLRU();
		for (dgInt32 i = 0; i < m_edgeCount; i ++) {
			dgFace& face = m_faceArray[i];
			if (face.m_incidenEdge->m_mark != mark) {
				dgList<dgEdge*> stack(m_mesh->GetAllocator());
				stack.Append(face.m_incidenEdge);

 				dgEdge* const next = face.m_incidenEdge->m_next;
				dgEdge* const prev = face.m_incidenEdge->m_prev;
				dgFace& nextFaceEdge = m_faceArray[next->m_incidentFace - 1];
				dgFace& prevFaceEdge = m_faceArray[prev->m_incidentFace - 1];
				dgAssert (nextFaceEdge.m_incidenEdge == next);
				dgAssert (prevFaceEdge.m_incidenEdge == prev);

				const dgBigVector& p0 = m_mesh->GetVertex(face.m_incidenEdge->m_incidentVertex);
				const dgBigVector& p1 = m_mesh->GetVertex(next->m_incidentVertex);
				dgBigVector p10 (p1 - p0);

				face.m_u = dgFloat64 (0.0f);
				face.m_v = dgFloat64 (0.0f);
				nextFaceEdge.m_u = sqrt (p10 % p10);
				nextFaceEdge.m_v = dgFloat64 (0.0f);

				dgFloat64 e2length = nextFaceEdge.m_u * sin (prevFaceEdge.m_alpha) / sin (face.m_alpha);
				prevFaceEdge.m_u = face.m_u + e2length * cos (nextFaceEdge.m_alpha);
				prevFaceEdge.m_v = face.m_v + e2length * sin (nextFaceEdge.m_alpha);


				while (stack.GetCount()) {
					dgEdge* const edge = stack.GetLast()->GetInfo();
					stack.Remove (stack.GetLast());
					if (edge->m_mark != mark) {
						dgEdge* const next = edge->m_next;
						dgEdge* const prev = edge->m_prev;

						edge->m_mark = mark;
						next->m_mark = mark;
						prev->m_mark = mark;

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
	
	dgArray<dgFace> m_faceArray; 
	dgArray<dgVertex> m_vertexArray; 

	dgMeshEffect* m_mesh;
	dgInt32 m_material;
	dgInt32 m_edgeCount;
	dgInt32 m_vertexCount;
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
	dgAngleBasedFlatteningMapping angleBadedFlattening (this, material);
}
