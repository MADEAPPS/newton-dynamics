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
#include "dgBody.h"
#include "dgWorld.h"
#include "dgMeshEffect.h"
#include "dgCollisionConvexHull.h"



bool dgMeshEffect::PlaneClip(const dgMeshEffect& convexMesh, const dgEdge* const convexFace)
{
	dgAssert (convexFace->m_incidentFace > 0);

	dgBigVector normal (convexMesh.FaceNormal(convexFace, &convexMesh.m_points[0].m_x, sizeof(dgBigVector)));
	dgFloat64 mag2 = normal % normal;
	if (mag2 < dgFloat64 (1.0e-30)) {
		dgAssert (0);
		return true;
	}  

	normal = normal.Scale3(dgFloat64 (1.0f) / sqrt (mag2));
	dgBigVector origin (convexMesh.m_points[convexFace->m_incidentVertex]);
	dgBigPlane plane (normal, - (origin % normal));

	dgAssert (!HasOpenEdges());

	dgInt32 pointCount = GetVertexCount();
	dgStack <dgFloat64> testPool (2 * pointCount + 1024);
	dgFloat64* const test = &testPool[0];
	for (dgInt32 i = 0; i < pointCount; i ++) {
		test[i] = plane.Evalue (m_points[i]);
		if (fabs (test[i]) < dgFloat32 (1.0e-5f)) {
			test[i] = dgFloat32 (0.0f);
		}
	}

	dgInt32 positive = 0;
	dgInt32 negative = 0;
	dgPolyhedra::Iterator iter (*this); 
	for (iter.Begin(); iter && !(positive && negative); iter ++){
		dgEdge* const edge = &(*iter);
		positive += test[edge->m_incidentVertex] > dgFloat32 (0.0f);
		negative += test[edge->m_incidentVertex] < dgFloat32 (0.0f);
	}
	if (positive  && !negative) {
		return false;
	}

	if (positive && negative) {

		const dgEdge* e0 = convexFace;
		const dgEdge* e1 = e0->m_next;
		const dgEdge* e2 = e1->m_next;

		dgMatrix matrix;
		dgBigVector p1 (convexMesh.m_points[e1->m_incidentVertex]);

		dgBigVector xDir (p1 - origin);
		dgAssert ((xDir % xDir) > dgFloat32 (0.0f));
		matrix[2] = dgVector (normal);
		matrix[0] = dgVector(xDir.Scale3(dgFloat64 (1.0f) / sqrt (xDir% xDir)));
		matrix[1] = matrix[2] * matrix[0];
		matrix[3] = dgVector (origin);
		matrix[3][3] = dgFloat32 (1.0f);

		dgVector q0 (matrix.UntransformVector(dgVector(convexMesh.m_points[e0->m_incidentVertex])));
		dgVector q1 (matrix.UntransformVector(dgVector(convexMesh.m_points[e1->m_incidentVertex])));
		dgVector q2 (matrix.UntransformVector(dgVector(convexMesh.m_points[e2->m_incidentVertex])));

		dgVector p10 (q1 - q0);
		dgVector p20 (q2 - q0);
		dgVector faceNormal (matrix.UnrotateVector (dgVector(normal)));
		dgFloat32 areaInv = (p10 * p20) % faceNormal;
		if (e2->m_next != e0) {
			const dgEdge* edge = e2;
			dgVector r1 (q2);
			dgVector p10 (p20);
			do {
				dgVector r2 (matrix.UntransformVector(dgVector(convexMesh.m_points[edge->m_next->m_incidentVertex])));
				dgVector p20 (r2 - q0);
				dgFloat32 areaInv1 = (p10 * p20) % faceNormal;
				if (areaInv1 > areaInv) {
					e1 = edge;
					e2 = edge->m_next;
					q1 = r1;
					q2 = r2;
					areaInv = areaInv1;
				}
				r1 = r2;
				p10 = p20;
				edge = edge->m_next;
			} while (edge->m_next != e0);
		}

		dgAssert (areaInv > dgFloat32 (0.0f));
		areaInv = dgFloat32 (1.0f) / areaInv;

		dgVector uv0_0 (dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_u0), dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_v0), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv0_1 (dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_u0), dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_v0), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv0_2 (dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_u0), dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_v0), dgFloat32 (0.0f), dgFloat32 (0.0f));

		dgVector uv1_0 (dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_u1), dgFloat32 (convexMesh.m_attrib[e0->m_userData].m_v1), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv1_1 (dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_u1), dgFloat32 (convexMesh.m_attrib[e1->m_userData].m_v1), dgFloat32 (0.0f), dgFloat32 (0.0f));
		dgVector uv1_2 (dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_u1), dgFloat32 (convexMesh.m_attrib[e2->m_userData].m_v1), dgFloat32 (0.0f), dgFloat32 (0.0f));

		for (iter.Begin(); iter; iter ++){
			dgEdge* const edge = &(*iter);

			dgFloat64 side0 = test[edge->m_prev->m_incidentVertex];
			dgFloat64 side1 = test[edge->m_incidentVertex];

			if ((side0 < dgFloat32 (0.0f)) && (side1 > dgFloat64 (0.0f))) {
				dgBigVector dp (m_points[edge->m_incidentVertex] - m_points[edge->m_prev->m_incidentVertex]);
				dgFloat64 param = - side0 / (plane % dp);

				dgEdge* const splitEdge = InsertEdgeVertex (edge->m_prev, param);
				test[splitEdge->m_next->m_incidentVertex] = dgFloat64 (0.0f);
			} 
		}

		dgInt32 colorMark = IncLRU();
		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			dgFloat64 side0 = test[edge->m_incidentVertex];
			dgFloat64 side1 = test[edge->m_next->m_incidentVertex];

			if ((side0 > dgFloat32 (0.0f)) || (side1 > dgFloat64 (0.0f))) {
				edge->m_mark = colorMark;
			}
		}

		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			dgFloat64 side0 = test[edge->m_incidentVertex];
			dgFloat64 side1 = test[edge->m_next->m_incidentVertex];
			if ((side0 == dgFloat32 (0.0f)) && (side1 == dgFloat64 (0.0f))) {
				dgEdge* ptr = edge->m_next;
				do {
					if (ptr->m_mark == colorMark) {
						edge->m_mark = colorMark;
						break;
					}
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}


		for (iter.Begin(); iter; iter ++) {
			dgEdge* const edge = &(*iter);
			if ((edge->m_mark == colorMark) && (edge->m_next->m_mark < colorMark)) {
				dgEdge* const startEdge = edge->m_next;
				dgEdge* end = startEdge;
				do {
					if (end->m_mark == colorMark) {
						break;
					}

					end = end->m_next;
				} while (end != startEdge);
				dgAssert (end != startEdge);
				dgEdge* const devideEdge = ConnectVertex (startEdge, end);
				dgAssert (devideEdge);
				dgAssert (devideEdge->m_next->m_mark != colorMark);
				dgAssert (devideEdge->m_prev->m_mark != colorMark);
				dgAssert (devideEdge->m_twin->m_next->m_mark == colorMark);
				dgAssert (devideEdge->m_twin->m_prev->m_mark == colorMark);
				devideEdge->m_mark = colorMark - 1;
				devideEdge->m_twin->m_mark = colorMark;
			}
		}

		dgInt32 mark = IncLRU();
		dgList<dgEdge*> faceList (GetAllocator());
		for (iter.Begin(); iter; iter ++){
			dgEdge* const face = &(*iter);
			if ((face->m_mark >= colorMark) && (face->m_mark != mark)) {
				faceList.Append(face);
				dgEdge* edge = face;
				do {
					edge->m_mark = mark;
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		for (dgList<dgEdge*>::dgListNode* node = faceList.GetFirst(); node; node = node->GetNext()) {
			dgEdge* const face = node->GetInfo();
			DeleteFace(face);
		}

		mark = IncLRU();
		faceList.RemoveAll();
		for (iter.Begin(); iter; iter ++){
			dgEdge* const face = &(*iter);
			if ((face->m_mark != mark) && (face->m_incidentFace < 0)) {
				faceList.Append(face);
				dgEdge* edge = face;
				do {
					edge->m_mark = mark;
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		const dgFloat64 capAttribute = convexMesh.m_attrib[convexFace->m_userData].m_material;
		for (dgList<dgEdge*>::dgListNode* node = faceList.GetFirst(); node; node = node->GetNext()) {
			dgEdge* const face = node->GetInfo();

			dgEdge* edge = face;
			do {
				dgVertexAtribute attibute;
				attibute.m_vertex = m_points[edge->m_incidentVertex];
				attibute.m_normal_x = normal.m_x;
				attibute.m_normal_y = normal.m_y;
				attibute.m_normal_z = normal.m_z;
				attibute.m_material = capAttribute;

				dgVector p (matrix.UntransformVector (attibute.m_vertex));

				dgVector p_p0 (p - q0);
				dgVector p_p1 (p - q1);
				dgVector p_p2 (p - q2);
				//dgFloat32 alpha1 = p10 % p_p0;
				//dgFloat32 alpha2 = p20 % p_p0;
				//dgFloat32 alpha3 = p10 % p_p1;
				//dgFloat32 alpha4 = p20 % p_p1;
				//dgFloat32 alpha5 = p10 % p_p2;
				//dgFloat32 alpha6 = p20 % p_p2;
				//dgFloat32 vc = alpha1 * alpha4 - alpha3 * alpha2;
				//dgFloat32 vb = alpha5 * alpha2 - alpha1 * alpha6;
				//dgFloat32 va = alpha3 * alpha6 - alpha5 * alpha4;
				//dgFloat32 den = va + vb + vc;
				//dgAssert (den > 0.0f);
				//den = dgFloat32 (1.0f) / (va + vb + vc);
				//dgFloat32 alpha0 = dgFloat32 (va * den);
				//alpha1 = dgFloat32 (vb * den);
				//alpha2 = dgFloat32 (vc * den);

				dgFloat32 alpha0 = ((p_p1 * p_p2) % faceNormal) * areaInv;
				dgFloat32 alpha1 = ((p_p2 * p_p0) % faceNormal) * areaInv;
				dgFloat32 alpha2 = ((p_p0 * p_p1) % faceNormal) * areaInv;

				attibute.m_u0 = uv0_0.m_x * alpha0 + uv0_1.m_x * alpha1 + uv0_2.m_x * alpha2; 
				attibute.m_v0 = uv0_0.m_y * alpha0 + uv0_1.m_y * alpha1 + uv0_2.m_y * alpha2; 
				attibute.m_u1 = uv1_0.m_x * alpha0 + uv1_1.m_x * alpha1 + uv1_2.m_x * alpha2; 
				attibute.m_v1 = uv1_0.m_y * alpha0 + uv1_1.m_y * alpha1 + uv1_2.m_y * alpha2; 

				AddAtribute (attibute);
				edge->m_incidentFace = 1;
				edge->m_userData = m_atribCount - 1;

				//faceIndices[indexCount] = edge->m_incidentVertex;
				//indexCount ++;
				//dgAssert (indexCount < sizeof (faceIndices) / sizeof (faceIndices[0]));

				edge = edge->m_next;
			} while (edge != face);

			//facePolygedra.AddFace(indexCount, faceIndices);
			//facePolygedra.EndFace();

			//dgPolyhedra leftOversOut(GetAllocator());
			//facePolygedra.ConvexPartition (&m_points[0].m_x, sizeof (dgBigVector), &leftOversOut);
			//dgAssert (leftOversOut.GetCount() == 0);
		}
	}

	return true;
}


dgMeshEffect* dgMeshEffect::ConvexMeshIntersection (const dgMeshEffect* const convexMeshSrc) const
{
	dgMeshEffect convexMesh (*convexMeshSrc);
	convexMesh.ConvertToPolygons();
	//return new (GetAllocator()) dgMeshEffect (*convexMesh);

	dgMeshEffect* const convexIntersection = new (GetAllocator()) dgMeshEffect (*this);
	//convexIntersection->ConvertToPolygons();
	//convexIntersection->Triangulate();
	convexIntersection->RemoveUnusedVertices(NULL);

	dgInt32 mark = convexMesh.IncLRU();
	dgPolyhedra::Iterator iter (convexMesh);

	for (iter.Begin(); iter; iter ++){
		dgEdge* const convexFace = &(*iter);
		if ((convexFace->m_incidentFace > 0) && (convexFace->m_mark != mark)) {
			dgEdge* ptr = convexFace;
			do {
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != convexFace);
			if (!convexIntersection->PlaneClip(convexMesh, convexFace)) {
				delete convexIntersection;
				return NULL;
			}
		}
	}

	convexIntersection->RemoveUnusedVertices(NULL);
	if (!convexIntersection->GetVertexCount()) {
		delete convexIntersection;
		return NULL;
	}

	convexIntersection->RemoveUnusedVertices(NULL);

	return convexIntersection;
}


void dgMeshEffect::ClipMesh (const dgMatrix& matrix, const dgMeshEffect* const clipMesh, dgMeshEffect** const back, dgMeshEffect** const front) const
{
	dgAssert (0);
/*
	dgMeshEffect clipper (*clipMesh);
	clipper.TransformMesh (matrix);

	dgMeshEffect* backMeshSource = NULL;
	dgMeshEffect* frontMeshSource = NULL;
	dgMeshEffect* backMeshClipper = NULL;
	dgMeshEffect* frontMeshClipper = NULL;

	ClipMesh (&clipper, &backMeshSource, &frontMeshSource);
	if (backMeshSource && frontMeshSource) {
		clipper.ClipMesh (this, &backMeshClipper, &frontMeshClipper);
		if (backMeshSource && frontMeshSource) {

			dgMeshEffect* backMesh;
			dgMeshEffect* frontMesh;

			backMesh = new (GetAllocator()) dgMeshEffect (GetAllocator(), true);
			frontMesh = new (GetAllocator()) dgMeshEffect (GetAllocator(), true);

			backMesh->BeginPolygon();
			frontMesh->BeginPolygon();

			backMesh->MergeFaces(backMeshSource);
			backMesh->MergeFaces(backMeshClipper);

			frontMesh->MergeFaces(frontMeshSource);
			frontMesh->ReverseMergeFaces(backMeshClipper);

			backMesh->EndPolygon(dgFloat64 (1.0e-5f));
			frontMesh->EndPolygon(dgFloat64 (1.0e-5f));

			*back = backMesh;
			*front = frontMesh;
		}
	}

	if (backMeshClipper) {
		delete backMeshClipper;
	}

	if (frontMeshClipper) {
		delete frontMeshClipper;
	}

	if (backMeshSource) {
		delete backMeshSource;
	}

	if (frontMeshSource) {
		delete frontMeshSource;
	}
*/
}




dgMeshEffect* dgMeshEffect::Union (const dgMatrix& matrix, const dgMeshEffect* const clipperMesh) const
{
	dgAssert (0);
	return NULL;
/*
	dgMeshEffect copy (*this);
	dgMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);

	dgMeshEffect* const mesh = new (GetAllocator()) dgMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddExteriorFaces (mesh, &copy);

	dgBooleanMeshClipper::AddExteriorFaces (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(NULL);
	return mesh;
*/
}

dgMeshEffect* dgMeshEffect::Difference (const dgMatrix& matrix, const dgMeshEffect* const clipperMesh) const
{
/*
	dgMeshEffect copy (*this);
	dgMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);

	dgMeshEffect* const mesh = new (GetAllocator()) dgMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddExteriorFaces (mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFacesInvertWinding (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(NULL);
	return mesh;
*/

	dgAssert (0);
	return NULL;
}


class dgBooleanMeshClipper: public dgMeshEffect::dgMeshBVH
{
	class dgNodeKey
	{
		public:
		dgNodeKey()
		{
		}

		dgNodeKey(const dgEdge* const edge, dgFloat64 param)
			:m_param (param)
			,m_edgeKey((edge->m_incidentVertex < edge->m_twin->m_incidentVertex) ? edge : edge->m_twin)
		{
			if (m_edgeKey != edge) {
				m_param = 1.0f - m_param;
			}
		}

		dgInt32 operator> (const dgNodeKey& key) const
		{
			if (key.m_edgeKey > m_edgeKey) {
				return 1;
			} else if (key.m_edgeKey == m_edgeKey) {
				return (key.m_param > m_param) ? 1 : 0;
			}
			return 0;
		}

		dgInt32 operator< (const dgNodeKey& key) const
		{
			if (key.m_edgeKey < m_edgeKey) {
				return 1;
			} else if (key.m_edgeKey == m_edgeKey) {
				return (key.m_param < m_param) ? 1 : 0;
			}
			return 0;
		}

		dgFloat64 m_param;
		const dgEdge* m_edgeKey;
	};

	class dgPoint
	{
		public:
		dgPoint()
			:m_links(NULL)
		{}

		dgPoint(dgMeshEffect* const edgeOwnerMesh, dgEdge* const edge, dgFloat64 param, dgMeshEffect* const faceOwnerMesh, dgEdge* const face)
			:m_edge(edge)
			,m_face(face)
			,m_edgeOwnerMesh(edgeOwnerMesh)
			,m_faceOwnerMesh(faceOwnerMesh)
			,m_links(edgeOwnerMesh->GetAllocator())
			,m_indexA(0)
			,m_indexB(0)
			,m_lru(0)
		{
			dgBigVector p0(m_edgeOwnerMesh->GetVertex(m_edge->m_incidentVertex));
			dgBigVector p1(m_edgeOwnerMesh->GetVertex(m_edge->m_twin->m_incidentVertex));
			m_posit = p0 + (p1 - p0).Scale3 (param);
		}

		dgBigVector m_posit;
		dgEdge* m_edge;
		dgEdge* m_face;
		dgMeshEffect* m_edgeOwnerMesh;
		dgMeshEffect* m_faceOwnerMesh;
		dgList<dgTree<dgPoint, dgNodeKey>::dgTreeNode*> m_links;
		dgInt32 m_indexA;
		dgInt32 m_indexB;
		dgInt32 m_lru;
	};

	class dgCurvesNetwork: public dgTree<dgPoint, dgNodeKey>
	{
		public:
		dgCurvesNetwork(dgBooleanMeshClipper* const BVHmeshA, dgBooleanMeshClipper* const BVHmeshB)
			:dgTree<dgPoint, dgNodeKey>(BVHmeshA->m_mesh->GetAllocator())
			,m_meshA(BVHmeshA->m_mesh)
			,m_meshB(BVHmeshB->m_mesh)
			,m_pointBaseA(m_meshA->GetVertexCount()) 
			,m_pointBaseB(m_meshB->GetVertexCount()) 
			,m_lru(0)
		{
		}

		dgHugeVector CalculateFaceNormal (const dgMeshEffect* const mesh, dgEdge* const face)
		{
			dgHugeVector plane(dgGoogol::m_zero, dgGoogol::m_zero, dgGoogol::m_zero, dgGoogol::m_zero);
			dgEdge* edge = face;
			dgHugeVector p0(mesh->GetVertex(edge->m_incidentVertex));
			edge = edge->m_next;
			dgHugeVector p1(mesh->GetVertex(edge->m_incidentVertex));
			dgHugeVector p1p0(p1 - p0);
			edge = edge->m_next;
			do {
				dgHugeVector p2(mesh->GetVertex(edge->m_incidentVertex));
				dgHugeVector p2p0(p2 - p0);
				plane += p2p0 * p1p0;
				p1p0 = p2p0;
				edge = edge->m_next;
			} while (edge != face);

			plane.m_w = dgGoogol::m_zero - (plane % p0);
			return plane;
		}


		bool IsPointInFace (const dgHugeVector& point, const dgMeshEffect* const mesh, dgEdge* const face, const dgHugeVector& normal) const 
		{
			dgEdge* edge = face;

			dgTrace (("%f %f %f\n", dgFloat64 (point.m_x), dgFloat64 (point.m_y), dgFloat64 (point.m_z)));
			do {
				dgBigVector p1(mesh->GetVertex(edge->m_incidentVertex));
				dgTrace (("%f %f %f\n", dgFloat64 (p1.m_x), dgFloat64 (p1.m_y), dgFloat64 (p1.m_z)));
				edge = edge->m_next;
			} while (edge != face);

			dgHugeVector p0(mesh->GetVertex(face->m_incidentVertex));
			do {
				dgHugeVector p1(mesh->GetVertex(edge->m_twin->m_incidentVertex));
				dgHugeVector p1p0(p1 - p0);
				dgHugeVector q1p0(point - p0);
				dgGoogol side (q1p0 % (normal * p1p0));
				if (side >= dgGoogol::m_zero) {
					return false;
				}
				p0 = p1;
				edge = edge->m_next;
			} while (edge != face);

			return true;
		}

		dgFloat64 ClipEdgeFace(const dgMeshEffect* const meshEdge, dgEdge* const edge, const dgMeshEffect* const meshFace, dgEdge* const face, const dgHugeVector& plane)
		{
			dgHugeVector p0 (meshEdge->GetVertex(edge->m_incidentVertex));
			dgHugeVector p1 (meshEdge->GetVertex(edge->m_twin->m_incidentVertex));
			
			dgGoogol test0 (plane.EvaluePlane(p0));
			dgGoogol test1 (plane.EvaluePlane(p1));

			if ((test0 * test1) > dgGoogol::m_zero) {
				// both point are in one side
				return -1.0f;
			}

			if ((test0 * test1) < dgGoogol::m_zero) {
				//point on different size, clip the line
				dgHugeVector p1p0 (p1 - p0);
				dgGoogol param = dgGoogol::m_zero - plane.EvaluePlane(p0) / (plane % p1p0);
				dgHugeVector p (p0 + p1p0.Scale3 (param));
				if (IsPointInFace (p, meshFace, face, plane)) {
					return param;
				}
				return -1.0f;
			} else {
				dgAssert (0);
				//special cases;
			}
			
			return -1.0f;
		}

		void AddPoint (dgMeshEffect* const edgeOwnerMesh, dgEdge* const edgeStart, dgMeshEffect* const faceOwnerMesh, dgEdge* const face, const dgHugeVector& plane, dgTreeNode** nodes, dgInt32& index)
		{
			dgEdge* edge = edgeStart;
			do {
				dgFloat64 param = ClipEdgeFace(edgeOwnerMesh, edge, faceOwnerMesh, face, plane);
				if (param > 0.0f) {
					dgPoint point(edgeOwnerMesh, edge, param, faceOwnerMesh, face);
					dgTreeNode* node = Find(dgNodeKey(edge, param));
					if (!node) {
						node = Insert(point, dgNodeKey(edge, param));
					}
					nodes[index] = node;
					index ++;
				}
				edge = edge->m_next;
			} while (edge != edgeStart);

		}

		void ClipMeshesFaces(dgEdge* const faceA, dgEdge* const faceB)
		{
			dgAssert (m_meshA->FindEdge(faceA->m_incidentVertex, faceA->m_twin->m_incidentVertex) == faceA);
			dgAssert (m_meshB->FindEdge(faceB->m_incidentVertex, faceB->m_twin->m_incidentVertex) == faceB);

			dgHugeVector planeA (CalculateFaceNormal (m_meshA, faceA));
			dgHugeVector planeB (CalculateFaceNormal (m_meshB, faceB));

			dgInt32 index = 0;
			dgTreeNode* nodes[16];
			AddPoint (m_meshA, faceA, m_meshB, faceB, planeB, nodes, index);
			AddPoint (m_meshB, faceB, m_meshA, faceA, planeA, nodes, index);
			dgAssert ((index == 0) || (index == 2));
			if (index == 2) {
				dgPoint& pointA = nodes[0]->GetInfo();
				dgPoint& pointB = nodes[1]->GetInfo();
				pointA.m_links.Append(nodes[1]);
				pointB.m_links.Append(nodes[0]);
			}
		}
/*
		void AddVertex()
		{
			Iterator iter (*this);
			dgInt32 indexA = m_meshA->GetVertexCount();
			dgInt32 indexB = m_meshB->GetVertexCount();
			for (iter.Begin(); iter; iter ++) {
				dgPoint& point = iter.GetNode()->GetInfo();
				m_meshA->AddVertex(point.m_posit);
				m_meshB->AddVertex(point.m_posit);
				point.m_indexA = indexA;
				point.m_indexB = indexB;
				indexA ++;
				indexB ++;
			}
		}
*/
		void GetCurve (dgList<dgTreeNode*>& curve, dgTreeNode* const node)
		{
			dgInt32 stack = 1;
			dgTreeNode* pool[64];

			pool[0] = node;
			while (stack) {
				stack --;
				dgTreeNode* const ptr = pool[stack];
				dgPoint& point = ptr->GetInfo();
				if (point.m_lru != m_lru) {
					point.m_lru = m_lru;
					curve.Append(ptr);
					for (dgList<dgTree<dgPoint, dgNodeKey>::dgTreeNode*>::dgListNode* ptrPoint = point.m_links.GetFirst(); ptrPoint; ptrPoint = ptrPoint->GetNext()) {
						dgTreeNode* const nextnode = ptrPoint->GetInfo();
						dgPoint& nextPoint = nextnode->GetInfo();
						if (nextPoint.m_lru != m_lru) {
							pool[stack] = nextnode;
							stack ++;
						}
					}
				}
			}
		}

		void EmbedCurveToSingleFace (dgList<dgTreeNode*>& curve, dgMeshEffect* const mesh)
		{
			dgEdge* const face = curve.GetFirst()->GetInfo()->GetInfo().m_face;

			dgInt32 indexBase = mesh->GetVertexCount();
			dgInt32 indexAttribBase = mesh->GetPropertiesCount();

			for (dgList<dgTreeNode*>::dgListNode* node = curve.GetFirst(); node; node = node->GetNext()) {
				dgPoint& point = node->GetInfo()->GetInfo();
				dgAssert (point.m_face == face);
				dgMeshEffect::dgVertexAtribute attribute(mesh->InterpolateVertex(point.m_posit, face));
				mesh->AddVertex(point.m_posit);
				mesh->AddAtribute(attribute);
			}

			dgList<dgEdge*> list(GetAllocator());
			dgInt32 i0 = curve.GetCount() - 1;
			for (dgInt32 i = 0; i < curve.GetCount(); i++) {
				dgEdge* const edge = mesh->AddHalfEdge(indexBase + i0, indexBase + i);
				dgEdge* const twin = mesh->AddHalfEdge(indexBase + i, indexBase + i0);

				edge->m_incidentFace = 1;
				twin->m_incidentFace = 1;
				edge->m_userData = indexAttribBase + i0;
				twin->m_userData = indexAttribBase + i;
				twin->m_twin = edge;
				edge->m_twin = twin;
				i0 = i;
				list.Append(edge);
			}

			dgEdge* closestEdge = NULL;
			dgFloat64 dist2 = dgFloat64 (1.0e10f);
			dgBigVector p(mesh->GetVertex(face->m_incidentVertex));

			list.Append(list.GetFirst()->GetInfo());
			list.Addtop(list.GetLast()->GetInfo());
			for (dgList<dgEdge*>::dgListNode* node = list.GetFirst()->GetNext(); node != list.GetLast(); node = node->GetNext()) {
				dgEdge* const edge = node->GetInfo();

				dgEdge* const prev = node->GetPrev()->GetInfo();
				edge->m_prev = prev;
				prev->m_next = edge;
				edge->m_twin->m_next = prev->m_twin;
				prev->m_twin->m_prev = edge->m_twin;

				dgEdge* const next = node->GetNext()->GetInfo();
				edge->m_next = next;
				next->m_prev = edge;
				edge->m_twin->m_prev = next->m_twin;
				next->m_twin->m_next = edge->m_twin;

				dgBigVector dist(mesh->GetVertex(edge->m_incidentVertex) - p);
				dgFloat64 err2 = dist % dist;
				if (err2 < dist2) {
					closestEdge = edge;
					dist2 = err2;
				}
			}

			dgBigVector faceNormal (mesh->FaceNormal(face, mesh->GetVertexPool(), mesh->GetVertexStrideInByte()));
			dgBigVector clipNormal (mesh->FaceNormal(closestEdge, mesh->GetVertexPool(), mesh->GetVertexStrideInByte()));
			if ((clipNormal % faceNormal) > dgFloat64(0.0f)) {
				closestEdge = closestEdge->m_twin->m_next;
			}
			dgEdge* const glueEdge = mesh->ConnectVertex (closestEdge, face);
			dgAssert (glueEdge);
			mesh->PolygonizeFace(glueEdge, mesh->GetVertexPool(), sizeof (dgBigVector));
		}

		void EmbedCurveToMulipleFaces (dgList<dgTreeNode*>& curve, dgMeshEffect* const mesh)
		{
//			dgInt32 indexBase = mesh->GetVertexCount();
//			dgInt32 indexAttribBase = mesh->GetPropertiesCount();

			for (dgList<dgTreeNode*>::dgListNode* node = curve.GetFirst(); node; node = node->GetNext()) {
				dgPoint& point = node->GetInfo()->GetInfo();
				if (point.m_edgeOwnerMesh == mesh) {
					dgEdge* const edge = point.m_edge;
					dgBigVector p0 (mesh->GetVertex(edge->m_incidentVertex));
					dgBigVector p1 (mesh->GetVertex(edge->m_twin->m_incidentVertex));
					dgVector p1p0 (p1 - p0);
					dgVector qp0 (point.m_posit - p0);
					dgFloat64 param = (qp0 % p1p0) / (p1p0 % p1p0);
					dgAssert (param >= dgFloat64 (0.0f));
					dgAssert (param <= dgFloat64 (1.0f));
					dgEdge* const newEdge = mesh->InsertEdgeVertex (edge, param);
				}
//				mesh->AddVertex(point.m_posit);
//				mesh->AddAtribute(attribute);
			}
		}


		void AddCurveToMesh (dgList<dgTreeNode*>& curve, dgMeshEffect* const mesh)
		{
			bool isIscribedInFace = true; 
			dgEdge* const face = curve.GetFirst()->GetInfo()->GetInfo().m_face;
			for (dgList<dgTreeNode*>::dgListNode* node = curve.GetFirst(); isIscribedInFace && node; node = node->GetNext()) {
				dgPoint& point = node->GetInfo()->GetInfo();
				isIscribedInFace = isIscribedInFace && (point.m_face == face);
				isIscribedInFace = isIscribedInFace && (point.m_faceOwnerMesh == mesh);
			}

			if (isIscribedInFace) {
				EmbedCurveToSingleFace (curve, mesh);
			} else {
				EmbedCurveToMulipleFaces (curve, mesh);
			}
		}

		void Colorize()
		{
			m_lru ++;
			Iterator iter (*this);
			for (iter.Begin(); iter; iter ++) {
				dgPoint& point = iter.GetNode()->GetInfo();
				if (point.m_lru != m_lru) {
					dgList<dgTreeNode*> curve (GetAllocator());
					GetCurve (curve, iter.GetNode());
					AddCurveToMesh (curve, m_meshB);
					AddCurveToMesh (curve, m_meshA);
				}
			}

			m_meshA->SaveOFF("xxxA0.off");
			m_meshB->SaveOFF("xxxB0.off");
		}


		dgMeshEffect* m_meshA;
		dgMeshEffect* m_meshB;
		dgInt32 m_pointBaseA;
		dgInt32 m_pointBaseB;
		dgInt32 m_lru;
	};


	public:
#if 0
	class dgClusterFace: public dgList<dgMeshBVHNode*>
	{
		public:
		dgClusterFace(dgMeshEffect* const mesh, dgEdge* const face)
			:dgList<dgMeshBVHNode*>(mesh->GetAllocator())
			,m_plane (mesh->FaceNormal (face, mesh->GetVertexPool(), sizeof (dgBigVector)), dgFloat32(0.0))
		{
			const dgBigVector* const vertex = (dgBigVector*) mesh->GetVertexPool();
			m_plane = m_plane.Scale(1.0 / sqrt(m_plane % m_plane));
			m_plane.m_w = - (m_plane % vertex[face->m_incidentVertex]);
		}

		bool AddFace (const dgMeshEffect* const mesh, dgMeshBVHNode* const faceNode, const dgMeshEffect* const myMesh, dgMeshBVHNode* const myFaceNode) 
		{
			dgAssert (!Find(faceNode));

			bool added = false;
			dgInt32 posCount = 0;
			dgInt32 negCount = 0;
			const dgBigVector* const vertex = (dgBigVector*) mesh->GetVertexPool();
			dgEdge* ptr = faceNode->m_face;
			do {
				dgFloat64 test = m_plane.Evalue(vertex[ptr->m_incidentVertex]);
				posCount += (test > -1.0e-3);
				negCount += (test <  1.0e-3);
				ptr = ptr->m_next;
			} while (ptr != faceNode->m_face);

			if (posCount * negCount) {
				dgBigPlane plane (mesh->FaceNormal (faceNode->m_face, &vertex[0].m_x, sizeof (dgBigVector)), dgFloat32 (0.0f));
				plane = plane.Scale(1.0 / sqrt(plane % plane));
				plane.m_w = - (plane % vertex[faceNode->m_face->m_incidentVertex]);

				dgInt32 posCount = 0;
				dgInt32 negCount = 0;
				const dgBigVector* const myVertex = (dgBigVector*) myMesh->GetVertexPool();
				dgEdge* ptr = myFaceNode->m_face;
				do {
					dgFloat64 test = plane.Evalue (myVertex[ptr->m_incidentVertex]);
					posCount += (test > -1.0e-3);
					negCount += (test <  1.0e-3);
					ptr = ptr->m_next;
				} while (ptr != myFaceNode->m_face);

				if (posCount * negCount) {
					Append(faceNode);
					added = true;
				}
			}

			return added;
		}

		dgEdge* FindFirstClipperEdge (const dgHugeVector& normal, const dgHugeVector& origin, const dgBooleanMeshClipper* const clipperMeshBVH, dgInt32 clipperMark) const
		{
/*
			dgEdge* vertexClipperEdge = NULL;
//normal.Trace();
//origin.Trace();

			for (dgListNode* clipperNode = GetFirst(); clipperNode; clipperNode = clipperNode->GetNext()) {
				dgMeshBVHNode* const clipperFaceNode = clipperNode ->GetInfo();

				dgEdge* ptr = clipperFaceNode->m_face;
//clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex].Trace();
				dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
				do {
					dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_next->m_incidentVertex] - origin));
//test0.Trace();
//test1.Trace();
					if (ptr->m_mark != clipperMark) {
						if ((test0 <= dgGoogol (m_negTol)) && (test1 >= dgGoogol (m_posTol))) {
							return ptr;
						} else if ((test0.Abs() < dgGoogol (m_posTol)) && (test1 >= dgGoogol (m_posTol))) {
							dgAssert (0);
							vertexClipperEdge = ptr;
						}
					}

					test0 = test1;
					ptr = ptr->m_next;
				} while (ptr != clipperFaceNode->m_face);
			}
			return vertexClipperEdge;
		}

		dgEdge* EmitPointOnPlaneAndGetNextCrossingEdge (const dgHugeVector& normal, const dgHugeVector& origin, const dgBooleanMeshClipper* const clipperMeshBVH, dgEdge* const clipperEdge, dgInt32 clipperMark, dgList<dgHugeVector>& curve) const
		{
			dgHugeVector& point = curve.Addtop()->GetInfo();
			point = clipperMeshBVH->m_vertexAlias[clipperEdge->m_twin->m_incidentVertex];

			dgEdge* ptr = clipperEdge;
			do {
				ptr->m_mark = clipperMark;
				ptr->m_twin->m_mark = clipperMark;
				ptr = ptr->m_next;
			} while (ptr != clipperEdge);

			dgEdge* nexEdge = NULL;
			ptr = clipperEdge->m_next->m_twin->m_next;
			do {
				ptr->m_mark = clipperMark;
				ptr->m_twin->m_mark = clipperMark;

				if (!nexEdge) {
					dgEdge* ptr1 = ptr->m_next;
					dgAssert (ptr1);
					dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr1->m_incidentVertex] - origin));
					do {
						dgAssert (ptr1->m_next);
						dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr1->m_next->m_incidentVertex] - origin));
						if (test0 >= m_posTol) {
							if (test1.Abs() < m_posTol) {
								nexEdge = EmitPointOnPlaneAndGetNextCrossingEdge (normal, origin, clipperMeshBVH, ptr1, clipperMark, curve);
							} else if (test1 <= m_negTol) {
								nexEdge = ptr1;
							}
						}

						test1 = test0;
						ptr1 = ptr1->m_next;
					} while (ptr1 != ptr->m_prev);
				}

				ptr = ptr->m_twin->m_next;
			} while (ptr != clipperEdge->m_next);

			dgAssert (nexEdge);
			return nexEdge;
		}


		bool Get2DCurve (const dgHugeVector& normal, const dgHugeVector& origin, const dgBooleanMeshClipper* const clipperMeshBVH, dgEdge* const clipperEdge, dgInt32 clipperMark, dgList<dgHugeVector>& curve) const
		{
			bool isClosedLoop = false;

static int xxx;
xxx ++;
if (xxx == 3){
clipperMeshBVH->m_mesh->Trace();
}

			dgInt32 clusterColor = clipperMark -1;	

			dgEdge* firstClipperEdge = clipperEdge;
			dgAssert (firstClipperEdge->m_mark >= clusterColor);

			for (bool state = true; state; ) {
				state = false;

				dgHugeVector p1p0 (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_next->m_incidentVertex] - clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex]);
				dgGoogol num (normal % (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - origin));
				dgGoogol den (normal % p1p0);
				dgHugeVector& point = curve.Addtop()->GetInfo();
				point = clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - p1p0.Scale3(num / den);

				if (firstClipperEdge->m_mark >= clusterColor) {
					firstClipperEdge->m_mark = clipperMark;
					firstClipperEdge->m_twin->m_mark = clipperMark;

					dgEdge* ptr = firstClipperEdge->m_next;
					dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
					do {
						dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_next->m_incidentVertex] - origin));
						dgAssert (ptr->m_mark >= clusterColor);
						if (test1.Abs() < m_posTol) {
							dgAssert (test0 >= m_posTol);
							state = true;
							firstClipperEdge = EmitPointOnPlaneAndGetNextCrossingEdge (normal, origin, clipperMeshBVH, ptr, clipperMark, curve);
							firstClipperEdge = firstClipperEdge->m_twin;
							break;

						} else if ((test0 >= m_posTol) && (test1 <= m_negTol)) {
							if (ptr->m_mark != clipperMark) {
								state = true;
								firstClipperEdge = ptr->m_twin;
							} else {
								isClosedLoop = true;
							}
							break;
						}

						test0 = test1;
						ptr = ptr->m_next;
					} while (ptr != firstClipperEdge);
				}
			}

			if (!isClosedLoop) {
				dgEdge* firstClipperEdge = clipperEdge->m_twin->m_next;
				for (bool state = true; state; ) {
					state = false;
					if (firstClipperEdge->m_mark >= clusterColor) {
						dgAssert (0);
/*
						firstClipperEdge->m_mark = clipperMark;
						firstClipperEdge->m_twin->m_mark = clipperMark;

						dgEdge* ptr = firstClipperEdge->m_next;
						dgGoogol test0 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
						do {
							dgGoogol test1 (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_next->m_incidentVertex] - origin));
							dgAssert (ptr->m_mark >= clusterColor);
							if ((test0 >= dgGoogol (m_posTol)) && (test1 <= dgGoogol (m_negTol))) {
								if (ptr->m_mark != clipperMark) {
									state = true;
									firstClipperEdge = ptr->m_twin;
								} else {
									isClosedLoop = true;
								}
								break;
							}

							test0 = test1;
							ptr = ptr->m_next;
						} while (ptr != firstClipperEdge);

//						dgHugeVector p1p0 (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_next->m_incidentVertex] - clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex]);
//						dgGoogol num (normal % (clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - origin  ));
//						dgGoogol den (normal % p1p0);
//						dgHugeVector& point = curve.Addtop()->GetInfo();
//						point = clipperMeshBVH->m_vertexAlias[firstClipperEdge->m_incidentVertex] - p1p0.Scale(num / den);
*/
					}
				}
			}
			return isClosedLoop;
		}

		bool TestPlanar (const dgHugeVector& normal, const dgList<dgHugeVector>& curve) const
		{
			dgHugeVector n (0.0, 0.0, 0.0, 0.0);
			const dgHugeVector& p0 = curve.GetFirst()->GetInfo();
			const dgHugeVector& p1 = curve.GetFirst()->GetNext()->GetInfo();

//			p0.Trace();
//			p1.Trace();
			dgHugeVector e10 (p1 - p0);
			for (dgList<dgHugeVector>::dgListNode* node = curve.GetFirst()->GetNext()->GetNext(); node; node = node->GetNext()) {
				const dgHugeVector& p2 = node->GetInfo();
//				p2.Trace();
				dgHugeVector e20 (p2 - p0);
				n += e10 * e20;
				e10 = e20;
			}

			dgGoogol dir (n % normal);
			return dir > dgGoogol::m_zero;
		}


		void RayRayIntersect (const dgHugeVector& ray_p0, const dgHugeVector& ray_p1, dgGoogol& paramP, const dgHugeVector& ray_q0, const dgHugeVector& ray_q1, dgGoogol& paramQ) const
		{
			dgHugeVector p1p0 (ray_p1 - ray_p0);
			dgHugeVector q1q0 (ray_q1 - ray_q0);
			dgHugeVector p0q0 (ray_p0 - ray_q0);

			dgGoogol a = p1p0 % p1p0;        // always >= 0
			dgGoogol c = q1q0 % q1q0;        // always >= 0
			dgGoogol b = p1p0 % q1q0;

			dgGoogol d = (p1p0 % p0q0);
			dgGoogol e = (q1q0 % p0q0);
			dgGoogol den = a * c - b * b;   // always >= 0

			// compute the line parameters of the two closest points
			dgAssert (den > m_tol3);

			// get the closest points on the infinite lines
			paramP = (b * e - c * d) / den;
			paramQ = (a * e - b * d) / den;
			paramP = dgClamp (paramP, dgGoogol::m_zero, dgGoogol::m_one);
			paramQ = dgClamp (paramQ, dgGoogol::m_zero, dgGoogol::m_one);
		}

		void InsertOpenCurve (dgBooleanMeshClipper* const meshBVH, dgList<dgEdge*>& facePerimeter, const dgHugeVector& normal, const dgHugeVector& origin, dgList<dgHugeVector>& curve) const
		{
			dgInt32 edgeConnectCount = 0;
			dgEdge* edgeConnect[4];
			const dgHugeVector* const vertex = &meshBVH->m_vertexAlias[0];
			for (dgList<dgEdge*>::dgListNode* node = facePerimeter.GetFirst(); node; node = node->GetNext()) {
				dgEdge* const edge = node->GetInfo();

				dgInt32 index0 = edge->m_incidentVertex;
				dgInt32 index1 = edge->m_next->m_incidentVertex;
				const dgHugeVector& q0 (vertex[index0]);
				const dgHugeVector& q1 (vertex[index1]);

				for (dgList<dgHugeVector>::dgListNode* curveNode = curve.GetFirst(); curveNode != curve.GetLast(); curveNode = curveNode->GetNext()) {
					const dgHugeVector& p0 = curveNode->GetInfo();
					const dgHugeVector& p1 = curveNode->GetNext()->GetInfo();

					dgHugeVector q0p0 (q0 - p0);
					dgHugeVector q1p0 (q1 - p0);
					dgGoogol alpha0 ((normal * q0p0) % q1p0);

					dgHugeVector q0p1 (q0 - p1);
					dgHugeVector q1p1 (q1 - p1);
					dgGoogol alpha1 ((normal * q0p1) % q1p1);

					bool test0 = (alpha0 <= m_tol2);
					bool test1 = (alpha1 <= m_tol2);
					if (test0 ^ test1) {
						dgGoogol paramP;
						dgGoogol paramQ;
						RayRayIntersect (q0, q1, paramQ, p0, p1, paramP);

						bool test0ParamP = paramP >= m_posTol;
						bool test1ParamP = paramP <= m_oneMinusTol;
						bool test0ParamQ = paramQ >= m_posTol;
						bool test1ParamQ = paramQ <= m_oneMinusTol;

						if (test0ParamP & test1ParamP & test0ParamQ & test1ParamQ) {
							dgEdge* edge0;
							dgEdge* edge1;

							meshBVH->SpliteEdge(edge, paramQ, &edge0, &edge1);
							edgeConnect[edgeConnectCount] = edge1;
							edgeConnectCount ++; 

							node->GetInfo() = edge0;
							facePerimeter.InsertAfter(node, facePerimeter.Append(edge1));
							curve.InsertAfter (curveNode, curve.Append(meshBVH->m_vertexAlias[meshBVH->m_mesh->GetVertexCount()-1]));

						} else if (!((test0ParamP ^ test1ParamP) & (test0ParamQ ^ test1ParamQ))) {
							dgHugeVector p (p0 + (p1 - p0).Scale3 (paramP));
							dgHugeVector q (q0 + (q1 - q0).Scale3 (paramQ));
							dgHugeVector pq = (p - q);
							dgGoogol dist2 (pq % pq);
							if (dist2 < m_tol2) {
								if (test0ParamQ ^ test1ParamQ) {
									dgAssert (test0ParamP & test1ParamP);
									if (!test0ParamQ) {
										dgAssert (test1ParamQ);
										edgeConnect[edgeConnectCount] = edge;
										edgeConnectCount ++; 
										curve.InsertAfter (curveNode, curve.Append(meshBVH->m_vertexAlias[edge->m_next->m_incidentVertex]));
									} else {
										dgAssert (!test1ParamQ);
										edgeConnect[edgeConnectCount] = edge->m_next;
										edgeConnectCount ++; 
										curve.InsertAfter (curveNode, curve.Append(meshBVH->m_vertexAlias[edge->m_next->m_incidentVertex]));
									}
								} else {
									dgAssert (0);
									dgAssert (test0ParamP ^ test1ParamP);
									dgAssert (test0ParamQ & test1ParamQ);
								}
							}
						}

						break;
					}
				}
			}

			dgList<dgHugeVector>::dgListNode* nextNode;
			for (dgList<dgHugeVector>::dgListNode* node = curve.GetFirst(); node; node = nextNode) {
				nextNode = node->GetNext();
				const dgHugeVector& p0 = node->GetInfo();

				for (dgList<dgEdge*>::dgListNode* faceNode = facePerimeter.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
					dgEdge* const edge = faceNode->GetInfo();

					dgInt32 index0 = edge->m_incidentVertex;
					dgInt32 index1 = edge->m_next->m_incidentVertex;
					const dgHugeVector& q0 (vertex[index0]);
					const dgHugeVector& q1 (vertex[index1]);
					dgHugeVector q0p0 (q0 - p0);
					dgHugeVector q1p0 (q1 - p0);
					dgGoogol alpha0 ((normal * q1p0) % q0p0);
					if (alpha0 >= m_tol2) {
						curve.Remove(node);
						break;
					}
				}
			}


			dgAssert (edgeConnectCount == 2);
			if (curve.GetCount() == 2) {
				meshBVH->m_mesh->ConnectVertex(edgeConnect[0], edgeConnect[1]);
			} else {
				dgAssert (0);
			}
		}


		void InsertClosedCurve (dgBooleanMeshClipper* const meshBVH, dgList<dgEdge*>& facePerimeter, const dgHugeVector& normal, const dgHugeVector& origin, const dgList<dgHugeVector>& clipperCurve) const
		{
			dgAssert (TestPlanar (normal, clipperCurve));

			bool isInside = true;
			const dgHugeVector* const vertex = &meshBVH->m_vertexAlias[0];
			for (dgList<dgEdge*>::dgListNode* node = facePerimeter.GetFirst(); isInside && node; node = node->GetNext()) {
				dgEdge* const edge = node->GetInfo();
				dgInt32 index0 = edge->m_incidentVertex;
				dgInt32 index1 = edge->m_next->m_incidentVertex;

				const dgHugeVector& q0 (vertex[index0]);
				const dgHugeVector& q1 (vertex[index1]);
				for (dgList<dgHugeVector>::dgListNode* clipperNode = clipperCurve.GetFirst(); clipperNode; clipperNode = clipperNode->GetNext()) {
					const dgHugeVector& p0 = clipperNode->GetInfo();
					dgHugeVector q0p0 (q0 - p0);
					dgHugeVector q1p0 (q1 - p0);
					dgGoogol alpha ((normal * q0p0) % q1p0);
					if (alpha <= m_tol2) {
						isInside = false;
						break;
					}
				}
			} 

			if (isInside) {
				dgInt32 face[1024];
				dgInt64 attrb[1024];

				dgInt32 indexCount = 0;
				dgEdge* const exteriorFace = facePerimeter.GetFirst()->GetInfo();
				for (dgList<dgHugeVector>::dgListNode* clipperNode = clipperCurve.GetFirst(); clipperNode; clipperNode = clipperNode->GetNext()) {
					meshBVH->InsertFaceVertex (exteriorFace, clipperNode->GetInfo());
					face[indexCount] = meshBVH->m_mesh->GetVertexCount() - 1;
					attrb[indexCount] = meshBVH->m_mesh->GetPropertiesCount() - 1;
					indexCount ++;
				}
				dgEdge* const interiorFace = meshBVH->m_mesh->AddFace(indexCount, face, attrb);
				dgEdge* ptr = interiorFace;
				do {
					ptr->m_twin = meshBVH->m_mesh->AddHalfEdge(ptr->m_next->m_incidentVertex, ptr->m_incidentVertex);
					dgAssert (ptr->m_twin);
					ptr->m_twin->m_twin = ptr;
					ptr = ptr->m_next;
				} while (ptr != interiorFace);

				ptr = interiorFace;
				do {
					ptr->m_twin->m_incidentFace = ptr->m_incidentFace;
					ptr->m_twin->m_userData = ptr->m_next->m_userData;

					ptr->m_twin->m_prev = ptr->m_next->m_twin;
					ptr->m_next->m_twin->m_next = ptr->m_twin;

					ptr = ptr->m_next;
				} while (ptr != interiorFace);

				dgGoogol error2 (1.0e20);
				dgEdge* closestEdge = NULL;
				const dgHugeVector* const vertex = &meshBVH->m_vertexAlias[0];
				const dgHugeVector& p0 = vertex[exteriorFace->m_incidentVertex];
				ptr = interiorFace;
				do {
					const dgHugeVector& q0 = vertex[ptr->m_incidentVertex];
					dgHugeVector dist (q0 - p0);
					dgGoogol mag2 (dist % dist);

					if (mag2 < error2) {
						error2 = mag2;
						closestEdge = ptr;
					}

					ptr->m_twin->m_incidentFace = interiorFace->m_incidentFace;
					ptr = ptr->m_next;
				} while (ptr != interiorFace);

				dgAssert (closestEdge);
				closestEdge = closestEdge->m_twin->m_next;
				closestEdge = meshBVH->m_mesh->ConnectVertex (exteriorFace, closestEdge);
				dgAssert (closestEdge);

				meshBVH->m_mesh->TriangulateFace(interiorFace, meshBVH->m_mesh->GetVertexPool(), sizeof (dgBigVector));
				meshBVH->m_mesh->TriangulateFace(closestEdge, meshBVH->m_mesh->GetVertexPool(), sizeof (dgBigVector));

			} else {
				dgAssert (0);
			}
		}



		bool ClipFace (dgMeshBVHNode* const faceNode, dgBooleanMeshClipper* const meshBVH, const dgBooleanMeshClipper* const clipperMeshBVH)
		{
			dgHugeVector normal (meshBVH->FaceNormal (faceNode->m_face));
			dgGoogol mag2 (normal % normal);
			normal = normal.Scale3 (mag2.InvSqrt());
			const dgHugeVector& origin (meshBVH->m_vertexAlias[faceNode->m_face->m_incidentVertex]);

			dgClusterFace::dgListNode* nextNode;
			dgInt32 clusterColor = clipperMeshBVH->m_mesh->IncLRU();
			for (dgClusterFace::dgListNode* node = GetFirst(); node; node = nextNode) {
				dgMeshBVHNode* const clipperFaceNode = node->GetInfo();
				nextNode = node->GetNext();

				dgInt32 posCount = 0;
				dgInt32 negCount = 0;
				dgInt32 onPlaneCount = 0;
				dgEdge* ptr = clipperFaceNode->m_face;
				do {
					ptr->m_mark = clusterColor;
					dgGoogol test (normal % (clipperMeshBVH->m_vertexAlias[ptr->m_incidentVertex] - origin));
					bool pos = (test > dgBooleanMeshClipper::m_posTol);
					bool neg = (test < dgBooleanMeshClipper::m_negTol);
					posCount += pos;
					negCount += neg;
					onPlaneCount += !(pos || neg);
					ptr = ptr->m_next;
				} while (ptr != clipperFaceNode->m_face);


				if (!((posCount && negCount) || (posCount && onPlaneCount) || (negCount && onPlaneCount))) {
					dgAssert (0);
					Remove(node);
				}
			}


			if (GetCount()) {
				dgList<dgEdge*> facePerimeter (clipperMeshBVH->m_mesh->GetAllocator());
				dgEdge* perimterEdge = faceNode->m_face;
				do {
					facePerimeter.Append(perimterEdge);
					perimterEdge = perimterEdge->m_next;
				} while (perimterEdge != faceNode->m_face);

				dgPolyhedra::dgPairKey key (faceNode->m_face->m_incidentVertex, faceNode->m_face->m_twin->m_incidentVertex);

				dgInt32 clipperMark = clipperMeshBVH->m_mesh->IncLRU();
				for (bool found = true; found;) {
					found = false;
					dgList<dgHugeVector> clipperCurve (clipperMeshBVH->m_mesh->GetAllocator());

					dgEdge* const firstClipperEdge = FindFirstClipperEdge (normal, origin, clipperMeshBVH, clipperMark);
					if (firstClipperEdge) {
						dgAssert (firstClipperEdge->m_mark == clusterColor);
						found = true;
						bool isClosedLoop = Get2DCurve (normal, origin, clipperMeshBVH, firstClipperEdge, clipperMark, clipperCurve);
						if (clipperCurve.GetCount() >= 2) {
							if (isClosedLoop) {
								InsertClosedCurve (meshBVH, facePerimeter, normal, origin, clipperCurve);
							} else {
								InsertOpenCurve (meshBVH, facePerimeter, normal, origin, clipperCurve);
							}
						}
					}
				}

				meshBVH->m_nodeEdgeMap.Remove(key.GetVal());
				meshBVH->RemoveNode(faceNode);

				dgInt32 stack = 0;
				dgEdge* stackPool[512];
				dgInt32 mark = meshBVH->m_mesh->IncLRU();
				for (dgList<dgEdge*>::dgListNode* node = facePerimeter.GetFirst(); node; node = node->GetNext()) {
					dgEdge* const edge = node->GetInfo();
					edge->m_twin->m_mark = mark;
					stackPool[stack] = edge;
					stack ++;
				}

				while (stack) {
					stack --;
					dgEdge* const face = stackPool[stack];
					if (face->m_mark != mark) {
						dgMeshBVHNode* const newNode = meshBVH->AddFaceNode (face, NULL);
						dgPolyhedra::dgPairKey key(newNode->m_face->m_incidentVertex, newNode->m_face->m_twin->m_incidentVertex);
						meshBVH->m_nodeEdgeMap.Insert(newNode, key.GetVal());
					}

					dgEdge* ptr = face;
					do {
						ptr->m_mark = mark;
						if (ptr->m_twin->m_mark != mark) {
							stackPool[stack] = ptr->m_twin;
							stack ++;
						}
						ptr = ptr->m_next;
					} while (ptr != face);
				}
			}

			return false;
		}


		dgBigPlane m_plane;
	};

	class dgBooleanFaceCluster: public dgTree<dgClusterFace, dgMeshBVHNode*>
	{
		public:
		dgBooleanFaceCluster (dgMemoryAllocator* const allocator)
			:dgTree<dgClusterFace, dgMeshBVHNode*>(allocator)
		{
		}

		bool ClipCluster (dgTreeNode* const clusterNode, dgBooleanMeshClipper* const meshBVH, const dgBooleanMeshClipper* const clipperMeshBVH)
		{
			dgClusterFace& cluster = clusterNode->GetInfo();
			dgMeshBVHNode* const faceNode = clusterNode->GetKey();
			bool ret = cluster.ClipFace(faceNode, meshBVH, clipperMeshBVH);
			Remove (clusterNode);
			return ret;
		}
	};


	void Cleanup ()
	{
		dgMeshBVH::Cleanup();
	}


	bool SanityCheck() const
	{
		#ifdef _DEBUG
			dgAssert (dgMeshBVH::SanityCheck()); 
			dgMeshBVHNode* stackPool[DG_MESH_EFFECT_BVH_STACK_DEPTH];
			dgInt32 stack = 1;
			stackPool[0] = m_rootNode;
			dgVector l0(dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (-1.0e15f), dgFloat32 (0.0f)); 
			dgVector l1(dgFloat32 (1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 ( 1.0e15f), dgFloat32 (0.0f)); 

			while (stack) {
				stack --;
				dgMeshBVHNode* const me = stackPool[stack];
				if (me && dgOverlapTest (me->m_p0, me->m_p1, l0, l1)) {
					if (!me->m_left) {
						dgAssert (!me->m_right);
						dgPolyhedra::dgPairKey key (me->m_face->m_incidentVertex, me->m_face->m_twin->m_incidentVertex);
						dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const node = m_nodeEdgeMap.Find(key.GetVal());
						dgAssert (node);
						dgAssert (node->GetInfo() == me);
					} else {
						dgAssert (me->m_left);
						dgAssert (me->m_right);
						stackPool[stack] = me->m_left;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

						stackPool[stack] = me->m_right;
						stack++;
						dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
					}
				}
			}
		#endif
		return true;
	}


	dgHugeVector FaceNormal (const dgEdge* const face) const
	{
		const dgEdge* edge = face;
		const dgHugeVector& p0 = m_vertexAlias[edge->m_incidentVertex];

		edge = edge->m_next;
		const dgHugeVector& p1 = m_vertexAlias[edge->m_incidentVertex];
		dgHugeVector e1 (p1 - p0);

		dgHugeVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
		for (edge = edge->m_next; edge != face; edge = edge->m_next) {
			const dgHugeVector& p2 = m_vertexAlias[edge->m_incidentVertex]; 
			dgHugeVector e2 (p2 - p0);
			normal += e1 * e2;
			e1 = e2;
		} 
		return normal;
	}

	void InsertFaceVertex (dgEdge* const face, const dgHugeVector& point)
	{
		dgMeshEffect::dgVertexAtribute attribute;
		memset (&attribute, 0, sizeof (attribute));

		dgGoogol tol(m_posTol);
		const dgMeshEffect::dgVertexAtribute* const attrib = (dgMeshEffect::dgVertexAtribute*)m_mesh->GetAttributePool(); 
		for (dgInt32 i = 0; i < 4; i ++) {

			dgGoogol posTol (tol + dgGoogol::m_one);
			dgGoogol negTol (dgGoogol::m_zero - m_posTol);

			dgEdge* ptr = face;
			dgEdge* const edge0 = ptr;
			dgHugeVector q0 (m_vertexAlias[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dgEdge* edge1 = ptr;
			dgHugeVector q1 (m_vertexAlias[ptr->m_incidentVertex]);

			ptr = ptr->m_next;
			const dgEdge* edge2 = ptr;
			do {
				const dgHugeVector& q2 = m_vertexAlias[ptr->m_incidentVertex];

				dgHugeVector p10 (q1 - q0);
				dgHugeVector p20 (q2 - q0);

				dgGoogol dot = p20 % p10;
				dgGoogol mag1 = p10 % p10;
				dgGoogol mag2 = p20 % p20;
				dgGoogol collinear = dot * dot - mag2 * mag1;
				if (collinear.Abs() > m_posTol) {
					dgHugeVector p_p0 (point - q0);
					dgHugeVector p_p1 (point - q1);
					dgHugeVector p_p2 (point - q2);

					dgGoogol alpha1 = p10 % p_p0;
					dgGoogol alpha2 = p20 % p_p0;
					dgGoogol alpha3 = p10 % p_p1;
					dgGoogol alpha4 = p20 % p_p1;
					dgGoogol alpha5 = p10 % p_p2;
					dgGoogol alpha6 = p20 % p_p2;

					dgGoogol vc = alpha1 * alpha4 - alpha3 * alpha2;
					dgGoogol vb = alpha5 * alpha2 - alpha1 * alpha6;
					dgGoogol va = alpha3 * alpha6 - alpha5 * alpha4;
					dgGoogol den = va + vb + vc;
					dgGoogol minError = den * negTol;
					dgGoogol maxError = den * posTol;

					if ((va > minError) && (vb > minError) && (vc > minError) && (va < maxError) && (vb < maxError) && (vc < maxError)) {

						edge2 = ptr;

						dgGoogol alpha0 = va / den;
						dgGoogol alpha1 = vb / den;
						dgGoogol alpha2 = vc / den;

						dgFloat64 falpha0 = alpha0;
						dgFloat64 falpha1 = alpha1;
						dgFloat64 falpha2 = alpha2;

						const dgMeshEffect::dgVertexAtribute& attr0 = attrib[edge0->m_userData];
						const dgMeshEffect::dgVertexAtribute& attr1 = attrib[edge1->m_userData];
						const dgMeshEffect::dgVertexAtribute& attr2 = attrib[edge2->m_userData];
						dgBigVector normal (attr0.m_normal_x * falpha0 + attr1.m_normal_x * falpha1 + attr2.m_normal_x * falpha2,
											attr0.m_normal_y * falpha0 + attr1.m_normal_y * falpha1 + attr2.m_normal_y * falpha2,
											attr0.m_normal_z * falpha0 + attr1.m_normal_z * falpha1 + attr2.m_normal_z * falpha2, dgFloat32 (0.0f));
						normal = normal.Scale3 (dgFloat64 (1.0f) / sqrt (normal % normal));


						attribute.m_vertex.m_x = point.m_x;
						attribute.m_vertex.m_y = point.m_y;
						attribute.m_vertex.m_z = point.m_z;
						attribute.m_vertex.m_w = point.m_w;
						attribute.m_normal_x = normal.m_x;
						attribute.m_normal_y = normal.m_y;
						attribute.m_normal_z = normal.m_z;
						attribute.m_u0 = attr0.m_u0 * falpha0 +  attr1.m_u0 * falpha1 + attr2.m_u0 * falpha2;
						attribute.m_v0 = attr0.m_v0 * falpha0 +  attr1.m_v0 * falpha1 + attr2.m_v0 * falpha2;
						attribute.m_u1 = attr0.m_u1 * falpha0 +  attr1.m_u1 * falpha1 + attr2.m_u1 * falpha2;
						attribute.m_v1 = attr0.m_v1 * falpha0 +  attr1.m_v1 * falpha1 + attr2.m_v1 * falpha2;

						attribute.m_material = attr0.m_material;
						dgAssert (attr0.m_material == attr1.m_material);
						dgAssert (attr0.m_material == attr2.m_material);

						m_mesh->AddPoint(&attribute.m_vertex.m_x, dgInt32 (attribute.m_material)); 
						m_vertexAlias[m_mesh->GetVertexCount()-1] = point;
						return;
					}
				}

				q1 = q2;
				edge1 = ptr;

				ptr = ptr->m_next;
			} while (ptr != face);
			dgAssert (0);
			tol = tol * dgGoogol::m_two;
		}
		// this should never happens
		dgAssert (0);
	}

	void SpliteEdge(dgEdge* const edge, dgGoogol param, dgEdge** edge0, dgEdge** edge1)
	{
		dgInt32 v0 = edge->m_incidentVertex;
		dgInt32 v1 = edge->m_twin->m_incidentVertex;
		dgPolyhedra::dgPairKey edgeKey (v0, v1);
		dgPolyhedra::dgPairKey twinKey (v1, v0);

		dgEdge* const newEdge = m_mesh->InsertEdgeVertex (edge, param);
		dgGoogol t0(dgGoogol::m_one - param);
		dgGoogol t1(param);
		dgInt32 v01 = newEdge->m_twin->m_incidentVertex;
		m_vertexAlias[v01] = m_vertexAlias[v0].Scale3 (t0) + m_vertexAlias[v1].Scale3 (t1);

		dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const mapNode = m_nodeEdgeMap.Find(edgeKey.GetVal());
		if (mapNode) {
			dgMeshBVHNode* const node = mapNode->GetInfo();
			node->m_face = newEdge;
			m_nodeEdgeMap.Remove(mapNode);
			dgPolyhedra::dgPairKey key (newEdge->m_incidentVertex, newEdge->m_twin->m_incidentVertex);
			m_nodeEdgeMap.Insert(node, key.GetVal());
		}

		dgTree<dgMeshBVHNode*, dgUnsigned64>::dgTreeNode* const twinMapNode = m_nodeEdgeMap.Find(twinKey.GetVal());
		if (twinMapNode) {
			dgMeshBVHNode* const node = twinMapNode->GetInfo();
			node->m_face = newEdge->m_twin;
			m_nodeEdgeMap.Remove(twinMapNode);
			dgPolyhedra::dgPairKey key (newEdge->m_twin->m_incidentVertex, newEdge->m_incidentVertex);
			m_nodeEdgeMap.Insert(node, key.GetVal());
		}

		*edge0 = newEdge;
		*edge1 = newEdge->m_next;

		dgAssert (SanityCheck());
	}

	bool ClippMesh (const dgBooleanMeshClipper& otherMeshBVH)
	{
		dgMeshBVHNode* stackPool[4 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];
	

		dgInt32 stack = 1;
		stackPool[0][0] = m_rootNode;
		stackPool[0][1] = otherMeshBVH.m_rootNode;

		dgBooleanFaceCluster bundaryClusters (m_mesh->GetAllocator());
		while (stack) {
			stack --;
			dgMeshBVHNode* const me = stackPool[stack][0];
			dgMeshBVHNode* const other = stackPool[stack][1];

			dgAssert (me && other);
			if (dgOverlapTest (me->m_p0, me->m_p1, other->m_p0, other->m_p1)) {

				if (!me->m_left && !other->m_left) {
					dgAssert (!me->m_right);
					dgAssert (!other->m_right);

					dgBooleanFaceCluster::dgTreeNode* node = bundaryClusters.Find(me);
					if (!node) {
						dgClusterFace tmp (m_mesh, me->m_face);
						node = bundaryClusters.Insert(tmp, me);
					}
					if (!node->GetInfo().AddFace (otherMeshBVH.m_mesh, other, m_mesh, me)) {
						if (!node->GetInfo().GetCount()) {
							bundaryClusters.Remove(node);
						}
					}

				} else if (!me->m_left) {
					dgAssert (other->m_left);
					dgAssert (other->m_right);

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

				} else if (!other->m_right) {
					dgAssert (me->m_left);
					dgAssert (me->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				} else {
					dgAssert (me->m_left && me->m_right);
					dgAssert (other->m_left && other->m_right);

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_left;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_left;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));

					stackPool[stack][0] = me->m_right;
					stackPool[stack][1] = other->m_right;
					stack++;
					dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (dgMeshBVHNode*)));
				}
			}
		}

		bool intersectionFound = false;
		while (bundaryClusters.GetCount()) {
			intersectionFound |= bundaryClusters.ClipCluster (bundaryClusters.GetRoot(), this, &otherMeshBVH);
		}
		ImproveNodeFitness();
		return intersectionFound;
	}

	static void ClipMeshesAndColorize (dgMeshEffect* const meshA, dgMeshEffect* const meshB)
	{
		dgBooleanMeshClipper BVHmeshA (meshA);
		dgBooleanMeshClipper BVHmeshB (meshB);


		BVHmeshA.ClippMesh (BVHmeshB);
BVHmeshA.m_mesh->SaveOFF("xxxA0.off");
		BVHmeshB.ClippMesh (BVHmeshA);
BVHmeshB.m_mesh->SaveOFF("xxxB0.off");
		BVHmeshA.ClippMesh (BVHmeshB);
		dgAssert (!BVHmeshB.ClippMesh (BVHmeshA));
		dgAssert (!BVHmeshA.ClippMesh (BVHmeshB));
		
	}
#endif


	dgBooleanMeshClipper(dgMeshEffect* const mesh)
		:dgMeshBVH(mesh)
//		,m_vertexBase(mesh->GetVertexCount())
//		,m_vertexAlias(mesh->GetVertexCount() + 512, mesh->GetAllocator())
//		,m_nodeEdgeMap(mesh->GetAllocator())
	{
		dgMeshBVH::Build();
	}

	~dgBooleanMeshClipper()
	{
	}


	static void ClipMeshesAndColorize(dgMeshEffect* const meshA, dgMeshEffect* const meshB)
	{
		dgBooleanMeshClipper BVHmeshA(meshA);
		dgBooleanMeshClipper BVHmeshB(meshB);

		dgCurvesNetwork network(&BVHmeshA, &BVHmeshB);

		int stack = 1;
		dgMeshBVHNode* stackPool[2 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];

		stackPool[0][0] = BVHmeshA.m_rootNode;
		stackPool[0][1] = BVHmeshB.m_rootNode;
		while (stack) {
			stack --;
			dgMeshBVHNode* const nodeA = stackPool[stack][0];
			dgMeshBVHNode* const nodeB = stackPool[stack][1];
			if (dgOverlapTest (nodeA->m_p0, nodeA->m_p1, nodeB->m_p0, nodeB->m_p1)) {
				if (nodeA->m_face && nodeB->m_face) {
					network.ClipMeshesFaces(nodeA->m_face, nodeB->m_face);
				} else if (nodeA->m_face) {
					stackPool[stack][0] = nodeA;
					stackPool[stack][1] = nodeB->m_left;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA;
					stackPool[stack][1] = nodeB->m_right;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));
					
				} else if (nodeB->m_face) {
					stackPool[stack][0] = nodeA->m_left;
					stackPool[stack][1] = nodeB;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_right;
					stackPool[stack][1] = nodeB;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

				} else {
					stackPool[stack][0] = nodeA->m_left;
					stackPool[stack][1] = nodeB->m_left;
					stack ++;
					dgAssert (stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_left;
					stackPool[stack][1] = nodeB->m_right;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_right;
					stackPool[stack][1] = nodeB->m_left;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_right;
					stackPool[stack][1] = nodeB->m_right;
					stack++;
					dgAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));
				}
			}
		}

		network.Colorize();
		
/*
		dgInt32 baseAttibuteCountB = BVHmeshB.m_mesh->GetPropertiesCount();

		BVHmeshA.m_mesh->SaveOFF("xxxA0.off");
		BVHmeshB.m_mesh->SaveOFF("xxxB0.off");

		// edge-face, edge-edge and edge-vertex intersections until not more intersections are found 
		for (bool intersectionFound = true; intersectionFound;) {
			intersectionFound = false;

			intersectionFound |= BVHmeshA.CalculateEdgeFacesIntersetions(BVHmeshB);
			intersectionFound |= BVHmeshB.CalculateEdgeFacesIntersetions(BVHmeshA);

			intersectionFound |= BVHmeshA.CalculateVertexFacesIntersetions(BVHmeshB);
			intersectionFound |= BVHmeshB.CalculateVertexFacesIntersetions(BVHmeshA);


			BVHmeshA.m_mesh->SaveOFF("xxxA1.off");
			BVHmeshB.m_mesh->SaveOFF("xxxB1.off");

			intersectionFound |= BVHmeshA.CalculateEdgeEdgeIntersetions(BVHmeshB);

			BVHmeshA.m_mesh->SaveOFF("xxxA2.off");
			BVHmeshB.m_mesh->SaveOFF("xxxB2.off");

			intersectionFound |= BVHmeshA.CalculateEdgeVertexIntersetions(BVHmeshB);
			intersectionFound |= BVHmeshB.CalculateEdgeVertexIntersetions(BVHmeshA);

			BVHmeshA.m_mesh->SaveOFF("xxxA3.off");
			BVHmeshB.m_mesh->SaveOFF("xxxB3.off");
		}
*/		
	}



//	dgInt32 m_vertexBase;
//	dgArray<dgHugeVector> m_vertexAlias;
//	dgArray<dgBigVector> m_vertexAlias;
//	dgTree<dgInt32,dgInt32> m_vertexMap;
//	dgTree<dgMeshBVHNode*, dgUnsigned64> m_nodeEdgeMap;

//	static dgBigVector m_posTol;
//	static dgBigVector m_negTol;
//	static dgBigVector m_tol2;
//	static dgBigVector m_tol3;
//	static dgBigVector m_oneMinusTol;
};

//dgBigVector dgBooleanMeshClipper::m_posTol ( DG_BOOLEAN_ZERO_TOLERANCE);
//dgBigVector dgBooleanMeshClipper::m_negTol (-DG_BOOLEAN_ZERO_TOLERANCE);
//dgBigVector dgBooleanMeshClipper::m_tol2 (m_posTol * m_posTol);
//dgBigVector dgBooleanMeshClipper::m_tol3 (m_posTol * m_tol2);
//dgBigVector dgBooleanMeshClipper::m_oneMinusTol (dgGoogol(1.0) - m_posTol);


dgMeshEffect* dgMeshEffect::Intersection (const dgMatrix& matrix, const dgMeshEffect* const clipperMesh) const
{
	dgMeshEffect copy (*this);
	dgMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);
/*
	dgMeshEffect* const mesh = new (GetAllocator()) dgMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFaces (mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFaces (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(NULL);

	return mesh;
*/

	dgAssert (0);
	return NULL;
}


