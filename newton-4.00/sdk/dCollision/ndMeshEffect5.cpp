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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndMeshEffect.h"
//#include "dgBody.h"
//#include "dgWorld.h"
//#include "ndMeshEffect.h"
//#include "dgCollisionConvexHull.h"


#if 0
void ndMeshEffect::ClipMesh (const dMatrix& matrix, const ndMeshEffect* const clipMesh, ndMeshEffect** const back, ndMeshEffect** const front) const
{
	dAssert (0);
/*
	ndMeshEffect clipper (*clipMesh);
	clipper.TransformMesh (matrix);

	ndMeshEffect* backMeshSource = nullptr;
	ndMeshEffect* frontMeshSource = nullptr;
	ndMeshEffect* backMeshClipper = nullptr;
	ndMeshEffect* frontMeshClipper = nullptr;

	ClipMesh (&clipper, &backMeshSource, &frontMeshSource);
	if (backMeshSource && frontMeshSource) {
		clipper.ClipMesh (this, &backMeshClipper, &frontMeshClipper);
		if (backMeshSource && frontMeshSource) {

			ndMeshEffect* backMesh;
			ndMeshEffect* frontMesh;

			backMesh = new (GetAllocator()) ndMeshEffect (GetAllocator(), true);
			frontMesh = new (GetAllocator()) ndMeshEffect (GetAllocator(), true);

			backMesh->BeginPolygon();
			frontMesh->BeginPolygon();

			backMesh->MergeFaces(backMeshSource);
			backMesh->MergeFaces(backMeshClipper);

			frontMesh->MergeFaces(frontMeshSource);
			frontMesh->ReverseMergeFaces(backMeshClipper);

			backMesh->EndPolygon(dFloat64 (1.0e-5f));
			frontMesh->EndPolygon(dFloat64 (1.0e-5f));

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




ndMeshEffect* ndMeshEffect::Union (const dMatrix& matrix, const ndMeshEffect* const clipperMesh) const
{
	dAssert (0);
	return nullptr;
/*
	ndMeshEffect copy (*this);
	ndMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);

	ndMeshEffect* const mesh = new (GetAllocator()) ndMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddExteriorFaces (mesh, &copy);

	dgBooleanMeshClipper::AddExteriorFaces (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(nullptr);
	return mesh;
*/
}

ndMeshEffect* ndMeshEffect::Difference (const dMatrix& matrix, const ndMeshEffect* const clipperMesh) const
{
/*
	ndMeshEffect copy (*this);
	ndMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);

	ndMeshEffect* const mesh = new (GetAllocator()) ndMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddExteriorFaces (mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFacesInvertWinding (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(nullptr);
	return mesh;
*/

	dAssert (0);
	return nullptr;
}


class dgBooleanMeshClipper: public ndMeshEffect::dMeshBVH
{
	class dgPoint: public dBigVector
	{
		public:
		dgPoint()
			:dBigVector(dFloat32 (0.0f))
			,m_links(nullptr)
			,m_lru(0)
		{
			dAssert (0);
		}

		dgPoint(const dBigVector& point, dgMemoryAllocator* const allocator)
			:dBigVector(point)
			,m_links(allocator)
			,m_lru(0)
		{
		}
		
		dList<dTree<dgPoint, dFloat64>::dTreeNode*> m_links;
		dInt32 m_lru;
	};

	class dgCurvesNetwork: public dTree<dgPoint, dFloat64>
	{
		public:
		dgCurvesNetwork ()
			:dTree<dgPoint, dFloat64>(nullptr)
		{
			dAssert (0);
		}

		dgCurvesNetwork (dgMemoryAllocator* const allocator)
			:dTree<dgPoint, dFloat64>(allocator)
		{
		}

		dTreeNode* AddVertex(const dBigVector& point, dgMemoryAllocator* const allocator)
		{
			dFloat64 key = ((point.m_z * dFloat64 (1024.0f) + point.m_y) * dFloat64 (1024.0f)) + point.m_x;
			dTreeNode* node = Find(key);
			if (!node) {
				dgPoint entry (point, allocator);
				node = Insert(entry, key);
			}
			return node;
		}

/*
		dgCurvesNetwork(dgBooleanMeshClipper* const BVHmeshA, dgBooleanMeshClipper* const BVHmeshB)
			:dTree<dgPoint, dFloat64>(BVHmeshA->m_mesh->GetAllocator())
//			,m_meshA(BVHmeshA->m_mesh)
//			,m_meshB(BVHmeshB->m_mesh)
//			,m_pointBaseA(m_meshA->GetVertexCount()) 
//			,m_pointBaseB(m_meshB->GetVertexCount()) 
//			,m_lru(0)
		{
		}
*/
/*
		dgHugeVector CalculateFaceNormal (const ndMeshEffect* const mesh, dEdge* const face)
		{
			dgHugeVector plane(dgGoogol::m_zero, dgGoogol::m_zero, dgGoogol::m_zero, dgGoogol::m_zero);
			dEdge* edge = face;
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


		bool IsPointInFace (const dgHugeVector& point, const ndMeshEffect* const mesh, dEdge* const face, const dgHugeVector& normal) const 
		{
			dEdge* edge = face;

			dTrace (("%f %f %f\n", dFloat64 (point.m_x), dFloat64 (point.m_y), dFloat64 (point.m_z)));
			do {
				dBigVector p1(mesh->GetVertex(edge->m_incidentVertex));
				dTrace (("%f %f %f\n", dFloat64 (p1.m_x), dFloat64 (p1.m_y), dFloat64 (p1.m_z)));
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

		dFloat64 ClipEdgeFace(const ndMeshEffect* const meshEdge, dEdge* const edge, const ndMeshEffect* const meshFace, dEdge* const face, const dgHugeVector& plane)
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
				dgHugeVector p (p0 + p1p0.Scale (param));
				if (IsPointInFace (p, meshFace, face, plane)) {
					return param;
				}
				return -1.0f;
			} else {
				dAssert (0);
				//special cases;
			}
			
			return -1.0f;
		}

		void AddPoint (ndMeshEffect* const edgeOwnerMesh, dEdge* const edgeStart, ndMeshEffect* const faceOwnerMesh, dEdge* const face, const dgHugeVector& plane, dTreeNode** nodes, dInt32& index)
		{
			dEdge* edge = edgeStart;
			do {
				dFloat64 param = ClipEdgeFace(edgeOwnerMesh, edge, faceOwnerMesh, face, plane);
				if (param > 0.0f) {
					dgPoint point(edgeOwnerMesh, edge, param, faceOwnerMesh, face);
					dTreeNode* node = Find(dgNodeKey(edge, param));
					if (!node) {
						node = Insert(point, dgNodeKey(edge, param));
					}
					nodes[index] = node;
					index ++;
				}
				edge = edge->m_next;
			} while (edge != edgeStart);

		}

		void ClipMeshesFaces(dEdge* const faceA, dEdge* const faceB)
		{
			dAssert (m_meshA->FindEdge(faceA->m_incidentVertex, faceA->m_twin->m_incidentVertex) == faceA);
			dAssert (m_meshB->FindEdge(faceB->m_incidentVertex, faceB->m_twin->m_incidentVertex) == faceB);

			dgHugeVector planeA (CalculateFaceNormal (m_meshA, faceA));
			dgHugeVector planeB (CalculateFaceNormal (m_meshB, faceB));

			dInt32 index = 0;
			dTreeNode* nodes[16];
			AddPoint (m_meshA, faceA, m_meshB, faceB, planeB, nodes, index);
			AddPoint (m_meshB, faceB, m_meshA, faceA, planeA, nodes, index);
			dAssert ((index == 0) || (index == 2));
			if (index == 2) {
				dgPoint& pointA = nodes[0]->GetInfo();
				dgPoint& pointB = nodes[1]->GetInfo();
				pointA.m_links.Append(nodes[1]);
				pointB.m_links.Append(nodes[0]);
			}
		}

		void GetCurve (dList<dTreeNode*>& curve, dTreeNode* const node)
		{
			dInt32 stack = 1;
			dTreeNode* pool[64];

			pool[0] = node;
			while (stack) {
				stack --;
				dTreeNode* const ptr = pool[stack];
				dgPoint& point = ptr->GetInfo();
				if (point.m_lru != m_lru) {
					point.m_lru = m_lru;
					curve.Append(ptr);
					for (dList<dTree<dgPoint, dgNodeKey>::dTreeNode*>::dListNode* ptrPoint = point.m_links.GetFirst(); ptrPoint; ptrPoint = ptrPoint->GetNext()) {
						dTreeNode* const nextnode = ptrPoint->GetInfo();
						dgPoint& nextPoint = nextnode->GetInfo();
						if (nextPoint.m_lru != m_lru) {
							pool[stack] = nextnode;
							stack ++;
						}
					}
				}
			}
		}

		void EmbedCurveToSingleFace (dList<dTreeNode*>& curve, ndMeshEffect* const mesh)
		{
			dEdge* const face = curve.GetFirst()->GetInfo()->GetInfo().m_face;

			dInt32 indexBase = mesh->GetVertexCount();
			dInt32 indexAttribBase = mesh->GetPropertiesCount();

			for (dList<dTreeNode*>::dListNode* node = curve.GetFirst(); node; node = node->GetNext()) {
				dgPoint& point = node->GetInfo()->GetInfo();
				dAssert (point.m_face == face);
				ndMeshEffect::dgVertexAtribute attribute(mesh->InterpolateVertex(point.m_posit, face));
				mesh->AddVertex(point.m_posit);
				mesh->AddAtribute(attribute);
			}

			dList<dEdge*> list(GetAllocator());
			dInt32 i0 = curve.GetCount() - 1;
			for (dInt32 i = 0; i < curve.GetCount(); i++) {
				dEdge* const edge = mesh->AddHalfEdge(indexBase + i0, indexBase + i);
				dEdge* const twin = mesh->AddHalfEdge(indexBase + i, indexBase + i0);

				edge->m_incidentFace = 1;
				twin->m_incidentFace = 1;
				edge->m_userData = indexAttribBase + i0;
				twin->m_userData = indexAttribBase + i;
				twin->m_twin = edge;
				edge->m_twin = twin;
				i0 = i;
				list.Append(edge);
			}

			dEdge* closestEdge = nullptr;
			dFloat64 dist2 = dFloat64 (1.0e10f);
			dBigVector p(mesh->GetVertex(face->m_incidentVertex));

			list.Append(list.GetFirst()->GetInfo());
			list.Addtop(list.GetLast()->GetInfo());
			for (dList<dEdge*>::dListNode* node = list.GetFirst()->GetNext(); node != list.GetLast(); node = node->GetNext()) {
				dEdge* const edge = node->GetInfo();

				dEdge* const prev = node->GetPrev()->GetInfo();
				edge->m_prev = prev;
				prev->m_next = edge;
				edge->m_twin->m_next = prev->m_twin;
				prev->m_twin->m_prev = edge->m_twin;

				dEdge* const next = node->GetNext()->GetInfo();
				edge->m_next = next;
				next->m_prev = edge;
				edge->m_twin->m_prev = next->m_twin;
				next->m_twin->m_next = edge->m_twin;

				dBigVector dist(mesh->GetVertex(edge->m_incidentVertex) - p);
				dFloat64 err2 = dist % dist;
				if (err2 < dist2) {
					closestEdge = edge;
					dist2 = err2;
				}
			}

			dBigVector faceNormal (mesh->FaceNormal(face, mesh->GetVertexPool(), mesh->GetVertexStrideInByte()));
			dBigVector clipNormal (mesh->FaceNormal(closestEdge, mesh->GetVertexPool(), mesh->GetVertexStrideInByte()));
			if ((clipNormal % faceNormal) > dFloat64(0.0f)) {
				closestEdge = closestEdge->m_twin->m_next;
			}
			dEdge* const glueEdge = mesh->ConnectVertex (closestEdge, face);
			dAssert (glueEdge);
			mesh->PolygonizeFace(glueEdge, mesh->GetVertexPool(), sizeof (dBigVector));
		}

		void EmbedCurveToMulipleFaces (dList<dTreeNode*>& curve, ndMeshEffect* const mesh)
		{
			for (dList<dTreeNode*>::dListNode* node = curve.GetFirst(); node; node = node->GetNext()) {
				dgPoint& point = node->GetInfo()->GetInfo();
				if (point.m_edgeOwnerMesh == mesh) {
					dEdge* const edge = point.m_edge;
					dBigVector p0 (mesh->GetVertex(edge->m_incidentVertex));
					dBigVector p1 (mesh->GetVertex(edge->m_twin->m_incidentVertex));
					dVector p1p0 (p1 - p0);
					dVector qp0 (point.m_posit - p0);
					dFloat64 param = (qp0 % p1p0) / (p1p0 % p1p0);
					dAssert (param >= dFloat64 (0.0f));
					dAssert (param <= dFloat64 (1.0f));
					dEdge* const newEdge = mesh->InsertEdgeVertex (edge, param);
				}
//				mesh->AddVertex(point.m_posit);
//				mesh->AddAtribute(attribute);
			}
		}


		void AddCurveToMesh (dList<dTreeNode*>& curve, ndMeshEffect* const mesh)
		{
			bool isIscribedInFace = true; 
			dEdge* const face = curve.GetFirst()->GetInfo()->GetInfo().m_face;
			for (dList<dTreeNode*>::dListNode* node = curve.GetFirst(); isIscribedInFace && node; node = node->GetNext()) {
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
					dList<dTreeNode*> curve (GetAllocator());
					GetCurve (curve, iter.GetNode());
					AddCurveToMesh (curve, m_meshB);
					AddCurveToMesh (curve, m_meshA);
				}
			}

			m_meshA->SaveOFF("xxxA0.off");
			m_meshB->SaveOFF("xxxB0.off");
		}

		ndMeshEffect* m_meshA;
		ndMeshEffect* m_meshB;
		dInt32 m_pointBaseA;
		dInt32 m_pointBaseB;
		dInt32 m_lru;
*/
	};

	class dgClippedFace: public ndMeshEffect
	{
		public:
		dgClippedFace ()
			:ndMeshEffect()
			,m_curveNetwork()
		{
			dAssert (0);
		}

		dgClippedFace (dgMemoryAllocator* const allocator)
			:ndMeshEffect(allocator)
			,m_curveNetwork(allocator)
		{
		}

		dgClippedFace (const dgClippedFace& copy)
			:ndMeshEffect(copy)
			,m_curveNetwork(copy.m_curveNetwork)
		{
		}

		void InitFace(ndMeshEffect* const mesh, dEdge* const face)
		{
			dInt32 indexCount = 0;
			dInt32 faceIndex[256];
			dInt64 faceDataIndex[256];
			BeginFace ();
			dEdge* ptr = face;
			do {
				dAssert (0);
				//const ndMeshEffect::dgVertexAtribute& point =  mesh->GetAttribute(dInt32 (ptr->m_userData));
				//AddPoint (&point.m_vertex.m_x, dInt32 (point.m_material));
				faceIndex[indexCount] = indexCount;
				faceDataIndex[indexCount] = indexCount;
				indexCount ++;
				ptr = ptr->m_next;
			} while (ptr != face);
			AddFace (indexCount, faceIndex, faceDataIndex);
			EndFace ();
		}


		void AddSegment (const dBigVector& plane, const dBigVector* const segment)
		{
			dAssert (0);
/*
			dgCurvesNetwork::dTreeNode* const node0 = m_curveNetwork.AddVertex (segment[0], GetAllocator());
			dgCurvesNetwork::dTreeNode* const node1 = m_curveNetwork.AddVertex (segment[1], GetAllocator());

			dgPoint& pointA = node0->GetInfo();
			dgPoint& pointB = node1->GetInfo();
			pointA.m_links.Append(node1);
			pointB.m_links.Append(node0);
*/
		}

		dgCurvesNetwork m_curveNetwork;
	};

	class dgClipppedFaces: public dTree<dgClippedFace, dEdge*>
	{
		public:
		dgClipppedFaces(ndMeshEffect* const mesh)
			:dTree<dgClippedFace, dEdge*>(mesh->GetAllocator())
			,m_parentMesh (mesh)
		{
		}

		void ClipMeshesFaces(dEdge* const faceA, const ndMeshEffect* const meshB, dEdge* const faceB, const dBigVector& planeB, const dBigVector* const segment)
		{
			dTreeNode* node = Find (faceA);
			if (!node) {
				dgClippedFace tmp (m_parentMesh->GetAllocator());
				node = Insert (tmp, faceA);
				dgClippedFace& faceHead = node->GetInfo();
				faceHead.InitFace (m_parentMesh, faceA);
			}
			dAssert (node);
			dgClippedFace& faceHead = node->GetInfo();
			faceHead.AddSegment(planeB, segment);
		}

		ndMeshEffect* m_parentMesh;
	};
	

	public:
	dgBooleanMeshClipper(ndMeshEffect* const mesh)
		:dMeshBVH(mesh)
		,m_clippedFaces(mesh)
	{
		dMeshBVH::Build();
	}

	~dgBooleanMeshClipper()
	{
	}

/*
	dFloat64 IntersetionSegment(const ndMeshEffect* const meshEdge, dEdge* const edge, const ndMeshEffect* const meshFace, dEdge* const face, const dgHugeVector& plane)
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
			dgHugeVector p (p0 + p1p0.Scale (param));
			if (IsPointInFace (p, meshFace, face, plane)) {
				return param;
			}
			return -1.0f;
		} else {
			dAssert (0);
			//special cases;
		}
			
		return -1.0f;
	}
*/

	static dgHugeVector CalculateFaceNormal (const ndMeshEffect* const mesh, dEdge* const face)
	{
		dgHugeVector plane(dgGoogol::m_zero, dgGoogol::m_zero, dgGoogol::m_zero, dgGoogol::m_zero);
		dEdge* edge = face;
		dgHugeVector p0(mesh->GetVertex(edge->m_incidentVertex));
		edge = edge->m_next;
		dgHugeVector p1(mesh->GetVertex(edge->m_incidentVertex));
		dgHugeVector p1p0(p1 - p0);
		edge = edge->m_next;
		do {
			dgHugeVector p2(mesh->GetVertex(edge->m_incidentVertex));
			dgHugeVector p2p0(p2 - p0);
			plane += p1p0.CrossProduct(p2p0);
			p1p0 = p2p0;
			edge = edge->m_next;
		} while (edge != face);

		dAssert(plane.m_w == dgGoogol(0.0));
		plane.m_w = dgGoogol::m_zero - plane.DotProduct(p0).GetScalar();
		return plane;
	}

	static bool IsPointInFace (const dgHugeVector& point, const ndMeshEffect* const mesh, dEdge* const face, const dgHugeVector& normal)
	{
		dEdge* edge = face;
		dgHugeVector p0(mesh->GetVertex(face->m_incidentVertex));
		do {
			dgHugeVector p1(mesh->GetVertex(edge->m_twin->m_incidentVertex));
			dgHugeVector p1p0(p1 - p0);
			dgHugeVector q1p0(point - p0);
			dAssert(p1p0.m_w == dgGoogol(0.0));
			dgGoogol side (q1p0.DotProduct(p1p0.CrossProduct(normal)).GetScalar());
			if (side >= dgGoogol::m_zero) {
				return false;
			}
			p0 = p1;
			edge = edge->m_next;
		} while (edge != face);

		return true;
	}

	static bool ClipEdgeFace(dBigVector& point, const ndMeshEffect* const meshEdge, dEdge* const edgeSrc, const ndMeshEffect* const meshFace, dEdge* const face, const dgHugeVector& plane)
	{
		const dEdge* const edge = (edgeSrc->m_incidentVertex < edgeSrc->m_twin->m_incidentVertex) ? edgeSrc : edgeSrc->m_twin;
		dgHugeVector p0 (meshEdge->GetVertex(edge->m_incidentVertex));
		dgHugeVector p1 (meshEdge->GetVertex(edge->m_twin->m_incidentVertex));
			
		dgGoogol test0 (plane.EvaluePlane(p0));
		dgGoogol test1 (plane.EvaluePlane(p1));

		if ((test0 * test1) > dgGoogol::m_zero) {
			// both point are in one side
			return false;
		}

		if ((test0 * test1) < dgGoogol::m_zero) {
			//point on different size, clip the line
			dgHugeVector p1p0 (p1 - p0);
			dAssert(p1p0.m_w == dgGoogol(0.0));
			dgGoogol param = dgGoogol::m_zero - plane.EvaluePlane(p0) / plane.DotProduct(p1p0).GetScalar();
			dgHugeVector p (p0 + p1p0.Scale (param));
			if (IsPointInFace (p, meshFace, face, plane)) {
				point = dBigVector(p.m_x, p.m_y, p.m_z, p.m_w);
				return true;
			}
			return false;
		} else {
			dAssert (0);
			//special cases;
		}
			
		return false;
	}

	static void CalculateIntersection (const ndMeshEffect* const edgeOwnerMesh, dEdge* const edgeStart, const ndMeshEffect* const faceOwnerMesh, dEdge* const face, const dgHugeVector& facePlane, dBigVector* const data, dInt32& index)
	{
		dEdge* edge = edgeStart;
		do {
			bool isCleipped = ClipEdgeFace(data[index], edgeOwnerMesh, edge, faceOwnerMesh, face, facePlane);
			if (isCleipped) {
				index ++;
			}
			edge = edge->m_next;
		} while (edge != edgeStart);
	}

	static void ClipMeshesFaces(dgBooleanMeshClipper& bvhMeshA, dEdge* const faceA, dgBooleanMeshClipper& bvhMeshB, dEdge* const faceB)
	{
		const ndMeshEffect* const meshA = bvhMeshA.m_mesh;
		const ndMeshEffect* const meshB = bvhMeshB.m_mesh;
		dAssert (meshA->FindEdge(faceA->m_incidentVertex, faceA->m_twin->m_incidentVertex) == faceA);
		dAssert (meshB->FindEdge(faceB->m_incidentVertex, faceB->m_twin->m_incidentVertex) == faceB);

		dgHugeVector planeA (CalculateFaceNormal (meshA, faceA));
		dgHugeVector planeB (CalculateFaceNormal (meshB, faceB));

		dBigVector points[16];
		dInt32 pointCount = 0;
		CalculateIntersection (meshA, faceA, meshB, faceB, planeB, points, pointCount);
		CalculateIntersection (meshB, faceB, meshA, faceA, planeA, points, pointCount);
		dAssert ((pointCount == 0) || (pointCount == 2));
		if (pointCount == 2) {
			dBigVector facePlaneA (planeA.m_x, planeA.m_y, planeA.m_z, planeA.m_w);
			dBigVector facePlaneB (planeB.m_x, planeB.m_y, planeB.m_z, planeB.m_w);

			bvhMeshA.m_clippedFaces.ClipMeshesFaces(faceA, meshB, faceB, facePlaneB, points);
			bvhMeshB.m_clippedFaces.ClipMeshesFaces(faceB, meshA, faceA, facePlaneA, points);
		}
	}

	static void ClipMeshesAndColorize(ndMeshEffect* const meshA, ndMeshEffect* const meshB)
	{
		dAssert (0);
/*
		dgBooleanMeshClipper BVHmeshA(meshA);
		dgBooleanMeshClipper BVHmeshB(meshB);

		dInt32 stack = 1;
		
		dgMeshBVHNode* stackPool[2 * DG_MESH_EFFECT_BVH_STACK_DEPTH][2];

		stackPool[0][0] = BVHmeshA.m_rootNode;
		stackPool[0][1] = BVHmeshB.m_rootNode;
		while (stack) {
			stack --;
			dgMeshBVHNode* const nodeA = stackPool[stack][0];
			dgMeshBVHNode* const nodeB = stackPool[stack][1];
			if (dgOverlapTest (nodeA->m_p0, nodeA->m_p1, nodeB->m_p0, nodeB->m_p1)) {
				if (nodeA->m_face && nodeB->m_face) {
					ClipMeshesFaces(BVHmeshA, nodeA->m_face, BVHmeshB, nodeB->m_face);
				} else if (nodeA->m_face) {
					stackPool[stack][0] = nodeA;
					stackPool[stack][1] = nodeB->m_left;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA;
					stackPool[stack][1] = nodeB->m_right;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));
					
				} else if (nodeB->m_face) {
					stackPool[stack][0] = nodeA->m_left;
					stackPool[stack][1] = nodeB;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_right;
					stackPool[stack][1] = nodeB;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

				} else {
					stackPool[stack][0] = nodeA->m_left;
					stackPool[stack][1] = nodeB->m_left;
					stack ++;
					dAssert (stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_left;
					stackPool[stack][1] = nodeB->m_right;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_right;
					stackPool[stack][1] = nodeB->m_left;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));

					stackPool[stack][0] = nodeA->m_right;
					stackPool[stack][1] = nodeB->m_right;
					stack++;
					dAssert(stack < sizeof (stackPool) / sizeof (stackPool[0]));
				}
			}
		}
*/
		dAssert (0);
//		network.Colorize();
	

/*
		dInt32 baseAttibuteCountB = BVHmeshB.m_mesh->GetPropertiesCount();

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

	dgClipppedFaces m_clippedFaces;
};



ndMeshEffect* ndMeshEffect::Intersection (const dMatrix& matrix, const ndMeshEffect* const clipperMesh) const
{
	ndMeshEffect copy (*this);
	ndMeshEffect clipper (*clipperMesh);
	clipper.TransformMesh (matrix);

	dgBooleanMeshClipper::ClipMeshesAndColorize (&copy, &clipper);
/*
	ndMeshEffect* const mesh = new (GetAllocator()) ndMeshEffect (GetAllocator());
	mesh->BeginFace();
	dgBooleanMeshClipper::CopyPoints(mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFaces (mesh, &copy);
	dgBooleanMeshClipper::AddInteriorFaces (mesh, &clipper);
	mesh->EndFace ();
	mesh->RepairTJoints();
	mesh->RemoveUnusedVertices(nullptr);

	return mesh;
*/

	dAssert (0);
	return nullptr;
}

#endif

// return 0 if the the shape was clipped
// return 1 if the shape is the positive size of the plane
// return -1 if the shape is the negative size of the plane
// return -2 if function fail
dInt32 ndMeshEffect::PlaneClip(const ndMeshEffect& convexMesh, const dEdge* const convexFace)
{
	dAssert(convexFace->m_incidentFace > 0);

	dBigVector normal(convexMesh.FaceNormal(convexFace, &convexMesh.m_points.m_vertex[0].m_x, sizeof(dBigVector)));
	dFloat64 mag2 = normal.DotProduct(normal).GetScalar();
	if (mag2 < dFloat64(1.0e-30))
	{
		dAssert(0);
		return -2;
	}

	normal = normal.Normalize();
	dBigVector origin(convexMesh.m_points.m_vertex[convexFace->m_incidentVertex]);
	dBigPlane plane(normal, -origin.DotProduct(normal).GetScalar());

	dAssert(!HasOpenEdges());

	dInt32 pointCount = GetVertexCount();
	dStack <dFloat64> testPool(2 * pointCount + 1024);
	dFloat64* const test = &testPool[0];
	for (dInt32 i = 0; i < pointCount; i++)
	{
		test[i] = plane.Evalue(m_points.m_vertex[i]);
		if (fabs(test[i]) < dFloat32(1.0e-5f))
		{
			test[i] = dFloat32(0.0f);
		}
	}

	dInt32 positive = 0;
	dInt32 negative = 0;
	dPolyhedra::Iterator iter(*this);
	for (iter.Begin(); iter && !(positive && negative); iter++)
	{
		dEdge* const edge = &(*iter);
		positive += test[edge->m_incidentVertex] > dFloat32(0.0f);
		negative += test[edge->m_incidentVertex] < dFloat32(0.0f);
	}
	if (positive && !negative)
	{
		return 1;
	}

	dInt32 retValue = -1;
	if (positive && negative)
	{
		retValue = 0;
		const dEdge* e0 = convexFace;
		const dEdge* e1 = e0->m_next;
		const dEdge* e2 = e1->m_next;

		dMatrix matrix;
		dBigVector p1(convexMesh.m_points.m_vertex[e1->m_incidentVertex]);

		dBigVector xDir(p1 - origin);
		dAssert(xDir.m_w == dFloat32(0.0f));
		dAssert(xDir.DotProduct(xDir).GetScalar() > dFloat32(0.0f));
		matrix[2] = dVector(normal);
		matrix[0] = dVector(xDir.Scale(dFloat64(1.0f) / sqrt(xDir.DotProduct(xDir).GetScalar())));
		matrix[1] = matrix[2].CrossProduct(matrix[0]);
		matrix[3] = dVector(origin);
		matrix[3][3] = dFloat32(1.0f);

		dVector q0(matrix.UntransformVector(dVector(convexMesh.m_points.m_vertex[e0->m_incidentVertex])));
		dVector q1(matrix.UntransformVector(dVector(convexMesh.m_points.m_vertex[e1->m_incidentVertex])));
		dVector q2(matrix.UntransformVector(dVector(convexMesh.m_points.m_vertex[e2->m_incidentVertex])));

		dVector p10(q1 - q0);
		dVector p20(q2 - q0);
		dVector faceNormal(matrix.UnrotateVector(dVector(normal)));
		dAssert(faceNormal.m_w == dFloat32(0.0f));
		dFloat32 areaInv = faceNormal.DotProduct(p10.CrossProduct(p20)).GetScalar();
		if (e2->m_next != e0)
		{
			const dEdge* edge = e2;
			dVector r1(q2);
			dVector q10(p20);
			do
			{
				dVector r2(matrix.UntransformVector(dVector(convexMesh.m_points.m_vertex[edge->m_next->m_incidentVertex])));
				dVector q20(r2 - q0);
				dFloat32 areaInv1 = faceNormal.DotProduct(q10.CrossProduct(q20)).GetScalar();
				if (areaInv1 > areaInv)
				{
					e1 = edge;
					e2 = edge->m_next;
					q1 = r1;
					q2 = r2;
					areaInv = areaInv1;
				}
				r1 = r2;
				q10 = q20;
				edge = edge->m_next;
			} while (edge->m_next != e0);
		}

		dAssert(areaInv > dFloat32(0.0f));
		areaInv = dFloat32(1.0f) / areaInv;

		dVector uv0[3];
		dVector uv1[3];
		memset(uv0, 0, sizeof(uv0));
		memset(uv1, 0, sizeof(uv1));
		if (m_attrib.m_uv0Channel.GetCount() && convexMesh.m_attrib.m_uv0Channel.GetCount())
		{
			uv0[0] = dVector(dFloat32(convexMesh.m_attrib.m_uv0Channel[dInt32(e0->m_userData)].m_u), dFloat32(convexMesh.m_attrib.m_uv0Channel[dInt32(e0->m_userData)].m_v), dFloat32(0.0f), dFloat32(0.0f));
			uv0[1] = dVector(dFloat32(convexMesh.m_attrib.m_uv0Channel[dInt32(e1->m_userData)].m_u), dFloat32(convexMesh.m_attrib.m_uv0Channel[dInt32(e1->m_userData)].m_v), dFloat32(0.0f), dFloat32(0.0f));
			uv0[2] = dVector(dFloat32(convexMesh.m_attrib.m_uv0Channel[dInt32(e2->m_userData)].m_u), dFloat32(convexMesh.m_attrib.m_uv0Channel[dInt32(e2->m_userData)].m_v), dFloat32(0.0f), dFloat32(0.0f));
		}

		if (m_attrib.m_uv1Channel.GetCount() && convexMesh.m_attrib.m_uv1Channel.GetCount())
		{
			uv1[0] = dVector(dFloat32(convexMesh.m_attrib.m_uv1Channel[dInt32(e0->m_userData)].m_u), dFloat32(convexMesh.m_attrib.m_uv1Channel[dInt32(e0->m_userData)].m_v), dFloat32(0.0f), dFloat32(0.0f));
			uv1[1] = dVector(dFloat32(convexMesh.m_attrib.m_uv1Channel[dInt32(e1->m_userData)].m_u), dFloat32(convexMesh.m_attrib.m_uv1Channel[dInt32(e1->m_userData)].m_v), dFloat32(0.0f), dFloat32(0.0f));
			uv1[2] = dVector(dFloat32(convexMesh.m_attrib.m_uv1Channel[dInt32(e2->m_userData)].m_u), dFloat32(convexMesh.m_attrib.m_uv1Channel[dInt32(e2->m_userData)].m_v), dFloat32(0.0f), dFloat32(0.0f));
		}

		for (iter.Begin(); iter; iter++)
		{
			dEdge* const edge = &(*iter);

			dFloat64 side0 = test[edge->m_prev->m_incidentVertex];
			dFloat64 side1 = test[edge->m_incidentVertex];

			if ((side0 < dFloat32(0.0f)) && (side1 > dFloat64(0.0f)))
			{
				dBigVector dp(m_points.m_vertex[edge->m_incidentVertex] - m_points.m_vertex[edge->m_prev->m_incidentVertex]);
				dAssert(dp.m_w == dFloat32(0.0f));
				dFloat64 param = -side0 / plane.DotProduct(dp).GetScalar();

				dEdge* const splitEdge = InsertEdgeVertex(edge->m_prev, param);
				test[splitEdge->m_next->m_incidentVertex] = dFloat64(0.0f);
			}
		}

		dInt32 colorMark = IncLRU();
		for (iter.Begin(); iter; iter++)
		{
			dEdge* const edge = &(*iter);
			dFloat64 side0 = test[edge->m_incidentVertex];
			dFloat64 side1 = test[edge->m_next->m_incidentVertex];

			if ((side0 > dFloat32(0.0f)) || (side1 > dFloat64(0.0f)))
			{
				edge->m_mark = colorMark;
			}
		}

		for (iter.Begin(); iter; iter++)
		{
			dEdge* const edge = &(*iter);
			dFloat64 side0 = test[edge->m_incidentVertex];
			dFloat64 side1 = test[edge->m_next->m_incidentVertex];
			if ((side0 == dFloat32(0.0f)) && (side1 == dFloat64(0.0f)))
			{
				dEdge* ptr = edge->m_next;
				do
				{
					if (ptr->m_mark == colorMark)
					{
						edge->m_mark = colorMark;
						break;
					}
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}

		for (iter.Begin(); iter; iter++)
		{
			dEdge* const edge = &(*iter);
			if ((edge->m_mark == colorMark) && (edge->m_next->m_mark < colorMark))
			{
				dEdge* const startEdge = edge->m_next;
				dEdge* end = startEdge;
				do
				{
					if (end->m_mark == colorMark)
					{
						break;
					}

					end = end->m_next;
				} while (end != startEdge);
				dAssert(end != startEdge);
				dEdge* const devideEdge = ConnectVertex(startEdge, end);
				dAssert(devideEdge);
				dAssert(devideEdge->m_next->m_mark != colorMark);
				dAssert(devideEdge->m_prev->m_mark != colorMark);
				dAssert(devideEdge->m_twin->m_next->m_mark == colorMark);
				dAssert(devideEdge->m_twin->m_prev->m_mark == colorMark);
				devideEdge->m_mark = colorMark - 1;
				devideEdge->m_twin->m_mark = colorMark;
			}
		}

		dInt32 mark = IncLRU();
		dList<dEdge*> faceList;
		for (iter.Begin(); iter; iter++)
		{
			dEdge* const face = &(*iter);
			if ((face->m_mark >= colorMark) && (face->m_mark != mark))
			{
				faceList.Append(face);
				dEdge* edge = face;
				do
				{
					edge->m_mark = mark;
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		for (dList<dEdge*>::dListNode* node = faceList.GetFirst(); node; node = node->GetNext())
		{
			dEdge* const face = node->GetInfo();
			DeleteFace(face);
		}

		mark = IncLRU();
		faceList.RemoveAll();
		for (iter.Begin(); iter; iter++)
		{
			dEdge* const face = &(*iter);
			if ((face->m_mark != mark) && (face->m_incidentFace < 0))
			{
				faceList.Append(face);
				dEdge* edge = face;
				do
				{
					edge->m_mark = mark;
					edge = edge->m_next;
				} while (edge != face);
			}
		}

		const dInt32 capAttribute = convexMesh.m_attrib.m_materialChannel.GetCount() ? convexMesh.m_attrib.m_materialChannel[dInt32(convexFace->m_userData)] : 0;
		for (dList<dEdge*>::dListNode* node = faceList.GetFirst(); node; node = node->GetNext())
		{
			dEdge* const face = node->GetInfo();

			dEdge* edge = face;
			do
			{
				edge->m_incidentFace = 1;
				edge->m_userData = m_attrib.m_pointChannel.GetCount();

				m_attrib.m_pointChannel.PushBack(edge->m_incidentVertex);
				if (m_attrib.m_normalChannel.GetCount())
				{
					dTriplex n;
					n.m_x = dFloat32(normal.m_x);
					n.m_y = dFloat32(normal.m_y);
					n.m_z = dFloat32(normal.m_z);
					m_attrib.m_normalChannel.PushBack(n);
				}

				if (m_attrib.m_binormalChannel.GetCount())
				{
					dAssert(0);
				}

				if (m_attrib.m_colorChannel.GetCount())
				{
					dAssert(0);
				}

				if (m_attrib.m_materialChannel.GetCount())
				{
					m_attrib.m_materialChannel.PushBack(capAttribute);
				}

				//dVector p (matrix.UntransformVector (attibute.m_vertex));
				dVector p(matrix.UntransformVector(m_points.m_vertex[edge->m_incidentVertex]));
				dVector p_p0(p - q0);
				dVector p_p1(p - q1);
				dVector p_p2(p - q2);
				dAssert(faceNormal.m_w == dFloat32(0.0f));
				dFloat32 alpha0 = faceNormal.DotProduct(p_p1.CrossProduct(p_p2)).GetScalar() * areaInv;
				dFloat32 alpha1 = faceNormal.DotProduct(p_p2.CrossProduct(p_p0)).GetScalar() * areaInv;
				dFloat32 alpha2 = faceNormal.DotProduct(p_p0.CrossProduct(p_p1)).GetScalar() * areaInv;

				//alpha0 = 0.0f;
				//alpha1 = 0.0f;
				//alpha2 = 0.0;
				if (m_attrib.m_uv0Channel.GetCount() && convexMesh.m_attrib.m_uv0Channel.GetCount())
				{
					dAttibutFormat::dgUV uv;
					uv.m_u = uv0[0].m_x * alpha0 + uv0[1].m_x * alpha1 + uv0[2].m_x * alpha2;
					uv.m_v = uv0[0].m_y * alpha0 + uv0[1].m_y * alpha1 + uv0[2].m_y * alpha2;
					m_attrib.m_uv0Channel.PushBack(uv);
				}

				if (m_attrib.m_uv1Channel.GetCount() && convexMesh.m_attrib.m_uv1Channel.GetCount())
				{
					dAttibutFormat::dgUV uv;
					uv.m_u = uv1[0].m_x * alpha0 + uv1[1].m_x * alpha1 + uv1[2].m_x * alpha2;
					uv.m_v = uv1[0].m_y * alpha0 + uv1[1].m_y * alpha1 + uv1[2].m_y * alpha2;
					m_attrib.m_uv1Channel.PushBack(uv);
				}

				edge = edge->m_next;
			} while (edge != face);
		}
	}

	return retValue;
}
ndMeshEffect* ndMeshEffect::ConvexMeshIntersection(const ndMeshEffect* const convexMeshSrc) const
{
	ndMeshEffect convexMesh(*convexMeshSrc);
	convexMesh.ConvertToPolygons();
	ndMeshEffect* const convexIntersection = new ndMeshEffect(*this);
	
	dInt32 mark = convexMesh.IncLRU();
	dPolyhedra::Iterator iter(convexMesh);
	
	for (iter.Begin(); iter; iter++) 
	{
		dEdge* const convexFace = &(*iter);
		if ((convexFace->m_incidentFace > 0) && (convexFace->m_mark != mark)) 
		{
			dEdge* ptr = convexFace;
			do 
			{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != convexFace);

			dInt32 clipCode = convexIntersection->PlaneClip(convexMesh, convexFace);
			if (clipCode > 0)
			{
				delete convexIntersection;
				return nullptr;
			}
		}
	}
	
	if (!convexIntersection->GetVertexCount()) 
	{
		delete convexIntersection;
		return nullptr;
	}

	convexIntersection->RemoveUnusedVertices(nullptr);
	return convexIntersection;
}

ndMeshEffect* ndMeshEffect::InverseConvexMeshIntersection(const ndMeshEffect* const convexMeshSrc) const
{
	ndMeshEffect concaveMesh(*convexMeshSrc);
	concaveMesh.ConvertToPolygons();

	ndMeshEffect convexMesh(concaveMesh);
	ndMeshEffect* intersection = new ndMeshEffect(*this);

	ndMeshEffect* const mergedOutput = new ndMeshEffect;
	mergedOutput->BeginBuild();

	dInt32 layer = 0;
	for (dInt32 i = 0; i < intersection->m_points.m_vertex.GetCount(); i++)
	{
		intersection->m_points.m_layers[i] = layer;
	}
	
	concaveMesh.FlipWinding();
	dInt32 mark = concaveMesh.IncLRU();
	dPolyhedra::Iterator iter(concaveMesh);
	for (iter.Begin(); iter; iter++)
	{
		dEdge* const concaveFace = &(*iter);
		if ((concaveFace->m_incidentFace > 0) && (concaveFace->m_mark != mark))
		{
			dEdge* ptr = concaveFace;
			do
			{
				ptr->m_mark = mark;
				ptr = ptr->m_next;
			} while (ptr != concaveFace);

			ndMeshEffect clipTest (*intersection);
			dInt32 clipCode = clipTest.PlaneClip(concaveMesh, concaveFace);
			if (clipCode <= 0)
			{
				clipTest.Triangulate();
				mergedOutput->MergeFaces(&clipTest);

				layer++;
				if (clipCode < 0)
				{
					break;
				}

				convexMesh.RemoveAll();
				convexMesh.BeginFace();

				ptr = concaveFace;
				dInt32 count = 0;
				dInt32 vetexIndex[256];
				dInt64 attribIndex[256];
				do
				{
					vetexIndex[count] = ptr->m_incidentVertex;
					attribIndex[count] = ptr->m_userData;
					count++;
					ptr = ptr->m_prev;
				} while (ptr != concaveFace);
				dEdge* const edge = convexMesh.AddFace(count, vetexIndex, attribIndex);
				convexMesh.EndFace();

				ndMeshEffect clipTest1(*intersection);
				delete intersection;
				clipTest1.PlaneClip(convexMesh, edge);
				intersection = new ndMeshEffect(clipTest1);
				
				for (dInt32 i = 0; i < intersection->m_points.m_vertex.GetCount(); i++)
				{
					intersection->m_points.m_layers[i] = layer;
				}
			}
		}
	}
	
	delete intersection;
	mergedOutput->EndBuild(dFloat64(1.0e-8f), false);

	if (!layer)
	{
		delete mergedOutput;
		return nullptr;
	}
	mergedOutput->RemoveUnusedVertices(nullptr);
	return mergedOutput;
}

