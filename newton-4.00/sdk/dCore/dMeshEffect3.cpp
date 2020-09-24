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
#include "dMeshEffect.h"
//#include "dgBody.h"
//#include "dgWorld.h"
//#include "dMeshEffect.h"
//#include "dgCollisionConvexHull.h"


// based of the paper Hierarchical Approximate Convex Decomposition by Khaled Mamou 
// with his permission to adapt his algorithm to be more efficient.
// also making some addition to his heuristic for better convex clusters selections
// for the details http://kmamou.blogspot.com/

#if 0
#define DG_CONCAVITY_SCALE				dFloat64 (100.0f)
#define DG_CONCAVITY_PERIMETER_HANDICAP	dFloat64 (0.5f)

class dgHACDEdge
{
	public:
	dgHACDEdge ()
		:m_mark(0)
		,m_proxyListNode(nullptr)
		,m_backFaceHandicap(dFloat64 (1.0))
	{
	}
	~dgHACDEdge ()
	{
	}

	dInt32 m_mark;
	void* m_proxyListNode;
	dFloat64 m_backFaceHandicap;
};

class dHACDClusterFace
{
	public:
	dHACDClusterFace()
		:m_edge(nullptr)
		,m_area(dFloat64(0.0f))
	{
	}
	~dHACDClusterFace()
	{
	}

	dEdge* m_edge;
	dFloat64 m_area;
	dBigVector m_normal;
};

class dgHACDCluster: public dList<dHACDClusterFace>
{
	public:
	dgHACDCluster ()
		:dList<dHACDClusterFace>(nullptr)
		,m_color(0)
		,m_hierachicalClusterIndex(0)
		,m_area(dFloat64 (0.0f))
		,m_concavity(dFloat64 (0.0f))
	{
	}

	bool IsCoplanar(const dBigPlane& plane, const dMeshEffect& mesh, dFloat64 tolerance) const
	{
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			const dHACDClusterFace& info = node->GetInfo();
			dEdge* ptr = info.m_edge;
			do {
				const dBigVector& p = points[ptr->m_incidentVertex];
				dFloat64 dist = fabs(plane.Evalue(p));
				if (dist > tolerance) {
					return false;
				}
				ptr = ptr->m_next;
			} while (ptr != info.m_edge);
		}
		return true;
	}


	dInt32 m_color;
	dInt32 m_hierachicalClusterIndex;
	dFloat64 m_area;
	dFloat64 m_concavity;
};


class dgHACDClusterGraph: public dGraph<dgHACDCluster, dgHACDEdge> 
{
	public:
	class dgHACDConveHull: public dConvexHull3d
	{
		class dgConvexHullRayCastData
		{
			public:
			dFloat64 m_normalProjection;
			dConvexHull3DFace* m_face;
		};

		public: 
		dgHACDConveHull (const dgHACDConveHull& hull)
			:dConvexHull3d(hull)
			,m_mark(1)
		{
		}

		dgHACDConveHull (dgMemoryAllocator* const allocator, const dBigVector* const points, dInt32 count)
			:dConvexHull3d(allocator, &points[0].m_x, sizeof (dBigVector),count, dFloat64 (0.0f))
			,m_mark(1)
		{
		}


		dFloat64 CalculateTriangleConcavity(const dBigVector& normal, dInt32 i0, dInt32 i1, dInt32 i2, const dBigVector* const points)
		{
			dUnsigned32 head = 1;
			dUnsigned32 tail = 0;
			dBigVector pool[1<<8][3];

			pool[0][0] = points[i0];
			pool[0][1] = points[i1];
			pool[0][2] = points[i2];

			const dFloat64 rayLength = dFloat64(4.0f) * GetDiagonal();
			const dBigVector step(normal.Scale(rayLength));

			dFloat64 concavity = dFloat32(0.0f);
			dFloat64 minArea = dFloat32(0.125f);
			dFloat64 minArea2 = minArea * minArea * 0.5f;

			dInt32 maxCount = 4;
			dUnsigned32 mask = (sizeof (pool) / (3 * sizeof (pool[0][0]))) - 1;

			dConvexHull3DFace* firstGuess = nullptr;
			while ((tail != head) && (maxCount >= 0)) {
				maxCount --;
				dBigVector p0(pool[tail][0]);
				dBigVector p1(pool[tail][1]);
				dBigVector p2(pool[tail][2]);
				p0.m_w = dFloat32 (0.0f);
				p1.m_w = dFloat32 (0.0f);
				p2.m_w = dFloat32 (0.0f);

				tail = (tail + 1) & mask;

				dBigVector q1((p0 + p1 + p2).Scale(dFloat64(1.0f / 3.0f)));
				dBigVector q0(q1 + step);

				dFloat64 param = RayCast(q0, q1, &firstGuess);
				if (param > dFloat64(1.0f)) {
					param = dFloat64(1.0f);
				}
				dBigVector dq(step.Scale(dFloat32(1.0f) - param));
				dAssert(dq.m_w == dFloat32(0.0f));
				dFloat64 lenght2 = sqrt (dq.DotProduct(dq).GetScalar());
				if (lenght2 > concavity) {
					concavity = lenght2;
				}

				if (((head + 1) & mask) != tail) {
					dBigVector edge10(p1 - p0);
					dBigVector edge20(p2 - p0);
					dBigVector n(edge10.CrossProduct(edge20));
					dAssert(n.m_w == dFloat32(0.0f));
					dFloat64 area2 = n.DotProduct(n).GetScalar();
					if (area2 > minArea2) {
						dBigVector p01((p0 + p1).Scale(dFloat64(0.5f)));
						dBigVector p12((p1 + p2).Scale(dFloat64(0.5f)));
						dBigVector p20((p2 + p0).Scale(dFloat64(0.5f)));

						pool[head][0] = p0;
						pool[head][1] = p01;
						pool[head][2] = p20;
						head = (head + 1) & mask;

						if (((head + 1) & mask) != tail) {
							pool[head][0] = p1;
							pool[head][1] = p12;
							pool[head][2] = p01;
							head = (head + 1) & mask;

							if (((head + 1) & mask) != tail)	{
								pool[head][0] = p2;
								pool[head][1] = p20;
								pool[head][2] = p12;
								head = (head + 1) & mask;
							}
						}
					}
				}
			}
			return concavity;
		}



		dFloat64 FaceRayCast (const dConvexHull3DFace* const face, const dBigVector& origin, const dBigVector& dist, dFloat64& normalProjection) const
		{
			dInt32 i0 = face->m_index[0];
			dInt32 i1 = face->m_index[1];
			dInt32 i2 = face->m_index[2];

			const dBigVector& p0 = m_points[i0];
			dBigVector normal ((m_points[i1] - p0).CrossProduct(m_points[i2] - p0));

			//dFloat64 N = (origin - p0) % normal;
			dAssert(normal.m_w == dFloat32(0.0f));
			dFloat64 N = normal.DotProduct(origin - p0).GetScalar();
			dFloat64 D = normal.DotProduct(dist).GetScalar();

			if (fabs(D) < dFloat64 (1.0e-16f)) { // 
				normalProjection = dFloat32 (0.0);
				if (N > dFloat64 (0.0f)) {
					return dFloat32 (-1.0e30);
				} else {

					return dFloat32 (1.0e30);
				}
			}
			normalProjection = D;
			return - N / D;
		}

		dConvexHull3DFace* ClosestFaceVertexToPoint (const dBigVector& point)
		{
			// note, for this function to be effective point should be an already close point to the Hull.
			// for example casting the point to the OBB or the AABB of the full is a good first guess. 
			dConvexHull3DFace* closestFace = &GetFirst()->GetInfo();	
			dInt8 pool[256 * (sizeof (dConvexHull3DFace*) + sizeof (dFloat64))];
			dUpHeap<dConvexHull3DFace*,dFloat64> heap (pool, sizeof (pool));

			for (dInt32 i = 0; i < 3; i ++) {
				dBigVector dist (m_points[closestFace->m_index[i]] - point);
				dAssert(dist.m_w == dFloat32(0.0f));
				heap.Push(closestFace, dist.DotProduct(dist).GetScalar());
			}

			m_mark ++;	
			dFloat64 minDist = heap.Value();
			while (heap.GetCount()) {
				dConvexHull3DFace* const face = heap[0];	
				if (heap.Value() < minDist) {
					minDist = heap.Value();
					closestFace = face;
				}
				heap.Pop();
				//face->m_mark = m_mark;
				face->SetMark(m_mark);
				for (dInt32 i = 0; i < 3; i ++) {
					dConvexHull3DFace* twin = &face->GetTwin(i)->GetInfo();	
					if (twin->GetMark() != m_mark) {
						dBigVector dist (m_points[twin->m_index[i]] - point);
						// use hysteresis to prevent stops at a local minimal, but at the same time fast descend
						dAssert(dist.m_w == dFloat32(0.0f));
						dFloat64 dist2 = dist.DotProduct(dist).GetScalar();
						if (dist2 < (minDist * dFloat64 (1.001f))) {
							heap.Push(twin, dist2);
						}
					}
				}
			}

			return closestFace;
		}


		// this version have input sensitive complexity (approximately  log2)
		// when casting parallel rays and using the last face as initial guess this version has const time complexity 
		dFloat64 RayCast (const dBigVector& localP0, const dBigVector& localP1, dConvexHull3DFace** firstFaceGuess)
		{
			dConvexHull3DFace* face = &GetFirst()->GetInfo();
			if (firstFaceGuess && *firstFaceGuess) {
				face = *firstFaceGuess;
			} else {
				if (GetCount() > 32) {
					dVector q0 (localP0);
					dVector q1 (localP1);
					if (dRayBoxClip (q0, q1, m_aabbP0, m_aabbP1)) {
						face = ClosestFaceVertexToPoint (q0);
					}
				}
			}

			m_mark ++;	
			face->SetMark (m_mark);
			dInt8 pool[256 * (sizeof (dgConvexHullRayCastData) + sizeof (dFloat64))];
			dDownHeap<dgConvexHullRayCastData,dFloat64> heap (pool, sizeof (pool));

			dFloat64 t0 = dFloat64 (-1.0e20);			//for the maximum entering segment parameter;
			dFloat64 t1 = dFloat64 ( 1.0e20);			//for the minimum leaving segment parameter;
			dBigVector dS (localP1 - localP0);		// is the segment direction vector;
			dgConvexHullRayCastData data;
			data.m_face = face;
			dFloat64 t = FaceRayCast (face, localP0, dS, data.m_normalProjection);
			if (data.m_normalProjection >= dFloat32 (0.0f)) {
				t = dFloat64 (-1.0e30);
			}

			heap.Push (data, t);
			while (heap.GetCount()) {
				dgConvexHullRayCastData data1 (heap[0]);
				t = heap.Value();
				dConvexHull3DFace* const face1 = data1.m_face;
				dFloat64 normalDistProjection = data1.m_normalProjection;
				heap.Pop();
				bool foundThisBestFace = true;
				if (normalDistProjection < dFloat64 (0.0f)) {
					if (t > t0) {
						t0 = t;
					}
					if (t0 > t1) {
						return dFloat64 (1.2f);
					}
				} else {
					foundThisBestFace = false;
				}

				for (dInt32 i = 0; i < 3; i ++) {
					dConvexHull3DFace* const face2 = &face1->GetTwin(i)->GetInfo();

					if (face2->GetMark() != m_mark) {
						face2->SetMark (m_mark);
						dgConvexHullRayCastData data2;
						data2.m_face = face2;
						t = FaceRayCast (face2, localP0, dS, data2.m_normalProjection);
						if (data2.m_normalProjection >= dFloat32 (0.0)) {
							t = dFloat64 (-1.0e30);
						} else if (t > t0) {
							foundThisBestFace = false;
						} else if (fabs (t - t0) < dFloat64 (1.0e-10f)) {
							return dConvexHull3d::RayCast (localP0, localP1);
						}
						if ((heap.GetCount() + 2)>= heap.GetMaxCount()) {
							// remove t values that are old and way outside interval [0.0, 1.0]  
							for (dInt32 j = heap.GetCount() - 1; j >= 0; j--) {
								dFloat64 val = heap.Value(j);
								if ((val < dFloat64 (-100.0f)) || (val > dFloat64 (100.0f))) {
									heap.Remove(j);
								}
							}
						}
						heap.Push (data2, t);
					}
				}
				if (foundThisBestFace) {
					if ((t0 >= dFloat64 (0.0f)) && (t0 <= dFloat64 (1.0f))) {
						if (firstFaceGuess) {
							*firstFaceGuess = face1;
						}
						return t0;
					}
					break;
				}
			}
			return dFloat64 (1.2f);
		}


		dInt32 m_mark;
	};

	class dgHACDConvacityLookAheadTree
	{
		public:
		DG_CLASS_ALLOCATOR(allocator)

			dgHACDConvacityLookAheadTree (dgMemoryAllocator* const allocator, dEdge* const face, dFloat64 concavity)
			:m_concavity(concavity)	
			,m_faceList (allocator)
			,m_left (nullptr)
			,m_right (nullptr)
		{
			m_faceList.Append(face);
		}


		dgHACDConvacityLookAheadTree (dgMemoryAllocator* const allocator, dgHACDConvacityLookAheadTree* const leftChild, dgHACDConvacityLookAheadTree* const rightChild, dFloat64 concavity)
			:m_concavity(concavity)	
			,m_faceList (allocator)
			,m_left (leftChild)
			,m_right (rightChild)
		{
			dAssert (leftChild);
			dAssert (rightChild);

			dFloat64 concavityTest = m_concavity - dFloat64 (1.0e-5f);
			if ((((m_left->m_faceList.GetCount() == 1) || (m_right->m_faceList.GetCount() == 1))) ||
				((concavityTest <= m_left->m_concavity) && (concavityTest <= m_right->m_concavity))) {
					//The the parent has lower concavity this mean that the two do no add more detail, 
					//the can be deleted and replaced the parent node
					// for example the two children can be two convex strips that are part of a larger convex piece
					// but each part has a non zero concavity, while the convex part has a lower concavity 
					m_faceList.Merge (m_left->m_faceList);
					m_faceList.Merge (m_right->m_faceList);

					delete m_left;
					delete m_right;
					m_left = nullptr;
					m_right = nullptr;
			} else {
				for (dList<dEdge*>::dListNode* node = m_left->m_faceList.GetFirst(); node; node = node->GetNext()) {
					m_faceList.Append(node->GetInfo());
				}
				for (dList<dEdge*>::dListNode* node = m_right->m_faceList.GetFirst(); node; node = node->GetNext()) {
					m_faceList.Append(node->GetInfo());
				}
			}
		}

		~dgHACDConvacityLookAheadTree ()
		{
			if (m_left) {
				dAssert (m_right);
				delete m_left;
				delete m_right;
			}
		}

		dInt32 GetNodesCount () const
		{
			dInt32 count = 0;
			dInt32 stack = 1;
			const dgHACDConvacityLookAheadTree* pool[1024];
			pool[0] = this;
			while (stack) {
				stack --;
				count ++;
				const dgHACDConvacityLookAheadTree* const root = pool[stack];
				if (root->m_left) {
					dAssert (root->m_right);
					pool[stack] = root->m_left;
					stack ++;
					dAssert (stack < sizeof (pool)/sizeof (pool[0]));
					pool[stack] = root->m_right;
					stack ++;
					dAssert (stack < sizeof (pool)/sizeof (pool[0]));
				}
			}
			return count;
		}

		void ReduceByCount (dInt32 count, dDownHeap<dgHACDConvacityLookAheadTree*, dFloat64>& approximation)
		{
			if (count < 1) {
				count = 1;
			}

			approximation.Flush();
			dgHACDConvacityLookAheadTree* tmp = this;
			approximation.Push(tmp, m_concavity);
			while ((approximation.GetCount() < count) && (approximation.Value() >= dFloat32 (0.0f))) {
				dgHACDConvacityLookAheadTree* worseCluster = approximation[0];
				dFloat64 concavity = approximation.Value();
				if (!worseCluster->m_left && (concavity >= dFloat32 (0.0f))) {
					dAssert (!worseCluster->m_right);
					approximation.Pop();
					approximation.Push(worseCluster, concavity - dFloat64 (1.0e10f));
				} else {
					dAssert (worseCluster->m_left);
					dAssert (worseCluster->m_right);
					approximation.Pop();
					approximation.Push(worseCluster->m_left, worseCluster->m_left->m_concavity);
					approximation.Push(worseCluster->m_right, worseCluster->m_right->m_concavity);
				}
			}
		}


		void ReduceByConcavity (dFloat64 concavity, dDownHeap<dgHACDConvacityLookAheadTree*, dFloat64>& approximation)
		{
			approximation.Flush();
			dgHACDConvacityLookAheadTree* tmp = this;

			approximation.Push(tmp, m_concavity);
			while (approximation.Value() > concavity) {
				dgHACDConvacityLookAheadTree* worseCluster = approximation[0];
				if (!worseCluster->m_left && approximation.Value() >= dFloat32 (0.0f)) {
					approximation.Pop();
					approximation.Push(worseCluster, dFloat32 (-1.0f));
				} else {
					dAssert (worseCluster->m_left);
					dAssert (worseCluster->m_right);
					approximation.Pop();
					approximation.Push(worseCluster->m_left, worseCluster->m_left->m_concavity);
					approximation.Push(worseCluster->m_right, worseCluster->m_right->m_concavity);
				}
			}
		}

		dFloat64 m_concavity; 
		dList<dEdge*> m_faceList;
		dgHACDConvacityLookAheadTree* m_left;
		dgHACDConvacityLookAheadTree* m_right;
	};

	class dgPairProxy
	{
		public:
		dgPairProxy()
			:m_nodeA(nullptr)
			,m_nodeB(nullptr)
			,m_hierachicalClusterIndexA(0)
			,m_hierachicalClusterIndexB(0)
			,m_area(dFloat64(0.0f))
		{
		}

		~dgPairProxy()
		{
		}

		dListNode* m_nodeA;
		dListNode* m_nodeB;
		dInt32 m_hierachicalClusterIndexA;
		dInt32 m_hierachicalClusterIndexB;
		dFloat64 m_area;
		dFloat64 m_distanceConcavity;
	};


	class dgBackFaceFinder: public dMeshEffect::dMeshBVH
	{
		public:
		dgBackFaceFinder(dMeshEffect* const mesh, dgHACDClusterGraph* const graph)
			:dMeshEffect::dMeshBVH(mesh)
			,m_clusterA(nullptr)
			,m_graph(graph)
		{
			for (dListNode* clusterNode = graph->GetFirst(); clusterNode; clusterNode = clusterNode->GetNext()) {
				dgHACDCluster& cluster = clusterNode->GetInfo().m_nodeData;
				dHACDClusterFace& face = cluster.GetFirst()->GetInfo();
				dEdge* const edge = face.m_edge;
				AddFaceNode(edge, &cluster);
			}
		}

		//dFloat64 RayFaceIntersect (const dgMeshBVHNode* const face, const dBigVector& p0, const dBigVector& p1, bool doublesided) const
		dFloat64 RayFaceIntersect (const dgMeshBVHNode* const face, const dBigVector& p0, const dBigVector& p1, void* const userData) const
		{
			dgHACDCluster* const clusterFace = (dgHACDCluster*) face->m_userData;

			dFloat64 param = dFloat32 (100.0f);
			if (clusterFace->m_color != m_clusterA->m_color) {
				param = dMeshEffect::dMeshBVH::RayFaceIntersect (face, p1, p0, nullptr);
				if ((param >= dFloat32 (0.0f)) && (param <= dFloat32(1.0f))) {
					param = dFloat32 (1.0f) - param;
				}
			}
			return param;
		}

		void CastBackFace (dListNode* const clusterNodeA, const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, dFloat32 distanceThreshold)
		{
			dAssert(0);
			/*
			dBigVector origin ((p0 + p1 + p2).Scale (dFloat32 (1.0f/3.0f)));

			dFloat32 rayDistance = distanceThreshold * dFloat32 (2.0f);


			m_clusterA = &clusterNodeA->GetInfo().m_nodeData;
			dHACDClusterFace& faceA = m_clusterA->GetFirst()->GetInfo();
			dBigVector end (origin - faceA.m_normal.Scale (rayDistance));

			dFloat64 paramOut;
			//dgMeshBVHNode* const node = FaceRayCast (origin, end, paramOut, false);

			dgMeshBVHNode* node;
			FaceRayCast (origin, end, paramOut, &node);

			if (node) {
				dgHACDCluster* const clusterB = (dgHACDCluster*) node->m_userData;
				dAssert (clusterB->m_color != m_clusterA->m_color);
				dFloat64 distance = rayDistance * paramOut;

				if (distance < distanceThreshold) {
					dHACDClusterFace& faceB = clusterB->GetFirst()->GetInfo();
					dEdge* const edgeB = faceB.m_edge;

					
					bool isAdjacent = false;
					dEdge* ptrA = faceA.m_edge;
					do {
						dEdge* ptrB = edgeB;
						do {
							if (ptrB->m_twin == ptrA) {
								ptrA = faceA.m_edge->m_prev;
								isAdjacent = true;
								break;
							}
							ptrB = ptrB->m_next;
						} while (ptrB != edgeB);

						ptrA = ptrA->m_next;
					} while (ptrA != faceA.m_edge);

					if (!isAdjacent) {
						isAdjacent = false;
						dgHACDClusterGraph::dListNode* const clusterNodeB = m_graph->GetNodeFromNodeData (clusterB);
						for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNode = clusterNodeA->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
							if (edgeNode->GetInfo().m_node == clusterNodeB) {
								isAdjacent = true;
								break;
							}
						}

						if (!isAdjacent) {
							dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* const edgeNodeAB = clusterNodeA->GetInfo().AddEdge (clusterNodeB);
							dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* const edgeNodeBA = clusterNodeB->GetInfo().AddEdge (clusterNodeA);

							dgHACDEdge& edgeAB = edgeNodeAB->GetInfo().m_edgeData;
							dgHACDEdge& edgeBA = edgeNodeBA->GetInfo().m_edgeData;
							edgeAB.m_backFaceHandicap = DG_CONCAVITY_PERIMETER_HANDICAP;
							edgeBA.m_backFaceHandicap = DG_CONCAVITY_PERIMETER_HANDICAP;
						}
					}
				}
			}
*/
		}

		
		dgHACDCluster* m_clusterA;
		dgHACDClusterGraph* m_graph;
	};

    dgHACDClusterGraph(dMeshEffect& mesh, dFloat32 backFaceDistanceFactor, dgReportProgress reportProgressCallback, void* const reportProgressUserData)
		:dGraph<dgHACDCluster, dgHACDEdge> (mesh.GetAllocator())
		,m_mark(0)
		,m_faceCount(0)
		,m_vertexMark(0)
		,m_progress(0)
		,m_concavityTreeIndex(0)
		,m_invFaceCount(dFloat32 (1.0f))
		,m_diagonal(dFloat64(1.0f))
		,m_vertexMarks(nullptr)
		,m_vertexPool(nullptr)
		,m_proxyList(mesh.GetAllocator())
		,m_concavityTreeArray(nullptr)
		,m_convexProximation(mesh.GetAllocator())
		,m_priorityHeap (mesh.GetCount() * 2 + 2048, mesh.GetAllocator())
        ,m_reportProgressCallback(reportProgressCallback)
        ,m_reportProgressUserData(reportProgressUserData)
	{
		m_faceCount = mesh.GetTotalFaceCount();

		dgMemoryAllocator* const allocator = mesh.GetAllocator();
		m_invFaceCount = dFloat32 (1.0f) / (m_faceCount);

		// init some auxiliary structures
		dInt32 vertexCount = mesh.GetVertexCount();
		m_vertexMarks = (dInt32*) dgMallocStack(vertexCount * sizeof(dInt32));
		m_vertexPool = (dBigVector*) dgMallocStack(vertexCount * sizeof(dBigVector));
		memset(m_vertexMarks, 0, vertexCount * sizeof(dInt32));

		m_concavityTreeIndex = m_faceCount + 1;
		m_concavityTreeArray = (dgHACDConvacityLookAheadTree**) dgMallocStack(2 * m_concavityTreeIndex * sizeof(dgHACDConvacityLookAheadTree*));
		memset(m_concavityTreeArray, 0, 2 * m_concavityTreeIndex * sizeof(dgHACDConvacityLookAheadTree*));

		// scan the mesh and and add a node for each face
		dInt32 color = 1;
		dMeshEffect::Iterator iter(mesh);

		dInt32 meshMask = mesh.IncLRU();
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();
		for (iter.Begin(); iter; iter++) {
			dEdge* const edge = &(*iter);
			if ((edge->m_mark != meshMask) && (edge->m_incidentFace > 0)) {

				dListNode* const clusterNode = AddNode ();
				dgHACDCluster& cluster = clusterNode->GetInfo().m_nodeData;
				cluster.SetAllocator(mesh.GetAllocator());

				dFloat64 perimeter = dFloat64(0.0f);
				dEdge* ptr = edge;
				do {
					dBigVector p1p0(points[ptr->m_incidentVertex] - points[ptr->m_prev->m_incidentVertex]);
					dAssert(p1p0.m_w == dFloat32(0.0f));
					perimeter += sqrt(p1p0.DotProduct(p1p0).GetScalar());
					ptr->m_incidentFace = color;
					ptr->m_userData = dgUnsigned64 (clusterNode);
					ptr->m_mark = meshMask;
					ptr = ptr->m_next;
				} while (ptr != edge);

				dBigVector normal (mesh.FaceNormal(edge, &points[0][0], sizeof(dBigVector)));
				dFloat64 mag = sqrt(normal.DotProduct(normal).GetScalar());

				cluster.m_color = color;
				cluster.m_hierachicalClusterIndex = color;
				cluster.m_area = dFloat64(0.5f) * mag;
				cluster.m_concavity = CalculateConcavityMetric (dFloat64 (0.0f), cluster.m_area, perimeter, 1, 0);

				dHACDClusterFace& face = cluster.Append()->GetInfo();
				face.m_edge = edge;
				face.m_area = dFloat64(0.5f) * mag;
				face.m_normal = normal.Scale(dFloat64(1.0f) / mag);

				m_concavityTreeArray[color] = new (allocator) dgHACDConvacityLookAheadTree (allocator, edge, dFloat64 (0.0f));

				color ++;
			}
		}

		// add all link adjacent faces links
		for (dListNode* clusterNode = GetFirst(); clusterNode; clusterNode = clusterNode->GetNext()) {

			dgHACDCluster& cluster = clusterNode->GetInfo().m_nodeData;
			dHACDClusterFace& face = cluster.GetFirst()->GetInfo();
			dEdge* const edge = face.m_edge;
			dEdge* ptr = edge; 
			do {
				if (ptr->m_twin->m_incidentFace > 0) {
					dAssert (ptr->m_twin->m_userData);
					dListNode* const twinClusterNode = (dListNode*) ptr->m_twin->m_userData;
					dAssert (twinClusterNode);

					bool doubleEdge = false;
					for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNode = clusterNode->GetInfo().GetFirst(); edgeNode; edgeNode = edgeNode->GetNext()) {
						if (edgeNode->GetInfo().m_node == twinClusterNode) {
							doubleEdge = true;
							break;
						}
					}
					if (!doubleEdge) {
						clusterNode->GetInfo().AddEdge (twinClusterNode);
					}
				}
				ptr = ptr->m_next;
			} while (ptr != edge);
		}

		Trace();
		dAssert (0);
/*
		// add links to back faces
		dBigVector minAABB;
		dBigVector maxAABB;
		mesh.CalculateAABB (minAABB, maxAABB);
		maxAABB -= minAABB;
		dAssert(maxAABB.m_w == dFloat32(0.0f));
		dFloat32 rayDiagonalLength = dFloat32 (sqrt (maxAABB.DotProduct(maxAABB).GetScalar()));
		m_diagonal = rayDiagonalLength;

		dgBackFaceFinder backFaces(&mesh, this);
		dFloat32 distanceThreshold = rayDiagonalLength * backFaceDistanceFactor;
		dAssert (distanceThreshold >= dFloat32 (0.0f));
		for (dListNode* clusterNodeA = GetFirst(); clusterNodeA; clusterNodeA = clusterNodeA->GetNext()) {

			dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
			dHACDClusterFace& faceA = clusterA.GetFirst()->GetInfo();
			dEdge* const edgeA = faceA.m_edge;
			dEdge* ptr = edgeA;

			dBigVector p0 (points[ptr->m_incidentVertex]);
			dBigVector p1 (points[ptr->m_next->m_incidentVertex]);
			ptr = ptr->m_next->m_next;
			do {
				dBigVector p2 (points[ptr->m_incidentVertex]);
				dBigVector p01 ((p0 + p1).Scale (dFloat32 (0.5f)));
				dBigVector p12 ((p1 + p2).Scale (dFloat32 (0.5f)));
				dBigVector p20 ((p2 + p0).Scale (dFloat32 (0.5f)));

				backFaces.CastBackFace (clusterNodeA, p0, p01, p20, distanceThreshold);
				backFaces.CastBackFace (clusterNodeA, p1, p12, p01, distanceThreshold);
				backFaces.CastBackFace (clusterNodeA, p2, p20, p12, distanceThreshold);
				backFaces.CastBackFace (clusterNodeA, p01, p12, p20, distanceThreshold);

				p1 = p2;
				ptr = ptr->m_next;
			} while (ptr != edgeA);
		}
*/
		Trace();
	}

	~dgHACDClusterGraph ()
	{
		for (dInt32 i = 0; i < m_faceCount * 2; i ++) {
			if (m_concavityTreeArray[i]) {
				delete m_concavityTreeArray[i];
			}
		}

		dgFreeStack(m_concavityTreeArray);
		dgFreeStack(m_vertexPool);
		dgFreeStack(m_vertexMarks);
	}




	void Trace() const
	{
#if 0
		for (dListNode* clusterNodeA = GetFirst(); clusterNodeA; clusterNodeA = clusterNodeA->GetNext()) {
			dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
			//dHACDClusterFace& faceA = clusterA.GetFirst()->GetInfo();
			//dEdge* const edgeA = faceA.m_edge;

			dTrace (("cluster node: %d\n", clusterA.m_color));
			dTrace (("            links: "));
			for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeA = clusterNodeA->GetInfo().GetFirst(); edgeNodeA; edgeNodeA = edgeNodeA->GetNext()) {
				dListNode* const clusterNodeB = edgeNodeA->GetInfo().m_node;
				dgHACDCluster& clusterB = clusterNodeB->GetInfo().m_nodeData;
				dTrace (("%d ", clusterB.m_color));
			}
			dTrace (("\n"));
		}
		dTrace (("\n"));
#endif
	}


	// you can insert callback here to print the progress as it collapse clusters
	bool ReportProgress ()
	{
		bool state = true;
		m_progress ++;
		if (m_reportProgressCallback) {
			dFloat32 progress = dFloat32(m_progress) * m_invFaceCount;
			state = m_reportProgressCallback (progress * dFloat32 (0.5f) + 0.5f, m_reportProgressUserData);
		}
		return state;
	}

	dMeshEffect* CreatePartitionMesh (dMeshEffect& mesh, dInt32 maxVertexPerHull)
	{
		dgMemoryAllocator* const allocator = mesh.GetAllocator();
		dMeshEffect* const convexPartionMesh = new (allocator) dMeshEffect(allocator);

		dgArray<dBigVector> convexVertexBuffer(mesh.m_points.m_vertex, mesh.m_points.m_vertex.m_count);
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();

		dInt32 layer = 0;
		convexPartionMesh->BeginBuild();
		for (dList<dgHACDConvacityLookAheadTree*>::dListNode* clusterNode = m_convexProximation.GetFirst(); clusterNode; clusterNode = clusterNode->GetNext()) {
			dgHACDConvacityLookAheadTree* const cluster = clusterNode->GetInfo();

			dInt32 vertexCount = 0;
			for (dList<dEdge*>::dListNode* faceNode = cluster->m_faceList.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
				dEdge* const edge = faceNode->GetInfo();
				dEdge* ptr = edge;
				do {
					dInt32 index = ptr->m_incidentVertex;
					convexVertexBuffer[vertexCount] = points[index];
					vertexCount++;
					ptr = ptr->m_next;
				} while (ptr != edge);
			}

			//dConvexHull3d convexHull(allocator, &convexVertexBuffer[0].m_x, sizeof(dBigVector), vertexCount, 0.0, maxVertexPerHull);
			dMeshEffect convexMesh(allocator, &convexVertexBuffer[0].m_x, vertexCount, sizeof(dBigVector), dFloat64(0.0f));
			if (convexMesh.GetCount()) {
				for (dInt32 i = 0; i < convexMesh.m_points.m_vertex.m_count; i++) {
					convexMesh.m_points.m_layers[i] = layer;
				}
				convexPartionMesh->MergeFaces(&convexMesh);
				layer++;
			}
		}
		convexPartionMesh->EndBuild(1.0e-5f);

		m_progress = m_faceCount - 1;
		ReportProgress();
		return convexPartionMesh;
	}

	dFloat64 ConcavityByFaceMedian (dInt32 faceCountA, dInt32 faceCountB) const
	{
		dFloat64 faceCountCost = DG_CONCAVITY_SCALE * dFloat64 (0.1f) * (faceCountA + faceCountB) * m_invFaceCount;
		return faceCountCost;
	}

	dFloat64 CalculateConcavityMetric (dFloat64 convexConcavity, dFloat64 area, dFloat64 perimeter, dInt32 faceCountA, dInt32 faceCountB) const 
	{
		dFloat64 edgeCost = perimeter * perimeter / (dFloat64(4.0f * dPi) * area);
		return convexConcavity * DG_CONCAVITY_SCALE + edgeCost + ConcavityByFaceMedian (faceCountA, faceCountB);
	}

	void SubmitInitialEdgeCosts (dMeshEffect& mesh) 
	{
		m_mark ++;
		for (dListNode* clusterNodeA = GetFirst(); clusterNodeA; clusterNodeA = clusterNodeA->GetNext()) {
			// call the progress callback
			for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeAB = clusterNodeA->GetInfo().GetFirst(); edgeNodeAB; edgeNodeAB = edgeNodeAB->GetNext()) {
				dgHACDEdge& edgeAB = edgeNodeAB->GetInfo().m_edgeData;
				dFloat64 weight = edgeAB.m_backFaceHandicap; 
				if (edgeAB.m_mark != m_mark) {
					edgeAB.m_mark = m_mark;
					dListNode* const clusterNodeB = edgeNodeAB->GetInfo().m_node;
					for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeBA = clusterNodeB->GetInfo().GetFirst(); edgeNodeBA; edgeNodeBA = edgeNodeBA->GetNext()) {
						dListNode* const clusterNode = edgeNodeBA->GetInfo().m_node;
						if (clusterNode == clusterNodeA) {
							dgHACDEdge& edgeBA = edgeNodeBA->GetInfo().m_edgeData;
							edgeBA.m_mark = m_mark;
							dAssert (!edgeAB.m_proxyListNode);
							dAssert (!edgeBA.m_proxyListNode);

							dAssert (edgeBA.m_backFaceHandicap == weight);
							dList<dgPairProxy>::dListNode* const proxyNode = SubmitEdgeCost (mesh, clusterNodeA, clusterNodeB, weight * edgeBA.m_backFaceHandicap);
							edgeAB.m_proxyListNode = proxyNode;
							edgeBA.m_proxyListNode = proxyNode;
							break;
						}
					}
				}
			}
		}
	}

	dInt32 CopyVertexToPool(const dMeshEffect& mesh, const dgHACDCluster& cluster, dInt32 start)
	{
		dInt32 count = start;

		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();
		for (dList<dHACDClusterFace>::dListNode* node = cluster.GetFirst(); node; node = node->GetNext()) {
			const dHACDClusterFace& clusterFace = node->GetInfo();
			dEdge* edge = clusterFace.m_edge;
			do {
				dInt32 index = edge->m_incidentVertex;
				if (m_vertexMarks[index] != m_vertexMark) {
					m_vertexMarks[index] = m_vertexMark;
					m_vertexPool[count] = points[index];
					count++;
				}
				edge = edge->m_next;
			} while (edge != clusterFace.m_edge);
		}
		return count;
	}


	void MarkInteriorClusterEdges (dMeshEffect& mesh, dInt32 mark, const dgHACDCluster& cluster, dInt32 colorA, dInt32 colorB) const
	{
		dAssert (colorA != colorB);
		for (dList<dHACDClusterFace>::dListNode* node = cluster.GetFirst(); node; node = node->GetNext()) {
			dHACDClusterFace& clusterFace = node->GetInfo();
			dEdge* edge = clusterFace.m_edge;
			do {
				if ((edge->m_twin->m_incidentFace == colorA) || (edge->m_twin->m_incidentFace == colorB)) {
					edge->m_mark = mark;
					edge->m_twin->m_mark = mark;
				}
				edge = edge->m_next;
			} while (edge != clusterFace.m_edge);
		}
	}

	dFloat64 CalculateClusterPerimeter (dMeshEffect& mesh, dInt32 mark, const dgHACDCluster& cluster, dInt32 colorA, dInt32 colorB) const
	{
		dAssert (colorA != colorB);
		dFloat64 perimeter = dFloat64 (0.0f);
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();
		for (dList<dHACDClusterFace>::dListNode* node = cluster.GetFirst(); node; node = node->GetNext()) {
			dHACDClusterFace& clusterFace = node->GetInfo();
			dEdge* edge = clusterFace.m_edge;
			do {
				if (!((edge->m_twin->m_incidentFace == colorA) || (edge->m_twin->m_incidentFace == colorB))) {
					dBigVector p1p0(points[edge->m_twin->m_incidentVertex] - points[edge->m_incidentVertex]);
					dAssert(p1p0.m_w == dFloat32(0.0f));
					perimeter += sqrt(p1p0.DotProduct(p1p0).GetScalar());
				}
				edge = edge->m_next;
			} while (edge != clusterFace.m_edge);
		}

		return perimeter;
	}

	void HeapCollectGarbage () 
	{
		if ((m_priorityHeap.GetCount() + 20) > m_priorityHeap.GetMaxCount()) {
			for (dInt32 i = m_priorityHeap.GetCount() - 1; i >= 0; i--) {
				dList<dgPairProxy>::dListNode* const emptyNode = m_priorityHeap[i];
				dgPairProxy& emptyPair = emptyNode->GetInfo();
				if ((emptyPair.m_nodeA == nullptr) && (emptyPair.m_nodeB == nullptr)) {
					m_priorityHeap.Remove(i);
				}
			}
		}
	}


	dFloat64 CalculateConcavity(dgHACDConveHull& hull, const dMeshEffect& mesh, const dgHACDCluster& cluster)
	{
		dFloat64 concavity = dFloat32(0.0f);

		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();
		for (dList<dHACDClusterFace>::dListNode* node = cluster.GetFirst(); node; node = node->GetNext()) {
			dHACDClusterFace& clusterFace = node->GetInfo();
			dEdge* edge = clusterFace.m_edge;
			dInt32 i0 = edge->m_incidentVertex;
			dInt32 i1 = edge->m_next->m_incidentVertex;
			for (dEdge* ptr = edge->m_next->m_next; ptr != edge; ptr = ptr->m_next) {
				dInt32 i2 = ptr->m_incidentVertex;
				dFloat64 val = hull.CalculateTriangleConcavity(clusterFace.m_normal, i0, i1, i2, points);
				if (val > concavity) {
					concavity = val;
				}
				i1 = i2;
			}
		}

		return concavity;
	}

	dFloat64 CalculateConcavity (dgHACDConveHull& hull, dMeshEffect& mesh, dgHACDCluster& clusterA, dgHACDCluster& clusterB)
	{
		return dMax(CalculateConcavity(hull, mesh, clusterA), CalculateConcavity(hull, mesh, clusterB));
	}


	dList<dgPairProxy>::dListNode* SubmitEdgeCost (dMeshEffect& mesh, dListNode* const clusterNodeA, dListNode* const clusterNodeB, dFloat64 perimeterHandicap)
	{
		dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
		dgHACDCluster& clusterB = clusterNodeB->GetInfo().m_nodeData;
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();

		bool flatStrip = true;
		dFloat64 tol = dFloat64 (1.0e-5f) * m_diagonal;
		dHACDClusterFace& clusterFaceA = clusterA.GetFirst()->GetInfo();
		dAssert(clusterFaceA.m_normal.m_w == dFloat32(0.0f));
		dBigPlane plane(clusterFaceA.m_normal, - points[clusterFaceA.m_edge->m_incidentVertex].DotProduct(clusterFaceA.m_normal).GetScalar());

		if (clusterA.GetCount() > 1) {
			flatStrip = clusterA.IsCoplanar(plane, mesh, tol);
		}

		if (flatStrip) {
			flatStrip = clusterB.IsCoplanar(plane, mesh, tol);
		}

		dList<dgPairProxy>::dListNode* pairNode = nullptr;
		if (!flatStrip) {
			m_vertexMark ++;
			dInt32 vertexCount = CopyVertexToPool(mesh, clusterA, 0);
			vertexCount = CopyVertexToPool(mesh, clusterB, vertexCount);

			dgHACDConveHull convexHull(mesh.GetAllocator(), m_vertexPool, vertexCount);

			if (convexHull.GetVertexCount()) {
				dInt32 mark = mesh.IncLRU();
				MarkInteriorClusterEdges (mesh, mark, clusterA, clusterA.m_color, clusterB.m_color);
				MarkInteriorClusterEdges (mesh, mark, clusterB, clusterA.m_color, clusterB.m_color);

				dFloat64 area = clusterA.m_area + clusterB.m_area;
				dFloat64 perimeter = CalculateClusterPerimeter (mesh, mark, clusterA, clusterA.m_color, clusterB.m_color) +
									  CalculateClusterPerimeter (mesh, mark, clusterB, clusterA.m_color, clusterB.m_color);
				dFloat64 concavity = CalculateConcavity (convexHull, mesh, clusterA, clusterB);

				if (concavity < dFloat64(1.0e-3f)) {
					concavity = dFloat64(0.0f);
				}

				// see if the heap will overflow
				HeapCollectGarbage ();

				// add a new pair to the heap
				dList<dgPairProxy>::dListNode* pairNode1 = m_proxyList.Append();
				dgPairProxy& pair = pairNode1->GetInfo();
				pair.m_nodeA = clusterNodeA;
				pair.m_nodeB = clusterNodeB;
				pair.m_distanceConcavity = concavity;
				pair.m_hierachicalClusterIndexA = clusterA.m_hierachicalClusterIndex;
				pair.m_hierachicalClusterIndexB = clusterB.m_hierachicalClusterIndex;

				pair.m_area = area;
				dFloat64 cost = CalculateConcavityMetric (concavity, area * perimeterHandicap, perimeter * perimeterHandicap, clusterA.GetCount(), clusterB.GetCount());
				m_priorityHeap.Push(pairNode1, cost);

				return pairNode1;
			}
		}
		return pairNode;
	}


	bool CollapseEdge (dList<dgPairProxy>::dListNode* const pairNode, dMeshEffect& mesh, dFloat64 concavity)
	{
		dListNode* adjacentNodes[1024];
		dgPairProxy& pair = pairNode->GetInfo();

		dgMemoryAllocator* const allocator = mesh.GetAllocator();

		bool continueColapsing = true;
		dAssert((pair.m_nodeA && pair.m_nodeB) || (!pair.m_nodeA && !pair.m_nodeB));
		if (pair.m_nodeA && pair.m_nodeB && continueColapsing) {
			// call the progress callback
			continueColapsing = ReportProgress();

			dListNode* const clusterNodeA = pair.m_nodeA;
			dListNode* const clusterNodeB = pair.m_nodeB;
			dAssert (clusterNodeA != clusterNodeB);

			dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
			dgHACDCluster& clusterB = clusterNodeB->GetInfo().m_nodeData;

			dAssert (&clusterA != &clusterB);
			dAssert(clusterA.m_color != clusterB.m_color);

			dgHACDConvacityLookAheadTree* const leftTree = m_concavityTreeArray[pair.m_hierachicalClusterIndexA];
			dgHACDConvacityLookAheadTree* const rightTree = m_concavityTreeArray[pair.m_hierachicalClusterIndexB];
			dAssert (leftTree);
			dAssert (rightTree);
			m_concavityTreeArray[pair.m_hierachicalClusterIndexA] = nullptr;
			m_concavityTreeArray[pair.m_hierachicalClusterIndexB] = nullptr;
			dAssert (m_concavityTreeIndex < (2 * (m_faceCount + 1)));

			dFloat64 treeConcavity = pair.m_distanceConcavity;
//			 dAssert (treeConcavity < 0.1);
			m_concavityTreeArray[m_concavityTreeIndex] = new (allocator) dgHACDConvacityLookAheadTree (allocator, leftTree, rightTree, treeConcavity);
			clusterA.m_hierachicalClusterIndex = m_concavityTreeIndex;
			clusterB.m_hierachicalClusterIndex = m_concavityTreeIndex;
			m_concavityTreeIndex ++;

			// merge two clusters
			while (clusterB.GetCount()) {

				dgHACDCluster::dListNode* const nodeB = clusterB.GetFirst();
				clusterB.Unlink(nodeB);
	
				// now color code all faces of the merged cluster
				dHACDClusterFace& faceB = nodeB->GetInfo();
				dEdge* ptr = faceB.m_edge;
				do {
					ptr->m_incidentFace = clusterA.m_color;
					ptr = ptr->m_next;
				} while (ptr != faceB.m_edge);
				clusterA.Append(nodeB);
			}
			clusterA.m_area = pair.m_area;
			clusterA.m_concavity = concavity;

			// invalidate all proxies that are still in the heap
			dInt32 adjacentCount = 1;
			adjacentNodes[0] = clusterNodeA;
			for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeAB = clusterNodeA->GetInfo().GetFirst(); edgeNodeAB; edgeNodeAB = edgeNodeAB->GetNext()) {
				dgHACDEdge& edgeAB = edgeNodeAB->GetInfo().m_edgeData;
				dList<dgPairProxy>::dListNode* const proxyNode = (dList<dgPairProxy>::dListNode*) edgeAB.m_proxyListNode;
				if (proxyNode) {
					dgPairProxy& pairProxy = proxyNode->GetInfo();
					dAssert ((edgeNodeAB->GetInfo().m_node == pairProxy.m_nodeA) || (edgeNodeAB->GetInfo().m_node == pairProxy.m_nodeB));
					pairProxy.m_nodeA = nullptr;
					pairProxy.m_nodeB = nullptr;
					edgeAB.m_proxyListNode = nullptr;
				}

				adjacentNodes[adjacentCount] = edgeNodeAB->GetInfo().m_node;
				adjacentCount ++;
				dAssert (adjacentCount < sizeof (adjacentNodes)/ sizeof (adjacentNodes[0]));
			}

			for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeBA = clusterNodeB->GetInfo().GetFirst(); edgeNodeBA; edgeNodeBA = edgeNodeBA->GetNext()) {
				dgHACDEdge& edgeBA = edgeNodeBA->GetInfo().m_edgeData;
				dList<dgPairProxy>::dListNode* const proxyNode = (dList<dgPairProxy>::dListNode*) edgeBA.m_proxyListNode;
				if (proxyNode) {
					dgPairProxy& pairProxy = proxyNode->GetInfo();
					pairProxy.m_nodeA = nullptr;
					pairProxy.m_nodeB = nullptr;
					edgeBA.m_proxyListNode = nullptr;
				}

				bool alreadyLinked = false;
				dListNode* const node = edgeNodeBA->GetInfo().m_node;
				for (dInt32 i = 0; i < adjacentCount; i ++) {
					if (node == adjacentNodes[i]) {
						alreadyLinked = true;
						break;
					}
				}
				if (!alreadyLinked) {
					clusterNodeA->GetInfo().AddEdge (node);
					node->GetInfo().AddEdge (clusterNodeA);
				}
			}
			DeleteNode (clusterNodeB);

			// submit all new costs for each edge connecting this new node to any other node 
			for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeAB = clusterNodeA->GetInfo().GetFirst(); edgeNodeAB; edgeNodeAB = edgeNodeAB->GetNext()) {
				dgHACDEdge& edgeAB = edgeNodeAB->GetInfo().m_edgeData;
				dFloat64 weigh = edgeAB.m_backFaceHandicap;
				dListNode* const clusterNodeB1 = edgeNodeAB->GetInfo().m_node;
				for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeBA = clusterNodeB1->GetInfo().GetFirst(); edgeNodeBA; edgeNodeBA = edgeNodeBA->GetNext()) {
					dListNode* const clusterNode = edgeNodeBA->GetInfo().m_node;
					if (clusterNode == clusterNodeA) {
						dgHACDEdge& edgeBA = edgeNodeBA->GetInfo().m_edgeData;
						dList<dgPairProxy>::dListNode* const proxyNode = SubmitEdgeCost (mesh, clusterNodeA, clusterNodeB1, weigh * edgeBA.m_backFaceHandicap);
						if (proxyNode) {
							edgeBA.m_proxyListNode = proxyNode;
							edgeAB.m_proxyListNode = proxyNode;
						}
						break;
					}
				}
			}
		}
		m_proxyList.Remove(pairNode);

		return continueColapsing;
	}

	bool CollapseClusters (dMeshEffect& mesh, dFloat64 maxConcavity___, dInt32 maxClustesCount)
	{
		bool collapseEdgeState = true;
		while (m_priorityHeap.GetCount() && collapseEdgeState) {
			dFloat64 concavity =  m_priorityHeap.Value();
			dList<dgPairProxy>::dListNode* const pairNode = m_priorityHeap[0];
			m_priorityHeap.Pop();
			collapseEdgeState = CollapseEdge (pairNode, mesh, concavity);
		}

		if (collapseEdgeState) {
			dInt32 treeCounts = 0;
			for (dInt32 i = 0; i < m_concavityTreeIndex; i ++) {
				if (m_concavityTreeArray[i]) {
					m_concavityTreeArray[treeCounts] = m_concavityTreeArray[i];
					m_concavityTreeArray[i] = nullptr;
					treeCounts ++;
				}
			}

			if (treeCounts > 1) {
				for (dInt32 i = 0; i < treeCounts; i ++) {
					dAssert (m_concavityTreeArray[i]);
					if (m_concavityTreeArray[i]->m_faceList.GetCount()==1) {
						delete m_concavityTreeArray[i];
						m_concavityTreeArray[i] = m_concavityTreeArray[treeCounts-1];
						m_concavityTreeArray[treeCounts-1]= nullptr;
						treeCounts --;
						i--;
					}
				}


				dFloat32 largeConcacvity = 10000;
				while (treeCounts > 1)	 {
					dgHACDConvacityLookAheadTree* const leftTree = m_concavityTreeArray[treeCounts-1];
					dgHACDConvacityLookAheadTree* const rightTree = m_concavityTreeArray[treeCounts-2];
					m_concavityTreeArray[treeCounts-1] = nullptr;
					m_concavityTreeArray[treeCounts-2] = new (mesh.GetAllocator()) dgHACDConvacityLookAheadTree (mesh.GetAllocator(), leftTree, rightTree, largeConcacvity);
					largeConcacvity *= 2;
					treeCounts --;
				}

			}

			dgHACDConvacityLookAheadTree* const tree = m_concavityTreeArray[0];
			if (tree) {
				dDownHeap<dgHACDConvacityLookAheadTree*, dFloat64> approximation(maxClustesCount * 2, mesh.GetAllocator());

				tree->ReduceByCount (maxClustesCount, approximation);
				//tree->ReduceByConcavity (maxConcavity, approximation);

//while ((approximation.Value() + dFloat64 (1.0e10f)) > 1.0e-5) {
//approximation.Pop();
//}

				while (approximation.GetCount()) {
					m_convexProximation.Append(approximation[0]);
					approximation.Pop();
				}
			}
		}
		return collapseEdgeState;
	}


	dInt32 m_mark;
	dInt32 m_faceCount;
	dInt32 m_vertexMark;
	dInt32 m_progress;
	dInt32 m_concavityTreeIndex;
	dFloat32 m_invFaceCount;
	dFloat64 m_diagonal;
	dInt32* m_vertexMarks;
	dBigVector* m_vertexPool;
	dList<dgPairProxy> m_proxyList;
	dgHACDConvacityLookAheadTree** m_concavityTreeArray;	
	dList<dgHACDConvacityLookAheadTree*> m_convexProximation;
	dUpHeap<dList<dgPairProxy>::dListNode*, dFloat64> m_priorityHeap;
	dgReportProgress m_reportProgressCallback;
    void* m_reportProgressUserData;
};

dMeshEffect* dMeshEffect::CreateConvexApproximation(dFloat32 maxConcavity, dFloat32 backFaceDistanceFactor, dInt32 maxHullsCount, dInt32 maxVertexPerHull, dgReportProgress reportProgressCallback, void* const progressReportUserData) const
{
	//	dgMeshEffect triangleMesh(*this);
	if (maxHullsCount <= 1) {
		maxHullsCount = 1;
	}
	if (maxConcavity <= dFloat32 (1.0e-5f)) {
		maxConcavity = dFloat32 (1.0e-5f);
	}

	if (maxVertexPerHull < 4) {
		maxVertexPerHull = 4;
	}
	backFaceDistanceFactor = dClamp(backFaceDistanceFactor, dFloat32 (1.0e-6f), dFloat32 (1.0f));

	dMeshEffect* partition = nullptr;

	// make a copy of the mesh
	dMeshEffect mesh(*this);
	mesh.m_attrib.m_materialChannel.Clear();
	mesh.m_attrib.m_normalChannel.Clear();
	mesh.m_attrib.m_binormalChannel.Clear();
	mesh.m_attrib.m_colorChannel.Clear();
	mesh.m_attrib.m_uv0Channel.Clear();
	mesh.m_attrib.m_uv1Channel.Clear();
	mesh.Triangulate ();

	mesh.UnpackAttibuteData();
	mesh.PackAttibuteData();
	mesh.UnpackPoints();
	bool state = mesh.Optimize (&mesh.m_points.m_vertex[0].m_x, sizeof (dBigVector), reportProgressCallback, progressReportUserData, dFloat32 (1.0e-3f), 1500);
	// optimized override userdata
	dPolyhedra::Iterator iter(mesh);
	for (iter.Begin(); iter; iter++) {
		dEdge* const edge = &iter.GetNode()->GetInfo();
		if (edge->m_incidentFace > 0) {
			edge->m_userData = edge->m_incidentVertex;
		}
	}
	mesh.PackPoints(dFloat32 (1.0e-24f));
	if (state) {
		mesh.DeleteDegenerateFaces (&mesh.m_points.m_vertex[0].m_x, sizeof (dBigVector), dFloat32 (1.0e-12f));
		mesh.RepairTJoints();
		mesh.ConvertToPolygons();
		//mesh.SaveOFF ("xxxxxx.off");

		// create a general connectivity graph    
		dgHACDClusterGraph graph (mesh, backFaceDistanceFactor, reportProgressCallback, progressReportUserData);

		// calculate initial edge costs
		graph.SubmitInitialEdgeCosts (mesh);

		// collapse the graph
		if (graph.CollapseClusters (mesh, maxConcavity, maxHullsCount)) {
			// Create Partition Mesh
			partition = graph.CreatePartitionMesh (mesh, maxVertexPerHull);
		}
	}

	return partition;
}

#endif