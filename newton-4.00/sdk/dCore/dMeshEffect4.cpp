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


#if 0

#if 0
#define DG_BUILD_HIERACHICAL_HACD

#define DG_CONCAVITY_MAX_THREADS	  8
#define DG_CONCAVITY_SCALE dFloat64 (100.0f)




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

	bool IsCoplanar(const dBigPlane& plane, const dgMeshEffect& mesh, dFloat64 tolerance) const
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


class dgHACDClusterGraph
	:public dGraph<dgHACDCluster, dgHACDEdge> 
	,public dgAABBPolygonSoup 
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

			const dBigVector step(normal.Scale(dFloat64(4.0f) * GetDiagonal()));

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
				tail = (tail + 1) & mask;

				dBigVector q1((p0 + p1 + p2).Scale(dFloat64(1.0f / 3.0f)));
				dBigVector q0(q1 + step);

				//dFloat64 param = convexHull.RayCast(q0, q1, &firstGuess);
				dFloat64 param = FastRayCast(q0, q1, &firstGuess);
				if (param > dFloat64(1.0f)) {
					param = dFloat64(1.0f);
				}
				dBigVector dq(step.Scale(dFloat32(1.0f) - param));
				dFloat64 lenght2 = sqrt (dq % dq);
				//dAssert (lenght2 < GetDiagonal());
				if (lenght2 > concavity) {
					concavity = lenght2;
				}

				if (((head + 1) & mask) != tail) {
					dBigVector edge10(p1 - p0);
					dBigVector edge20(p2 - p0);
					dBigVector n(edge10 * edge20);
					dFloat64 area2 = n % n;
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
			dBigVector normal ((m_points[i1] - p0) * (m_points[i2] - p0));

			dFloat64 N = (origin - p0) % normal;
			dFloat64 D = dist % normal;

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
				heap.Push(closestFace, dist % dist);
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
					//const dConvexHull3DFace* twin = &face->m_twin[i]->GetInfo();	
					dConvexHull3DFace* twin = &face->GetTwin(i)->GetInfo();	
					//if (twin->m_mark != m_mark) {
					if (twin->GetMark() != m_mark) {
						dBigVector dist (m_points[twin->m_index[i]] - point);
						// use hysteresis to prevent stops at a local minimal, but at the same time fast descend
						dFloat64 dist2 = dist % dist;
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
			//face->m_mark = m_mark;
			face->SetMark (m_mark);
			dInt8 pool[256 * (sizeof (dgConvexHullRayCastData) + sizeof (dFloat64))];
			dDownHeap<dgConvexHullRayCastData,dFloat64> heap (pool, sizeof (pool));

			dFloat64 t0 = dFloat64 (-1.0e20);			//for the maximum entering segment parameter;
			dFloat64 t1 = dFloat64 ( 1.0e20);			//for the minimum leaving segment parameter;
			dBigVector dS (localP1 - localP0);		// is the segment direction vector;
			dgConvexHullRayCastData data;
			data.m_face = face;
			dFloat64 t = FaceRayCast (face, localP0, dS, data.m_normalProjection);
			if (data.m_normalProjection >= dFloat32 (0.0)) {
				t = dFloat64 (-1.0e30);
			}

			heap.Push (data, t);
			while (heap.GetCount()) {
				dgConvexHullRayCastData data (heap[0]);
				dFloat64 t = heap.Value();
				dConvexHull3DFace* face = data.m_face;
				dFloat64 normalDistProjection = data.m_normalProjection;
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
					//dConvexHull3DFace* const face1 = &face->m_twin[i]->GetInfo();
					dConvexHull3DFace* const face1 = &face->GetTwin(i)->GetInfo();

					//if (face1->m_mark != m_mark) {
					if (face1->GetMark() != m_mark) {
						//face1->m_mark = m_mark;
						face1->SetMark (m_mark);
						dgConvexHullRayCastData data;
						data.m_face = face1;
						dFloat64 t = FaceRayCast (face1, localP0, dS, data.m_normalProjection);
						if (data.m_normalProjection >= dFloat32 (0.0)) {
							t = dFloat64 (-1.0e30);
						} else if (t > t0) {
							foundThisBestFace = false;
						} else if (fabs (t - t0) < dFloat64 (1.0e-10f)) {
							return dConvexHull3d::RayCast (localP0, localP1);
						}
						if ((heap.GetCount() + 2)>= heap.GetMaxCount()) {
							// remove t values that are old and way outside interval [0.0, 1.0]  
							for (dInt32 i = heap.GetCount() - 1; i >= 0; i--) {
								dFloat64 val = heap.Value(i);
								if ((val < dFloat64 (-100.0f)) || (val > dFloat64 (100.0f))) {
									heap.Remove(i);
								}
							}
						}
						heap.Push (data, t);
					}
				}
				if (foundThisBestFace) {
					if ((t0 >= dFloat64 (0.0f)) && (t0 <= dFloat64 (1.0f))) {
						if (firstFaceGuess) {
							*firstFaceGuess = face;
						}
						return t0;
					}
					break;
				}
			}

			return dFloat64 (1.2f);

		}

		dFloat64 FastRayCast (const dBigVector& localP0, const dBigVector& localP1, dConvexHull3DFace** guess)
		{
#if 0
	#ifdef _DEBUG
			dFloat64 t0 = dConvexHull3d::RayCast (localP0, localP1);
			dFloat64 t1 = RayCast (localP0, localP1, guess);
			dAssert (fabs(t0 - t1) < dFloat64 (1.0e-5f));
	#endif
#endif

			//return dConvexHull3d::RayCast (localP0, localP1);
			return RayCast (localP0, localP1, guess);
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
			//if ((m_left->m_faceList.GetCount() == 1) || (m_right->m_faceList.GetCount() == 1)) {
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
//			dInt32 nodesCount = GetNodesCount();

			approximation.Flush();
			dgHACDConvacityLookAheadTree* tmp = this;
			approximation.Push(tmp, m_concavity);
//			nodesCount --;
			//while (nodesCount && (approximation.GetCount() < count) && (approximation.Value() >= dFloat32 (0.0f))) {
			while ((approximation.GetCount() < count) && (approximation.Value() >= dFloat32 (0.0f))) {
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
//					nodesCount -= 2;
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

	class dgHACDRayCasterContext: public dgFastRayTest
	{
		public:
		dgHACDRayCasterContext (const dVector& l0, const dVector& l1, dgHACDClusterGraph* const me, dInt32 mycolor)
			:dgFastRayTest (l0, l1)
			,m_myColor(mycolor)
			,m_colorHit(-1)
			,m_param (1.0f) 
			,m_me (me) 
		{
		}

		dInt32 m_myColor;
		dInt32 m_colorHit;
		dFloat32 m_param;
		dgHACDClusterGraph* m_me;
	};


	dgHACDClusterGraph(dMeshEffect& mesh, dFloat32 backFaceDistanceFactor, dgReportProgress reportProgressCallback)
		:dGraph<dgHACDCluster, dgHACDEdge> (mesh.GetAllocator())
		,dgAABBPolygonSoup()
		,m_mark(0)
		,m_faceCount(0)
		,m_vertexMark(0)
		,m_progress(0)
		,m_cancavityTreeIndex(0)
		,m_invFaceCount(dFloat32 (1.0f))
		,m_vertexMarks(nullptr)
		,m_diagonal(dFloat64(1.0f))
		,m_vertexPool(nullptr)
		,m_proxyList(mesh.GetAllocator())
		,m_concavityTreeArray(nullptr)
		,m_convexProximation(mesh.GetAllocator())
		,m_priorityHeap (mesh.GetCount() + 2048, mesh.GetAllocator())
		,m_reportProgressCallback (reportProgressCallback)
		,m_parallerConcavityCalculator(mesh.GetAllocator())
	{
		
		m_parallerConcavityCalculator.SetThreadsCount(DG_CONCAVITY_MAX_THREADS);

		// precondition the mesh for better approximation
		mesh.ConvertToPolygons();

		m_faceCount = mesh.GetTotalFaceCount();

		dgMemoryAllocator* const allocator = mesh.GetAllocator();
		m_invFaceCount = dFloat32 (1.0f) / (m_faceCount);

		// init some auxiliary structures
		dInt32 vertexCount = mesh.GetVertexCount();
		m_vertexMarks =  (dInt32*) dgMallocStack(vertexCount * sizeof(dInt32));
		m_vertexPool =  (dBigVector*) dgMallocStack(vertexCount * sizeof(dBigVector));
		memset(m_vertexMarks, 0, vertexCount * sizeof(dInt32));

		m_cancavityTreeIndex = m_faceCount + 1;
		m_concavityTreeArray = (dgHACDConvacityLookAheadTree**) dgMallocStack(2 * m_cancavityTreeIndex * sizeof(dgHACDConvacityLookAheadTree*));
		memset(m_concavityTreeArray, 0, 2 * m_cancavityTreeIndex * sizeof(dgHACDConvacityLookAheadTree*));

		// scan the mesh and and add a node for each face
		dInt32 color = 1;
		dMeshEffect::Iterator iter(mesh);

		dInt32 meshMask = mesh.IncLRU();
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();
		for (iter.Begin(); iter; iter++) {
			dEdge* const edge = &(*iter);
			if ((edge->m_mark != meshMask) && (edge->m_incidentFace > 0)) {

				// call the progress callback
				//ReportProgress();

				dListNode* const clusterNode = AddNode ();
				dgHACDCluster& cluster = clusterNode->GetInfo().m_nodeData;
				cluster.SetAllocator(mesh.GetAllocator());

				dFloat64 perimeter = dFloat64(0.0f);
				dEdge* ptr = edge;
				do {
					dBigVector p1p0(points[ptr->m_incidentVertex] - points[ptr->m_prev->m_incidentVertex]);
					perimeter += sqrt(p1p0 % p1p0);
					ptr->m_incidentFace = color;
					ptr->m_userData = dgUnsigned64 (clusterNode);
					ptr->m_mark = meshMask;
					ptr = ptr->m_next;
				} while (ptr != edge);

				dBigVector normal = mesh.FaceNormal(edge, &points[0][0], sizeof(dBigVector));
				dFloat64 mag = sqrt(normal % normal);

				cluster.m_color = color;
				cluster.m_hierachicalClusterIndex = color;
				cluster.m_area = dFloat64(0.5f) * mag;
				cluster.m_concavity = CalculateConcavityMetric (dFloat64 (0.0f), cluster.m_area, perimeter, 1, 0);

				dHACDClusterFace& face = cluster.Append()->GetInfo();
				face.m_edge = edge;
				face.m_area = dFloat64(0.5f) * mag;
				face.m_normal = normal.Scale(dFloat64(1.0f) / mag);

				//m_concavityTreeArray[color] = new (allocator) dgHACDConvacityLookAheadTree (allocator, edge, cluster.m_concavity);
				m_concavityTreeArray[color] = new (allocator) dgHACDConvacityLookAheadTree (allocator, edge, dFloat64 (0.0f));

				color ++;
			}
		}

		// add all link adjacent faces links
		for (dListNode* clusterNode = GetFirst(); clusterNode; clusterNode = clusterNode->GetNext()) {

			// call the progress callback
			//ReportProgress();

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

		// add links to back faces
		dgPolygonSoupDatabaseBuilder builder (mesh.GetAllocator());
		dVector polygon[64];
		dInt32 indexList[64];

		dgMatrix matrix (dgGetIdentityMatrix());
		for (dInt32 i = 0; i < sizeof (polygon) / sizeof (polygon[0]); i ++) {
			indexList[i] = i;
		}

		dBigVector minAABB;
		dBigVector maxAABB;
		mesh.CalculateAABB (minAABB, maxAABB);
		maxAABB -= minAABB;
		dFloat32 rayDiagonalLength = dFloat32 (sqrt (maxAABB % maxAABB));
		m_diagonal = rayDiagonalLength;

		builder.Begin();
		dTree<dListNode*,dInt32> clusterMap (GetAllocator());
		for (dListNode* clusterNode = GetFirst(); clusterNode; clusterNode = clusterNode->GetNext()) {

			// call the progress callback
			//ReportProgress();

			dgHACDCluster& cluster = clusterNode->GetInfo().m_nodeData;
			clusterMap.Insert(clusterNode, cluster.m_color);
			dHACDClusterFace& face = cluster.GetFirst()->GetInfo();
			dEdge* const edge = face.m_edge;
			dInt32 count = 0;
			dEdge* ptr = edge;
			do {
				polygon[count] = points[ptr->m_incidentVertex];
				count ++;
				ptr = ptr->m_prev;
			} while (ptr != edge);

			builder.AddMesh(&polygon[0].m_x, count, sizeof (dVector), 1, &count, indexList, &cluster.m_color, matrix);
		}
		builder.End(false);
		Create (builder, false);


		dFloat32 distanceThreshold = rayDiagonalLength * backFaceDistanceFactor;
		for (dListNode* clusterNodeA = GetFirst(); clusterNodeA; clusterNodeA = clusterNodeA->GetNext()) {

			// call the progress callback
			//ReportProgress();
			dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
			dHACDClusterFace& faceA = clusterA.GetFirst()->GetInfo();
			dEdge* const edgeA = faceA.m_edge;
			dEdge* ptr = edgeA;

			dVector p0 (points[ptr->m_incidentVertex]);
			dVector p1 (points[ptr->m_next->m_incidentVertex]);
			ptr = ptr->m_next->m_next;
			do {
				dVector p2 (points[ptr->m_incidentVertex]);
				dVector p01 ((p0 + p1).Scale (dFloat32 (0.5f)));
				dVector p12 ((p1 + p2).Scale (dFloat32 (0.5f)));
				dVector p20 ((p2 + p0).Scale (dFloat32 (0.5f)));

				CastBackFace (clusterNodeA, p0, p01, p20, distanceThreshold, clusterMap);
				CastBackFace (clusterNodeA, p1, p12, p01, distanceThreshold, clusterMap);
				CastBackFace (clusterNodeA, p2, p20, p12, distanceThreshold, clusterMap);
				CastBackFace (clusterNodeA, p01, p12, p20, distanceThreshold, clusterMap);

				p1 = p2;
				ptr = ptr->m_next;
			} while (ptr != edgeA);
		}

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


	void CastBackFace (
		dListNode* const clusterNodeA,
		const dVector& p0, 
		const dVector& p1, 
		const dVector& p2,
		dFloat32 distanceThreshold,
		dTree<dListNode*,dInt32>& clusterMap)
	{
		dVector origin ((p0 + p1 + p2).Scale (dFloat32 (1.0f/3.0f)));

		dFloat32 rayDistance = distanceThreshold * dFloat32 (2.0f);

		dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
		dHACDClusterFace& faceA = clusterA.GetFirst()->GetInfo();
		dVector end (origin - dVector (faceA.m_normal).Scale (rayDistance));

		dgHACDRayCasterContext ray (origin, end, this, clusterA.m_color);
		ForAllSectorsRayHit(ray, RayHit, &ray);

		if (ray.m_colorHit != -1) {
			dAssert (ray.m_colorHit != ray.m_myColor);
			dFloat32 distance = rayDistance * ray.m_param;

			if (distance < distanceThreshold) {

				dAssert (ray.m_colorHit != clusterA.m_color);
				dAssert (clusterMap.Find(ray.m_colorHit));
				dListNode* const clusterNodeB = clusterMap.Find(ray.m_colorHit)->GetInfo();
				dgHACDCluster& clusterB = clusterNodeB->GetInfo().m_nodeData;

				dHACDClusterFace& faceB = clusterB.GetFirst()->GetInfo();
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
						edgeAB.m_backFaceHandicap = dFloat64 (0.5f);
						edgeBA.m_backFaceHandicap = dFloat64 (0.5f);
					}
				}
			}
		}
	}


	void Trace() const
	{
		/*
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
		*/
	}


	// you can insert cal callback here  to print the progress as it collapse clusters
	void ReportProgress ()
	{
		m_progress ++;
		if (m_reportProgressCallback) {
			dFloat32 progress = dFloat32(m_progress) * m_invFaceCount;
			m_reportProgressCallback (progress);
		}
	}

	dMeshEffect* CreatePatitionMesh (dMeshEffect& mesh, dInt32 maxVertexPerHull)
	{
		dgMemoryAllocator* const allocator = mesh.GetAllocator();
		dMeshEffect* const convexPartionMesh = new (allocator) dMeshEffect(allocator, true);

		dMeshEffect::dgVertexAtribute polygon[256];
		memset(polygon, 0, sizeof(polygon));
		dgArray<dBigVector> convexVertexBuffer(mesh.GetCount(), GetAllocator());
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();

		convexPartionMesh->BeginBuild();
		dFloat64 layer = dFloat64 (0.0f);
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
			dConvexHull3d convexHull(allocator, &convexVertexBuffer[0].m_x, sizeof(dBigVector), vertexCount, 0.0, maxVertexPerHull);
			if (convexHull.GetCount()) {
				const dBigVector* const vertex = convexHull.GetVertexPool();
				for (dConvexHull3d::dListNode* node = convexHull.GetFirst(); node; node = node->GetNext()) {
					const dConvexHull3DFace* const face = &node->GetInfo();

					dInt32 i0 = face->m_index[0];
					dInt32 i1 = face->m_index[1];
					dInt32 i2 = face->m_index[2];

					polygon[0].m_vertex = vertex[i0];
					polygon[0].m_vertex.m_w = layer;

					polygon[1].m_vertex = vertex[i1];
					polygon[1].m_vertex.m_w = layer;

					polygon[2].m_vertex = vertex[i2];
					polygon[2].m_vertex.m_w = layer;

					convexPartionMesh->AddPolygon(3, &polygon[0].m_vertex.m_x, sizeof(dMeshEffect::dgVertexAtribute), 0);
				}
				layer += dFloat64 (1.0f);
			}
		}
		convexPartionMesh->EndBuild(1.0e-5f);

		m_progress = m_faceCount - 1;
		ReportProgress();

		return convexPartionMesh;
	}



	static dFloat32 RayHit (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
	{
		dgHACDRayCasterContext& me = *((dgHACDRayCasterContext*) context);
		dVector normal (&polygon[indexArray[indexCount] * (strideInBytes / sizeof (dFloat32))]);
		dFloat32 t = me.PolygonIntersect (normal, polygon, strideInBytes, indexArray, indexCount);
		if (t < me.m_param) {
			dInt32 faceColor = me.m_me->GetTagId(indexArray);
			if (faceColor != me.m_myColor) {
				me.m_param = t;
				me.m_colorHit = faceColor;
			}
		}
		return t;
	}


	dFloat64 ConcavityByFaceMedian (dInt32 faceCountA, dInt32 faceCountB) const
	{
		dFloat64 faceCountCost = DG_CONCAVITY_SCALE * dFloat64 (0.1f) * (faceCountA + faceCountB) * m_invFaceCount;
		//faceCountCost *= 0;
		return faceCountCost;
	}

	dFloat64 CalculateConcavityMetric (dFloat64 convexConcavity, dFloat64 area, dFloat64 perimeter, dInt32 faceCountA, dInt32 faceCountB) const 
	{
		dFloat64 edgeCost = perimeter * perimeter / (dFloat64(4.0f * dgPi) * area);
		return convexConcavity * DG_CONCAVITY_SCALE + edgeCost + ConcavityByFaceMedian (faceCountA, faceCountB);
	}

	void SubmitInitialEdgeCosts (dMeshEffect& mesh) 
	{
		m_mark ++;
		for (dListNode* clusterNodeA = GetFirst(); clusterNodeA; clusterNodeA = clusterNodeA->GetNext()) {
			// call the progress callback
			//ReportProgress();

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
					perimeter += sqrt(p1p0 % p1p0);
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

	dFloat64 CalculateConcavitySingleThread (dgHACDConveHull& hull, dMeshEffect& mesh, dgHACDCluster& clusterA, dgHACDCluster& clusterB)
	{
		return dMax(CalculateConcavity(hull, mesh, clusterA), CalculateConcavity(hull, mesh, clusterB));
	}


	class dgConvexHullRayCastContext
	{
		public: 
		dgConvexHullRayCastContext (dgHACDConveHull& hull, dMeshEffect& mesh, dgThreadHive* const manager)
			:m_atomicLock(0)
			,m_mesh(&mesh)
			,m_cluster(nullptr)
			,m_threadManager(manager)
			,m_faceNode(nullptr)
		{
			for(dInt32 i = 0; i < DG_CONCAVITY_MAX_THREADS; i ++) {
				hullArray[i] = new (mesh.GetAllocator()) dgHACDConveHull (hull);
			}
		}

		~dgConvexHullRayCastContext ()
		{
			for(dInt32 i = 0; i < DG_CONCAVITY_MAX_THREADS; i ++) {
				delete hullArray[i];
			}
		}

		void SetCluster (dgHACDCluster& cluster)
		{
			m_cluster = &cluster;
			m_node = m_cluster->GetFirst();
			memset (m_concavity, 0, sizeof (m_concavity));
		}

		dFloat64 GetConcavity() const 
		{
			dFloat64 concavity = dFloat32(0.0f);
			for (dInt32 i = 0; i < DG_CONCAVITY_MAX_THREADS; i ++) {	
				if (concavity < m_concavity[i]) {
					concavity = m_concavity[i];
				}
			}
			return concavity;
		}


		static void RayCastKernel (void* const context, dInt32 threadID)
		{
			dgConvexHullRayCastContext* const data = (dgConvexHullRayCastContext*) context;
			const dBigVector* const points = (dBigVector*) data->m_mesh->GetVertexPool();
			
			data->m_threadManager->GetIndirectLock(&data->m_atomicLock, threadID);
			dList<dHACDClusterFace>::dListNode* node = data->m_node;
			if (node) {
				data->m_node = node->GetNext();
			}
			data->m_threadManager->ReleaseIndirectLock (&data->m_atomicLock);
			for (; node;) {

				dHACDClusterFace& clusterFace = node->GetInfo();
				dEdge* edge = clusterFace.m_edge;
				dInt32 i0 = edge->m_incidentVertex;
				dInt32 i1 = edge->m_next->m_incidentVertex;
				for (dEdge* ptr = edge->m_next->m_next; ptr != edge; ptr = ptr->m_next) {
					dInt32 i2 = ptr->m_incidentVertex;
					dFloat64 val = data->hullArray[threadID]->CalculateTriangleConcavity(clusterFace.m_normal, i0, i1, i2, points);
					if (val > data->m_concavity[threadID]) {
						data->m_concavity[threadID] = val;
					}
					i1 = i2;
				}

				data->m_threadManager->GetIndirectLock(&data->m_atomicLock, threadID);
				node = data->m_node;
				if (node) {
					data->m_node = node->GetNext();;
				}
				data->m_threadManager->ReleaseIndirectLock (&data->m_atomicLock);
			}
		}


		dInt32 m_atomicLock;
		dMeshEffect* m_mesh;
		dgHACDCluster* m_cluster;
		dgThreadHive* m_threadManager;
		dList<dHACDClusterFace>::dListNode* m_node;

		dList<dHACDClusterFace>::dListNode* m_faceNode;
		dFloat64 m_concavity[DG_CONCAVITY_MAX_THREADS];
		dgHACDConveHull* hullArray[DG_CONCAVITY_MAX_THREADS];		
	};


	dFloat64 CalculateConcavityMultiThread (dgHACDConveHull& hull, dMeshEffect& mesh, dgHACDCluster& clusterA, dgHACDCluster& clusterB)
	{
		dgConvexHullRayCastContext data (hull, mesh, &m_parallerConcavityCalculator);

		dInt32 threadsCount = m_parallerConcavityCalculator.GetThreadCount();	
		data.SetCluster (clusterA);
		for (dInt32 i = 0; i < threadsCount; i ++) {		
			m_parallerConcavityCalculator.QueueJob(dgConvexHullRayCastContext::RayCastKernel, &data);
		}
		m_parallerConcavityCalculator.SynchronizationBarrier();
		dFloat64 concavity = data.GetConcavity();

		data.SetCluster (clusterB);
		for (dInt32 i = 0; i < threadsCount; i ++) {		
			m_parallerConcavityCalculator.QueueJob(dgConvexHullRayCastContext::RayCastKernel, &data);
		}
		m_parallerConcavityCalculator.SynchronizationBarrier();
		
		concavity = dMax(concavity, data.GetConcavity());
		//dFloat64 xxx = CalculateConcavitySingleThread (hull, mesh, clusterA, clusterB);
		//dAssert (fabs(concavity - xxx) < dFloat64 (1.0e-5f));
		return concavity;
	}

	dList<dgPairProxy>::dListNode* SubmitEdgeCost (dMeshEffect& mesh, dListNode* const clusterNodeA, dListNode* const clusterNodeB, dFloat64 perimeterHandicap)
	{
		dgHACDCluster& clusterA = clusterNodeA->GetInfo().m_nodeData;
		dgHACDCluster& clusterB = clusterNodeB->GetInfo().m_nodeData;
		const dBigVector* const points = (dBigVector*) mesh.GetVertexPool();

		bool flatStrip = true;
		dFloat64 tol = dFloat64 (1.0e-5f) * m_diagonal;
		dHACDClusterFace& clusterFaceA = clusterA.GetFirst()->GetInfo();
		dBigPlane plane(clusterFaceA.m_normal, -(points[clusterFaceA.m_edge->m_incidentVertex] % clusterFaceA.m_normal));

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

	
				dFloat64 concavity = dFloat64 (0.0f);
				if ((convexHull.GetCount() > 128) && ((clusterA.GetCount() > 256) || (clusterB.GetCount() > 256))) { 
					concavity = CalculateConcavityMultiThread (convexHull, mesh, clusterA, clusterB);
				} else {
					concavity = CalculateConcavitySingleThread (convexHull, mesh, clusterA, clusterB);
				}

				if (concavity < dFloat64(1.0e-3f)) {
					concavity = dFloat64(0.0f);
				}

				// see if the heap will overflow
				HeapCollectGarbage ();

				// add a new pair to the heap
				dList<dgPairProxy>::dListNode* pairNode = m_proxyList.Append();
				dgPairProxy& pair = pairNode->GetInfo();
				pair.m_nodeA = clusterNodeA;
				pair.m_nodeB = clusterNodeB;
				pair.m_distanceConcavity = concavity;
				pair.m_hierachicalClusterIndexA = clusterA.m_hierachicalClusterIndex;
				pair.m_hierachicalClusterIndexB = clusterB.m_hierachicalClusterIndex;

				pair.m_area = area;
				dFloat64 cost = CalculateConcavityMetric (concavity, area, perimeter * perimeterHandicap, clusterA.GetCount(), clusterB.GetCount());
				m_priorityHeap.Push(pairNode, cost);

				return pairNode;
			}
		}
		return pairNode;
	}


	void CollapseEdge (dList<dgPairProxy>::dListNode* const pairNode, dMeshEffect& mesh, dFloat64 concavity)
	{
		dListNode* adjacentNodes[1024];
		dgPairProxy& pair = pairNode->GetInfo();

		dgMemoryAllocator* const allocator = mesh.GetAllocator();


		dAssert((pair.m_nodeA && pair.m_nodeB) || (!pair.m_nodeA && !pair.m_nodeB));
		if (pair.m_nodeA && pair.m_nodeB) {
			// call the progress callback
			ReportProgress();

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
			dAssert (m_cancavityTreeIndex < (2 * (m_faceCount + 1)));

			dFloat64 treeConcavity = pair.m_distanceConcavity;
//			 dAssert (treeConcavity < 0.1);
			m_concavityTreeArray[m_cancavityTreeIndex] = new (allocator) dgHACDConvacityLookAheadTree (allocator, leftTree, rightTree, treeConcavity);
			clusterA.m_hierachicalClusterIndex = m_cancavityTreeIndex;
			clusterB.m_hierachicalClusterIndex = m_cancavityTreeIndex;
			m_cancavityTreeIndex ++;

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
				dListNode* const clusterNodeB = edgeNodeAB->GetInfo().m_node;
				dFloat64 weigh = edgeAB.m_backFaceHandicap;
				for (dgGraphNode<dgHACDCluster, dgHACDEdge>::dListNode* edgeNodeBA = clusterNodeB->GetInfo().GetFirst(); edgeNodeBA; edgeNodeBA = edgeNodeBA->GetNext()) {
					dListNode* const clusterNode = edgeNodeBA->GetInfo().m_node;
					if (clusterNode == clusterNodeA) {
						dgHACDEdge& edgeBA = edgeNodeBA->GetInfo().m_edgeData;
						dList<dgPairProxy>::dListNode* const proxyNode = SubmitEdgeCost (mesh, clusterNodeA, clusterNodeB, weigh * edgeBA.m_backFaceHandicap);
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
	}

#ifdef DG_BUILD_HIERACHICAL_HACD
	void CollapseClusters (dMeshEffect& mesh, dFloat64 maxConcavity, dInt32 maxClustesCount)
	{

		maxConcavity *= (m_diagonal * DG_CONCAVITY_SCALE);
		while (m_priorityHeap.GetCount()) {
			dFloat64 concavity =  m_priorityHeap.Value();
			dList<dgPairProxy>::dListNode* const pairNode = m_priorityHeap[0];
			m_priorityHeap.Pop();
			CollapseEdge (pairNode, mesh, concavity);

//if (m_progress == 24)
//break;

		}



		dInt32 treeCounts = 0;
		for (dInt32 i = 0; i < m_cancavityTreeIndex; i ++) {
			if (m_concavityTreeArray[i]) {
				m_concavityTreeArray[treeCounts] = m_concavityTreeArray[i];
				m_concavityTreeArray[i] = nullptr;
				treeCounts ++;
			}
		}

		if (treeCounts > 1) {

			for (dInt32 i = 0; i < treeCounts; i ++) {
				if (m_concavityTreeArray[i]->m_faceList.GetCount()==1) {
					delete m_concavityTreeArray[i];
					m_concavityTreeArray[i] = m_concavityTreeArray[treeCounts-1];
					m_concavityTreeArray[treeCounts-1]= nullptr;
					treeCounts --;
					i--;
				}
			}


			dFloat32 C = 10000;
			while (treeCounts > 1)	 {
				dgHACDConvacityLookAheadTree* const leftTree = m_concavityTreeArray[treeCounts-1];
				dgHACDConvacityLookAheadTree* const rightTree = m_concavityTreeArray[treeCounts-2];
				m_concavityTreeArray[treeCounts-1] = nullptr;
				m_concavityTreeArray[treeCounts-2] = new (mesh.GetAllocator()) dgHACDConvacityLookAheadTree (mesh.GetAllocator(), leftTree, rightTree, C);
				C *= 2;
				treeCounts --;
			}

		}

		dgHACDConvacityLookAheadTree* const tree = m_concavityTreeArray[0];
		dDownHeap<dgHACDConvacityLookAheadTree*, dFloat64> approximation(maxClustesCount * 2, mesh.GetAllocator());

		tree->ReduceByCount (maxClustesCount, approximation);
		//		tree->ReduceByConcavity (maxConcavity, approximation);

		while (approximation.GetCount()) {
			m_convexProximation.Append(approximation[0]);
			approximation.Pop();
		}
	}
#else 
	void CollapseClusters (dMeshEffect& mesh, dFloat64 maxConcavity, dInt32 maxClustesCount)
	{
		maxConcavity *= (m_diagonal * DG_CONCAVITY_SCALE);

		bool terminate = false;
		while (m_priorityHeap.GetCount() && !terminate) {
			dFloat64 concavity =  m_priorityHeap.Value();
			dList<dgPairProxy>::dListNode* const pairNode = m_priorityHeap[0];
			if ((concavity < maxConcavity) && (GetCount() < maxClustesCount)) {
				terminate  = true;
			} else {
				m_priorityHeap.Pop();
				CollapseEdge (pairNode, mesh, concavity);
			}
		}
	}
#endif

	dInt32 m_mark;
	dInt32 m_faceCount;
	dInt32 m_vertexMark;
	dInt32 m_progress;
	dInt32 m_cancavityTreeIndex;
	dInt32* m_vertexMarks;
	dFloat32 m_invFaceCount;
	dFloat64 m_diagonal;
	dBigVector* m_vertexPool;
	dList<dgPairProxy> m_proxyList;
	dgHACDConvacityLookAheadTree** m_concavityTreeArray;	
	dList<dgHACDConvacityLookAheadTree*> m_convexProximation;
	dUpHeap<dList<dgPairProxy>::dListNode*, dFloat64> m_priorityHeap;
	dgReportProgress m_reportProgressCallback;
	dgThreadHive m_parallerConcavityCalculator;
};

#endif

dMeshEffect* dMeshEffect::CreateSimplification(dInt32 maxVertexCount, dgReportProgress reportProgressCallback, void* const reportPrgressUserData) const
{
	if (GetVertexCount() <= maxVertexCount) {
		return new (GetAllocator()) dMeshEffect(*this); 
	}
dAssert (0);
return new (GetAllocator()) dMeshEffect(*this); 
/*
	//	dMeshEffect triangleMesh(*this);
	if (maxHullsCount <= 1) {
		maxHullsCount = 1;
	}
	if (maxConcavity <= dFloat32 (1.0e-5f)) {
		maxConcavity = dFloat32 (1.0e-5f);
	}

	if (maxVertexPerHull < 4) {
		maxVertexPerHull = 4;
	}
	ClampValue(backFaceDistanceFactor, dFloat32 (0.01f), dFloat32 (1.0f));

	if (reportProgressCallback) {
		reportProgressCallback (0.0f);
	}


	// make a copy of the mesh
	dMeshEffect mesh(*this);
	mesh.ClearAttributeArray();


	dInt32 faceCount = mesh.GetTotalFaceCount();
	if (faceCount  > meshSimplicationMaxFaceCount) {
		mesh.Triangulate();

		dPolyhedra polygon(GetAllocator());
		dInt32 mark = mesh.IncLRU();
		polygon.BeginFace();
		dPolyhedra::Iterator iter (mesh);
		for (iter.Begin(); iter; iter ++){
			dEdge* const face = &(*iter);

			if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
				dInt32	index[DG_MESH_EFFECT_POINT_SPLITED];

				dEdge* ptr = face;
				dInt32 indexCount = 0;
				do {
					index[indexCount] = ptr->m_incidentVertex;
					ptr->m_mark = mark;
					indexCount ++;
					ptr = ptr->m_next;
				} while (ptr != face);
				polygon.AddFace(indexCount, index);
			}
		}
		polygon.EndFace();

		polygon.Optimize(&mesh.m_points[0].m_x, sizeof (dFloat64), 1000.0f, meshSimplicationMaxFaceCount);

		mesh.RemoveAll();
		
		mark = polygon.IncLRU();
		mesh.BeginFace();
		dPolyhedra::Iterator iter1 (polygon);
		for (iter1.Begin(); iter1; iter1 ++){
			dEdge* const face = &(*iter1);
			if ((face->m_mark != mark) && (face->m_incidentFace > 0)) {
				dInt32	index[DG_MESH_EFFECT_POINT_SPLITED];
				dEdge* ptr = face;
				dInt32 indexCount = 0;
				do {
					ptr->m_mark = mark;
					index[indexCount] = dInt32 (ptr->m_incidentVertex);
					indexCount ++;
					ptr = ptr->m_next;
				} while (ptr != face);
				mesh.AddFace(indexCount, index);
			}
		}
		mesh.EndFace();

		faceCount = mesh.GetTotalFaceCount();
		mesh.ClearAttributeArray();
	}

	// create a general connectivity graph    
	dgHACDClusterGraph graph (mesh, backFaceDistanceFactor, reportProgressCallback);

	// calculate initial edge costs
	graph.SubmitInitialEdgeCosts (mesh);

	// collapse the graph
	graph.CollapseClusters (mesh, maxConcavity, maxHullsCount);

	// Create Partition Mesh
	return graph.CreatePatitionMesh (mesh, maxVertexPerHull);
*/
}

#endif