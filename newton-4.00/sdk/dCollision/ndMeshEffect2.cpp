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
#include "dStack.h"
#include "dMatrix.h"
#include "ndMeshEffect.h"
#include "dConvexHull3d.h"
#include "dConvexHull4d.h"
#include "dDelaunayTetrahedralization.h"

//#include "dgCollisionConvexHull.h"


#if 0
class dgRayTrataAABBAccelerator: public ndMeshEffect::dMeshBVH
{
	public:
	dgRayTrataAABBAccelerator(const ndMeshEffect* const tetraMesh)
		:ndMeshEffect::dMeshBVH(tetraMesh)
	{
		Build();
	}

	dgMeshBVHNode* CreateLeafNode(dEdge* const face, void* const userData)
	{
		dgMemoryAllocator* const allocator = m_mesh->GetAllocator();
		dgMeshBVHNode* const node = new (allocator)dgMeshBVHNode(m_mesh, face, userData);

		dInt32 mark = m_mesh->GetLRU();

		dVector p0(m_mesh->GetVertex(face->m_twin->m_prev->m_incidentVertex));
		dVector p1(p0);
		dEdge* faceEdge = face;
		do {
			dEdge* twinFace = faceEdge->m_twin;
			do {
				twinFace->m_mark = mark;
				twinFace = twinFace->m_next;
			} while (twinFace != faceEdge->m_twin);

			dVector point(m_mesh->GetVertex(faceEdge->m_incidentVertex));
			p0 = point.GetMin(p0);
			p1 = point.GetMax(p1);

			faceEdge = faceEdge->m_next;
		} while (faceEdge != face);

		dVector padding(dFloat32(0.01f));
		p0 -= padding;
		p1 += padding;
		node->SetBox(p0, p1);
		return node;
	}
};


// idea taken from paper: Fast Tetrahedral Meshes with Good Dihedral Angles, by Francois Labelle Jonathan Richard Shewchuk
// but quite different approach.
class dgTetraIsoSufaceStuffing
{
	public:
	enum dgVertexSign
	{
		m_onSuface,
		m_inside,
		m_outside,
	};

	class dgGridDimension 
	{
		public:
		dBigVector m_origin;
		dFloat64 m_cellSize;
		dFloat64 m_diameter;
		dInt32 m_gridSizeX;
		dInt32 m_gridSizeY;
		dInt32 m_gridSizeZ;
		dInt32 m_innerSize;
		dInt32 m_outerSize;
	};

	template<class T, dInt32 size>
	class dgAssessor 
	{
		public:
		dgAssessor() 
			:m_count(0)
		{
		}

		dInt32 GetCount() const 
		{
			return m_count; 
		}

		const T& operator[] (dInt32 i) const
		{
			dAssert(i >= 0);
			dAssert(i < size);
			return m_elements[i];
		}

		T& operator[] (dInt32 i)
		{
			dAssert (i >= 0);
			dAssert (i < size);
			return m_elements[i];
		}

		void PushBack (const T data)
		{
			dAssert(m_count >= 0);
			dAssert(m_count < size);
			m_elements[m_count] = data;
			m_count ++;
		}

		private:
		dInt32 m_count;
		T m_elements[size];
	};

	typedef dgAssessor<dInt32, 4> dgTetrahedra; 
	typedef dgAssessor<dFloat32, 6> dgTetraEdgeCuts; 
	typedef dgAssessor<dInt32, 32> dgTetraToVertexNode; 

	class dgNormalMap
	{
		public:
		dgNormalMap()
			:m_count(sizeof (m_normal)/sizeof (m_normal[0]))
		{
			dVector p0(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
			dVector p1(dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
			dVector p2(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
			dVector p3(dFloat32(0.0f), dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f));
			dVector p4(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
			dVector p5(dFloat32(0.0f), dFloat32(0.0f), dFloat32(-1.0f), dFloat32(0.0f));

			dInt32 count = 0;
			dInt32 subdivitions = 1;
			TessellateTriangle(subdivitions, p4, p0, p2, count);
			TessellateTriangle(subdivitions, p0, p5, p2, count);
			TessellateTriangle(subdivitions, p5, p1, p2, count);
			TessellateTriangle(subdivitions, p1, p4, p2, count);
			TessellateTriangle(subdivitions, p0, p4, p3, count);
			TessellateTriangle(subdivitions, p5, p0, p3, count);
			TessellateTriangle(subdivitions, p1, p5, p3, count);
			TessellateTriangle(subdivitions, p4, p1, p3, count);
		}

		private:
		void TessellateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dInt32& count)
		{
			if (level) {
				dAssert(dAbs(p0.DotProduct(p0).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
				dAssert(dAbs(p1.DotProduct(p1).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
				dAssert(dAbs(p2.DotProduct(p2).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
				dVector p01(p0 + p1);
				dVector p12(p1 + p2);
				dVector p20(p2 + p0);

				p01 = p01.Scale(dgRsqrt(p01.DotProduct(p01).GetScalar()));
				p12 = p12.Scale(dgRsqrt(p12.DotProduct(p12).GetScalar()));
				p20 = p20.Scale(dgRsqrt(p20.DotProduct(p20).GetScalar()));

				dAssert(dAbs(p01.DotProduct(p01).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
				dAssert(dAbs(p12.DotProduct(p12).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
				dAssert(dAbs(p20.DotProduct(p20).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));

				TessellateTriangle(level - 1, p0, p01, p20, count);
				TessellateTriangle(level - 1, p1, p12, p01, count);
				TessellateTriangle(level - 1, p2, p20, p12, count);
				TessellateTriangle(level - 1, p01, p12, p20, count);
			} else {
				dBigPlane n(p0, p1, p2);
				n = n.Scale(dFloat64(1.0f) / sqrt(n.DotProduct3(n)));
				n.m_w = dFloat64(0.0f);
				dInt32 index = dgBitReversal(count, sizeof (m_normal) / sizeof (m_normal[0]));
				m_normal[index] = n;
				count++;
				dAssert(count <= sizeof (m_normal) / sizeof (m_normal[0]));
			}
		}

		public:
		dBigVector m_normal[32];
		dInt32 m_count;
	};

	class dgClosePointsAccelerator: public ndMeshEffect::dMeshBVH
	{
		public:
		dgClosePointsAccelerator(const ndMeshEffect* const mesh)
			:ndMeshEffect::dMeshBVH(mesh)
		{
			Build();
		}

		dgMeshBVHNode* CreateLeafNode(dEdge* const face, void* const userData)
		{
			dgMemoryAllocator* const allocator = m_mesh->GetAllocator();
			dgMeshBVHNode* const node = new (allocator)dgMeshBVHNode(m_mesh, face, userData);

			dInt32 mark = m_mesh->GetLRU();
			dAssert(mark != face->m_mark);

			dEdge* faceEdge = face;
			do {
				faceEdge->m_mark = mark;
				faceEdge = faceEdge->m_twin->m_next;
			} while (faceEdge != face);

			dVector padding(dFloat32(1.0f / 32.0f));
			dVector p(m_mesh->GetVertex(face->m_incidentVertex));
			dVector p0(p - padding);
			dVector p1(p + padding);
			node->SetBox(p0, p1);
			return node;
		}

		bool DoesTetrahedrumHasInsidePoints (const dgArray<dBigVector>& points, const dgTetrahedra& tetra) const 
		{
			dBigVector box0(dFloat64( 1.0e10f));
			dBigVector box1(dFloat64(-1.0e10f));
			for (dInt32 i = 0; i < 4; i++) {
				box0 = box0.GetMin(points[tetra[i]]);
				box1 = box1.GetMax(points[tetra[i]]);
			}

			dBigVector padding (dFloat64 (0.01f));
			box0 -= padding;
			box1 += padding;

			dList<dMeshBVH::dgMeshBVHNode*> overlapNodes(m_mesh->GetAllocator());
			GetOverlapNodes(overlapNodes, box0, box1);
			if (overlapNodes.GetCount()) {
				dBigVector p0(points[tetra[0]]);
				dBigVector p1(points[tetra[1]]);
				dBigVector p2(points[tetra[2]]);
				dBigVector p3(points[tetra[3]]);

				for (dList<dMeshBVH::dgMeshBVHNode*>::dListNode* node = overlapNodes.GetFirst(); node; node = node->GetNext()) {
					dEdge* const edge = node->GetInfo()->m_face;
					dBigVector point(m_mesh->GetVertex(edge->m_incidentVertex));
					dBigVector closestPoint(dgPointToTetrahedrumDistance(point, p0, p1, p2, p3));
					dBigVector error(closestPoint - point);
					dFloat64 error2 = error.DotProduct3(error);
					if (error2 < dFloat64(1.0e-8f)) {
						return true;
					}
				}
			}

			return false;
		}
	};

	class dgRayTraceAccelerator: public ndMeshEffect::dMeshBVH
	{
		enum dgTraceType
		{
			m_pointSide,
			m_pointOnSurface,
		};

		class dgRayTraceBase
		{
			public:
			dgRayTraceBase(dgTraceType type)
				:m_type(type)
			{
			}
			dgTraceType m_type;
		};

		class dgRayTracePointSide: public dgRayTraceBase
		{
			public:
			dgRayTracePointSide()
				:dgRayTraceBase(m_pointSide)
				,m_hitCount(0)
				,m_rayIsDegenerate(true)
			{
			}
			dInt32 m_hitCount;
			bool m_rayIsDegenerate;
		};

		class dgRayPointOnSurface: public dgRayTraceBase
		{
			public:
			dgRayPointOnSurface()
				:dgRayTraceBase(m_pointOnSurface)
				,m_param(2.0f)
			{
			}

			dFloat32 m_param;
		};

		public:
		dgRayTraceAccelerator(const ndMeshEffect* const mesh, dFloat64 diameter)
			:ndMeshEffect::dMeshBVH(mesh)
			,m_normals()
			,m_diameter(diameter)
		{
			Build();
		}

		dgMeshBVHNode* CreateLeafNode(dEdge* const face, void* const userData)
		{
			dgMemoryAllocator* const allocator = m_mesh->GetAllocator();
			dgMeshBVHNode* const node = new (allocator)dgMeshBVHNode(m_mesh, face, userData);
			dInt32 mark = m_mesh->GetLRU();
			dAssert(mark != face->m_mark);

			dEdge* faceEdge = face;
			do {
				faceEdge->m_mark = mark;
				faceEdge = faceEdge->m_next;
			} while (faceEdge != face);
			return node;
		}

		dFloat64 dgPointToRayDistance (const dBigVector& point, const dBigVector& ray_p0, const dBigVector& ray_p1, dgRayTracePointSide* const rayType) const
		{
			dBigVector dp(ray_p1 - ray_p0);
			dFloat64 den = dp.DotProduct3(dp);
			dFloat64 num = dp.DotProduct3(point - ray_p0);
			if ((num >= dFloat64 (0.0f)) && (num <= den)) { 
				dBigVector p (ray_p0 + dp.Scale (num / den));
				dBigVector dist (point - p);
				if (dist.DotProduct3(dist) < dFloat64 (1.0e-12f)) {
					rayType->m_rayIsDegenerate = true;
					return dFloat64 (-2.0f);
				}
			}
			return dFloat64 (2.0f);
		}

		dFloat64 PointSideTest(const dgMeshBVHNode* const faceNode, const dBigVector& point0, const dBigVector& point1, dgRayTracePointSide* const rayType) const
		{
			const dEdge* const edge = faceNode->m_face;
			const dBigVector p0 (m_mesh->GetVertex(edge->m_incidentVertex));
			const dBigVector p1 (m_mesh->GetVertex(edge->m_next->m_incidentVertex));
			const dBigVector p2 (m_mesh->GetVertex(edge->m_next->m_next->m_incidentVertex));

			const dBigVector e10(p1 - p0);
			const dBigVector e20(p2 - p0);
			const dFloat64 a00 = e10.DotProduct(e10).GetScalar();
			const dFloat64 a11 = e20.DotProduct(e20).GetScalar();
			const dFloat64 a01 = e10.DotProduct(e20).GetScalar();

			const dFloat64 det = a00 * a11 - a01 * a01;
			dAssert(det >= dFloat32(0.0f));
			if (dAbs(det) > dFloat32(1.0e-24f)) {
				dBigVector p0Point(point0 - p0);
				dBigVector normal(e10.CrossProduct(e20));
				dFloat64 t = -normal.DotProduct3(p0Point) / normal.DotProduct3(point1 - point0);
				if ((t > dFloat64(0.0f)) && (t < dFloat64(1.0f))) {
					dBigVector point(point0 + (point1 - point0).Scale(t));
					dBigVector variPoint(point - p0);	
					const dFloat64 b0 = e10.DotProduct(variPoint).GetScalar();
					const dFloat64 b1 = e20.DotProduct(variPoint).GetScalar();

					dFloat64 beta = b1 * a00 - a01 * b0;
					dFloat64 alpha = b0 * a11 - a01 * b1;

					if (beta <= dFloat32(0.0f)) {
						return dgPointToRayDistance (point, p0, p1, rayType);
					} else if (alpha <= dFloat32(0.0f)) {
						return dgPointToRayDistance (point, p0, p2, rayType);
					} else if ((alpha + beta) >= det) {
						return dgPointToRayDistance (point, p1, p2, rayType);
					}
					rayType->m_hitCount ++;
				}
			}
			return dFloat64 (2.0f);
		}


		dFloat64 PointSurfaceHit(const dgMeshBVHNode* const faceNode, const dBigVector& point0, const dBigVector& point1, dgRayPointOnSurface* const rayType) const
		{
			const dEdge* const edge = faceNode->m_face;
			const dBigVector p0(m_mesh->GetVertex(edge->m_incidentVertex));
			const dBigVector p1(m_mesh->GetVertex(edge->m_next->m_incidentVertex));
			const dBigVector p2(m_mesh->GetVertex(edge->m_next->m_next->m_incidentVertex));

			const dBigVector e10(p1 - p0);
			const dBigVector e20(p2 - p0);
			const dFloat64 a00 = e10.DotProduct(e10).GetScalar();
			const dFloat64 a11 = e20.DotProduct(e20).GetScalar();
			const dFloat64 a01 = e10.DotProduct(e20).GetScalar();

			const dFloat64 det = a00 * a11 - a01 * a01;
			dAssert(det >= dFloat32(0.0f));
			if (dAbs(det) > dFloat32(1.0e-24f)) {
				dBigVector p0Point(point0 - p0);
				dBigVector normal(e10.CrossProduct(e20));
				dFloat64 t = -normal.DotProduct3(p0Point) / normal.DotProduct3(point1 - point0);
				if ((t > dFloat64(0.0f)) && (t < dFloat64(1.0f))) {
					dBigVector point(point0 + (point1 - point0).Scale(t));
					dBigVector variPoint(point - p0);
					const dFloat64 b0 = e10.DotProduct(variPoint).GetScalar();
					const dFloat64 b1 = e20.DotProduct(variPoint).GetScalar();

					dFloat64 beta = b1 * a00 - a01 * b0;
					dFloat64 alpha = b0 * a11 - a01 * b1;

					if (beta <= dFloat32(0.0f)) {
						return dFloat64(2.0f);
					} else if (alpha <= dFloat32(0.0f)) {
						return dFloat64(2.0f);
					} else if ((alpha + beta) >= det) {
						return dFloat64(2.0f);
					}
					if (t < rayType->m_param) {
						rayType->m_param = dFloat32 (t);
					}
				}
			}
			return dFloat64(2.0f);
		}

		dFloat64 RayFaceIntersect(const dgMeshBVHNode* const faceNode, const dBigVector& point0, const dBigVector& point1, void* const userData) const
		{
			dgRayTraceBase* const rayType = (dgRayTraceBase*)userData;
			
			switch (rayType->m_type)
			{
				case m_pointSide:
					return PointSideTest(faceNode, point0, point1, (dgRayTracePointSide*) rayType);

				case m_pointOnSurface:
					return PointSurfaceHit(faceNode, point0, point1, (dgRayPointOnSurface*) rayType);
			}

			dAssert (0);
			return dFloat64 (-1.0f);
		}

		dgVertexSign CalculateVertexSide (const dBigVector& point0) const
		{
			dgRayTracePointSide hits;
			for (dInt32 i = 0; (i < m_normals.m_count) && hits.m_rayIsDegenerate; i ++) {
				hits.m_hitCount = 0;
				hits.m_rayIsDegenerate = false;
				dBigVector point1 (point0 + dBigVector (m_normals.m_normal[i].Scale (m_diameter))); 
				FaceRayCast (point0, point1, &hits);
			}
			dAssert (!hits.m_rayIsDegenerate);
			return (hits.m_hitCount & 1) ? m_inside : m_outside;
		}

		dFloat32 CalculateEdgeCut (const dBigVector& point0, const dBigVector& point1) const
		{
			dgRayPointOnSurface pointOnSurface;
			FaceRayCast (point0, point1, &pointOnSurface);
			return pointOnSurface.m_param;
		}

		dgNormalMap m_normals;
		dFloat64 m_diameter;
	};

	dgTetraIsoSufaceStuffing(const ndMeshEffect* const mesh, dFloat64 cellSize)
		:m_points(mesh->GetAllocator())
		,m_tetraList(mesh->GetAllocator())
		,m_pointCount(0)
		,m_tetraCount(0)
	{
		dgArray<dgVertexSign> vertexSide(mesh->GetAllocator());
		dgArray<dgTetraToVertexNode> tetraGraph(mesh->GetAllocator());
		dgArray<dgTetraEdgeCuts> tetraEdgeCuts(mesh->GetAllocator());

		dgGridDimension gridDim (CalculateGridSize(mesh, cellSize));
		dgClosePointsAccelerator closePointaAccelerator (mesh);
		dgRayTraceAccelerator rayAccelerator (mesh, gridDim.m_diameter);
		
		PopulateGridPoints (gridDim);
		CalculateVertexSide (vertexSide, rayAccelerator);
		BuildTetraGraph (gridDim, vertexSide, closePointaAccelerator, tetraGraph);
		//CalculateEdgeCuts (tetraEdgeCuts, tetraGraph, vertexSide, rayAccelerator);
		//SnapClosePoints (tetraEdgeCuts, tetraGraph, vertexSide, rayAccelerator);
	}

	void CalculateEdgeCuts (dgArray<dgTetraEdgeCuts>& tetraEdgeCuts, const dgArray<dgTetraToVertexNode>& tetraGraph, const dgArray<dgVertexSign>& vertexSide, const dgRayTraceAccelerator& rayAccelerator)
	{
		tetraEdgeCuts.Resize(m_tetraCount);
		for (dInt32 i = 0; i < m_tetraCount; i ++) {
			tetraEdgeCuts[i] = dgTetraEdgeCuts();
			for (dInt32 j = 0; j < 6; j ++) {
				tetraEdgeCuts[i].PushBack(dFloat32(-1.0f));
			}
		}

		for (dInt32 i = 0; i < m_pointCount; i ++) {
			if (vertexSide[i] == m_outside) {
				const dgTetraToVertexNode& graphNode = tetraGraph[i];
				for (dInt32 j = 0; j < graphNode.GetCount(); j ++) {
					dgTetraEdgeCuts& cuts = tetraEdgeCuts[graphNode[j]];
					const dgTetrahedra& tetra = m_tetraList[graphNode[j]];
					dAssert ((tetra[0] == i) || (tetra[1] == i) || (tetra[2] == i) || (tetra[3] == i));
					
					dInt32 index = 0;
					for (dInt32 i0 = 0; i0 < 3; i0 ++) {
						const dBigVector& p0 = m_points[tetra[i0]];
						for (dInt32 i1 = i0 + 1; i1 < 4; i1 ++) {
							if ((tetra[i0] == i) && (vertexSide[tetra[i1]] == m_inside)) {
								const dBigVector& p1 = m_points[tetra[i1]];
								dFloat32 param = rayAccelerator.CalculateEdgeCut (p0, p1);
								cuts[index] = param;
							}
							index ++;
							dAssert (index <= 6);
						}
					}
				}
			}
		}
	}

	void SnapClosePoints (dgArray<dgTetraEdgeCuts>& tetraEdgeCuts, const dgArray<dgTetraToVertexNode>& tetraGraph, const dgArray<dgVertexSign>& vertexSide, const dgRayTraceAccelerator& rayAccelerator)
	{
		for (dInt32 i = 0; i < m_pointCount; i++) {
			if (vertexSide[i] == m_outside) {
/*
				const dgTetraToVertexNode& graphNode = tetraGraph[i];
				for (dInt32 j = 0; j < graphNode.GetCount(); j++) {
					dgTetraEdgeCuts& cuts = tetraEdgeCuts[graphNode[j]];
					const dgTetrahedra& tetra = m_tetraList[graphNode[j]];
					dAssert((tetra[0] == i) || (tetra[1] == i) || (tetra[2] == i) || (tetra[3] == i));

					dInt32 index = 0;
					for (dInt32 i0 = 0; i0 < 3; i0++) {
						const dBigVector& p0 = m_points[tetra[i0]];
						for (dInt32 i1 = i0 + 1; i1 < 4; i1++) {
							if ((tetra[i0] == i) && (vertexSide[tetra[i1]] == m_inside)) {
								const dBigVector& p1 = m_points[tetra[i1]];
								dFloat32 param = rayAccelerator.CalculateEdgeCut(p0, p1);
								cuts[index] = param;
							}
							index++;
							dAssert(index <= 6);
						}
					}
				}
*/
			}
		}
	}

	void CalculateVertexSide (dgArray<dgVertexSign>& vertexSide, const dgRayTraceAccelerator& rayAccelerator)
	{
		vertexSide.Resize(m_pointCount);
		for (dInt32 i = 0; i < m_pointCount; i ++) {
			vertexSide[i] = rayAccelerator.CalculateVertexSide (m_points[i]);
		}
	}

	dgGridDimension CalculateGridSize(const ndMeshEffect* const mesh, dFloat64 cellsize) const
	{
		dBigVector minBox;
		dBigVector maxBox;
		mesh->CalculateAABB(minBox, maxBox);
		minBox -= (maxBox - minBox).Scale(dFloat64(1.e-3f));
		maxBox += (maxBox - minBox).Scale(dFloat64(1.e-3f));

		dBigVector mMinInt((minBox.Scale(dFloat64(1.0f) / cellsize)).Floor());
		dBigVector mMaxInt((maxBox.Scale(dFloat64(1.0f) / cellsize)).Floor() + dBigVector::m_one);
		dBigVector gridSize(mMaxInt - mMinInt + dBigVector::m_one);

		dBigVector size(maxBox - minBox);
		
		dgGridDimension gridDimension;
		gridDimension.m_origin = minBox; 
		gridDimension.m_cellSize = cellsize;
		gridDimension.m_diameter = sqrt (size.DotProduct3(size));
		gridDimension.m_gridSizeX = dInt32(gridSize.m_x);
		gridDimension.m_gridSizeY = dInt32(gridSize.m_y);
		gridDimension.m_gridSizeZ = dInt32(gridSize.m_z);

		gridDimension.m_innerSize = gridDimension.m_gridSizeX * gridDimension.m_gridSizeY * gridDimension.m_gridSizeZ;
		gridDimension.m_outerSize = gridDimension.m_innerSize + (gridDimension.m_gridSizeX + 1) * (gridDimension.m_gridSizeY + 1) * (gridDimension.m_gridSizeZ + 1);
		return gridDimension;
	}

	void PopulateGridPoints(const dgGridDimension& gridDimension)
	{
		m_pointCount = 0;
		m_points.Resize(gridDimension.m_outerSize);

		for (dInt32 z = 0; z < gridDimension.m_gridSizeZ; z++) {
			for (dInt32 y = 0; y < gridDimension.m_gridSizeY; y++) {
				for (dInt32 x = 0; x < gridDimension.m_gridSizeX; x++) {
					m_points[m_pointCount] = gridDimension.m_origin + dBigVector(x * gridDimension.m_cellSize, y * gridDimension.m_cellSize, z * gridDimension.m_cellSize, dFloat64(0.0f));
					m_pointCount++;
				}
			}
		}

		dBigVector outerOrigin(gridDimension.m_origin - dBigVector(gridDimension.m_cellSize * dFloat64(0.5f)));
		outerOrigin.m_w = dFloat64 (0.0f);
		for (dInt32 z = 0; z < gridDimension.m_gridSizeZ + 1; z++) {
			for (dInt32 y = 0; y < gridDimension.m_gridSizeY + 1; y++) {
				for (dInt32 x = 0; x < gridDimension.m_gridSizeX + 1; x++) {
					m_points[m_pointCount] = outerOrigin + dBigVector(x * gridDimension.m_cellSize, y * gridDimension.m_cellSize, z * gridDimension.m_cellSize, dFloat64(0.0f));
					m_pointCount++;
				}
			}
		}
	}

	void AddTetra(dgArray<dgTetraToVertexNode>& graph, const dgTetrahedra& tetra, const dgArray<dgVertexSign>& vertexSigns, const dgClosePointsAccelerator& closePoint)
	{
		dAssert(CalculateVolume(tetra) > dFloat64(0.0f));
		bool hasInsizePoints = false;
		hasInsizePoints = hasInsizePoints || (vertexSigns[tetra[0]] == m_inside);
		hasInsizePoints = hasInsizePoints || (vertexSigns[tetra[1]] == m_inside);
		hasInsizePoints = hasInsizePoints || (vertexSigns[tetra[2]] == m_inside);
		hasInsizePoints = hasInsizePoints || (vertexSigns[tetra[3]] == m_inside);
		hasInsizePoints = hasInsizePoints || closePoint.DoesTetrahedrumHasInsidePoints(m_points, tetra);

		if (hasInsizePoints) {
			m_tetraList[m_tetraCount] = tetra;
			dgTetrahedra& tetraEntry = m_tetraList[m_tetraCount];
			for (dInt32 i = 0; i < 4; i ++) {
				dInt32 vertexIndex = tetra[i];
				tetraEntry.PushBack(vertexIndex);
				graph[vertexIndex].PushBack(m_tetraCount);
			}
			m_tetraCount ++;
		}
	}

	void BuildTetraGraph(const dgGridDimension& gridDimension, const dgArray<dgVertexSign>& vertexSigns, const dgClosePointsAccelerator& closePoint, dgArray<dgTetraToVertexNode>& graph)
	{
		graph.Resize(m_pointCount);
		for (dInt32 i = 0; i < m_pointCount; i ++) {
			graph[i] = dgTetraToVertexNode();
		}

		dgDelaunayTetrahedralization delaunayTetrahedras(m_points.GetAllocator(), &m_points[0].m_x, m_pointCount, sizeof (dBigVector), dFloat32(0.0f));
		delaunayTetrahedras.RemoveUpperHull();

		for (dgDelaunayTetrahedralization::dListNode* node = delaunayTetrahedras.GetFirst(); node; node = node->GetNext()) {
			dgTetrahedra stuffingTetra;
			dgConvexHull4dTetraherum& delaunayTetra = node->GetInfo();
			
			for (dInt32 i = 0; i < 4; i ++) {
				stuffingTetra[i] = delaunayTetra.m_faces[0].m_index[i];
			}
			dFloat64 volume = CalculateVolume(stuffingTetra);
			if (volume < dFloat64 (0.0f)) {
				dSwap(stuffingTetra[0], stuffingTetra[1]);
			}
			AddTetra(graph, stuffingTetra, vertexSigns, closePoint);
		}

		
/*
		const dInt32 base = gridDimension.m_innerSize;
		for (dInt32 z = 0; z < gridDimension.m_gridSizeZ; z++) {
			for (dInt32 y = 0; y < gridDimension.m_gridSizeY; y++) {
				for (dInt32 x = 0; x < gridDimension.m_gridSizeX - 1; x++) {
					dgTetrahedra tetra;
					tetra[0] = ((z * gridDimension.m_gridSizeY + y) * gridDimension.m_gridSizeX) + x;
					tetra[1] = ((z * gridDimension.m_gridSizeY + y) * gridDimension.m_gridSizeX) + x + 1;
					tetra[2] = base + (((z + 0) * (gridDimension.m_gridSizeY + 1) + y + 0) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					tetra[3] = base + (((z + 0) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					AddTetra(graph, tetra, vertexSigns, closePoint);

					tetra[0] = ((z * gridDimension.m_gridSizeY + y) * gridDimension.m_gridSizeX) + x;
					tetra[1] = ((z * gridDimension.m_gridSizeY + y) * gridDimension.m_gridSizeX) + x + 1;
					tetra[3] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 0) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					tetra[2] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					AddTetra(graph, tetra, vertexSigns, closePoint);
				}
			}
		}

		for (dInt32 z = 0; z < gridDimension.m_gridSizeZ; z++) {
			for (dInt32 y = 0; y < gridDimension.m_gridSizeY - 1; y++) {
				for (dInt32 x = 0; x < gridDimension.m_gridSizeX; x++) {
					dgTetrahedra tetra;
					tetra[0] = ((z * gridDimension.m_gridSizeY + (y + 0)) * gridDimension.m_gridSizeX) + x;
					tetra[1] = ((z * gridDimension.m_gridSizeY + (y + 1)) * gridDimension.m_gridSizeX) + x;
					tetra[3] = base + (((z + 0) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					tetra[2] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					AddTetra(graph, tetra, vertexSigns, closePoint);

					tetra[0] = ((z * gridDimension.m_gridSizeY + (y + 0)) * gridDimension.m_gridSizeX) + x;
					tetra[1] = ((z * gridDimension.m_gridSizeY + (y + 1)) * gridDimension.m_gridSizeX) + x;
					tetra[2] = base + (((z + 0) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 0;
					tetra[3] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 0;
					AddTetra(graph, tetra, vertexSigns, closePoint);
				}
			}
		}

		for (dInt32 z = 0; z < gridDimension.m_gridSizeZ - 1; z++) {
			for (dInt32 y = 0; y < gridDimension.m_gridSizeY; y++) {
				for (dInt32 x = 0; x < gridDimension.m_gridSizeX; x++) {
					dgTetrahedra tetra;
					tetra[0] = (((z + 0) * gridDimension.m_gridSizeY + y + 0) * gridDimension.m_gridSizeX) + x;
					tetra[1] = (((z + 1) * gridDimension.m_gridSizeY + y + 0) * gridDimension.m_gridSizeX) + x;
					tetra[3] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 0) * (gridDimension.m_gridSizeX + 1)) + x + 0;
					tetra[2] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 0;
					AddTetra(graph, tetra, vertexSigns, closePoint);

					tetra[0] = (((z + 0) * gridDimension.m_gridSizeY + y + 0) * gridDimension.m_gridSizeX) + x;
					tetra[1] = (((z + 1) * gridDimension.m_gridSizeY + y + 0) * gridDimension.m_gridSizeX) + x;
					tetra[2] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 0) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					tetra[3] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					AddTetra(graph, tetra, vertexSigns, closePoint);
				}
			}
		}

		for (dInt32 z = 0; z < gridDimension.m_gridSizeZ; z++) {
			for (dInt32 y = 0; y < gridDimension.m_gridSizeY - 1; y++) {
				for (dInt32 x = 0; x < gridDimension.m_gridSizeX; x++) {
					dgTetrahedra tetra;
					tetra[0] = ((z * gridDimension.m_gridSizeY + (y + 0)) * gridDimension.m_gridSizeX) + x;
					tetra[1] = ((z * gridDimension.m_gridSizeY + (y + 1)) * gridDimension.m_gridSizeX) + x;
					tetra[2] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 0;
					tetra[3] = base + (((z + 1) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					AddTetra(graph, tetra, vertexSigns, closePoint);

					tetra[0] = ((z * gridDimension.m_gridSizeY + (y + 0)) * gridDimension.m_gridSizeX) + x;
					tetra[1] = ((z * gridDimension.m_gridSizeY + (y + 1)) * gridDimension.m_gridSizeX) + x;
					tetra[3] = base + (((z + 0) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 0;
					tetra[2] = base + (((z + 0) * (gridDimension.m_gridSizeY + 1) + y + 1) * (gridDimension.m_gridSizeX + 1)) + x + 1;
					AddTetra(graph, tetra, vertexSigns, closePoint);
				}
			}
		}
*/
	}

	dFloat64 CalculateVolume(const dgTetrahedra& tetra) const
	{
		const dBigVector& p0 = m_points[tetra[0]];
		const dBigVector& p1 = m_points[tetra[1]];
		const dBigVector& p2 = m_points[tetra[2]];
		const dBigVector& p3 = m_points[tetra[3]];
		dBigVector p10(p1 - p0);
		dBigVector p20(p2 - p0);
		dBigVector p30(p3 - p0);
		return p10.DotProduct3(p20.CrossProduct(p30));
	}

	dgArray<dBigVector> m_points;
	dgArray<dgTetrahedra> m_tetraList;
	dInt32 m_pointCount;
	dInt32 m_tetraCount;
};

void ndMeshEffect::LoadOffMesh(const char* const fileName)
{
	class ParceOFF
	{
		public:
		enum Token
		{
			m_off,
			m_value,
			m_end,
		};

		ParceOFF(FILE* const file)
			:m_file(file)
		{
		}

		Token GetToken(char* const buffer) const
		{
			while (!feof(m_file) && fscanf(m_file, "%s", buffer)) {
				if (buffer[0] == '#') {
					SkipLine();
				} else {
					if (!_stricmp(buffer, "OFF")) {
						return m_off;
					}
					return m_value;
				}
			}
			return m_end;
		}

		char* SkipLine() const
		{
			char tmp[1024];
			return fgets(tmp, sizeof (tmp), m_file);
		}

		dInt32 GetInteger() const
		{
			char buffer[1024];
			GetToken(buffer);
			return atoi(buffer);
		}

		dFloat64 GetFloat() const
		{
			char buffer[1024];
			GetToken(buffer);
			return atof(buffer);
		}

		FILE* m_file;
	};

	FILE* const file = fopen(fileName, "rb");
	if (file) {
		ParceOFF parcel(file);

		dInt32 vertexCount = 0;
		dInt32 faceCount = 0;
		//dInt32 edgeCount = 0;

		char buffer[1024];
		bool stillData = true;
		while (stillData) {
			ParceOFF::Token token = parcel.GetToken(buffer);
			switch (token) 
			{
				case ParceOFF::m_off:
				{
					vertexCount = parcel.GetInteger();
					faceCount = parcel.GetInteger();
					//					edgeCount = parcel.GetInteger();
					parcel.SkipLine();

					dgArray<dBigVector> points(GetAllocator());
					for (dInt32 i = 0; i < vertexCount; i++) {
						dFloat64 x = parcel.GetFloat();
						dFloat64 y = parcel.GetFloat();
						dFloat64 z = parcel.GetFloat();
						dBigVector p(x, y, z, dFloat32(0.0f));
						points[i] = p;
					}

					dgArray<dInt32> indexList(GetAllocator());
					dgArray<dInt32> faceVertex(GetAllocator());
					dInt32 index = 0;
					for (dInt32 i = 0; i < faceCount; i++) {
						const dInt32 faceVertexCount = parcel.GetInteger();
						faceVertex[i] = faceVertexCount;
						for (dInt32 j = 0; j < faceVertexCount; j++) {
							indexList[index] = parcel.GetInteger();
							index++;
						}
						parcel.SkipLine();
					}

					dMeshVertexFormat vertexFormat;
					vertexFormat.m_faceCount = faceCount;
					vertexFormat.m_faceIndexCount = &faceVertex[0];

					vertexFormat.m_vertex.m_data = &points[0].m_x;
					vertexFormat.m_vertex.m_strideInBytes = sizeof (dBigVector);
					vertexFormat.m_vertex.m_indexList = &indexList[0];
					BuildFromIndexList(&vertexFormat);

					CalculateNormals(30.0f * dDegreeToRad);
					stillData = false;
					break;
				}

				default:;
			}
		}

		fclose(file);
	}
}

void ndMeshEffect::LoadTetraMesh (const char* const filename)
{
	FILE* const file = fopen(filename, "rb");
	if (file) {
		dInt32 vertexCount;
		size_t ret = fscanf(file, "%d", &vertexCount);
		dgArray<dBigVector> points(GetAllocator());
		for (dInt32 i = 0; i < vertexCount; i ++) {
			float x;
			float y;
			float z;
			ret = fscanf(file, "%f %f %f", &x, &y, &z);
			points[i] = dBigVector (x, y, z, dFloat32 (0.0f));
		}
		
		BeginBuild();
		dInt32 tetras;
		ret = fscanf(file, "%d", &tetras);
		dgMemoryAllocator* const allocator = GetAllocator();
		for (dInt32 layers = 0; layers < tetras; layers ++) {
			dInt32 tetra[4];
			ret = fscanf(file, "%d %d %d %d", &tetra[0], &tetra[1], &tetra[2], &tetra[3]);
			ret = 0; 
			dBigVector pointArray[4];
			for (dInt32 i = 0; i < 4; i++) {
				dInt32 index = tetra[i];
				pointArray[i] = points[index];
			}

			ndMeshEffect convexMesh(allocator, &pointArray[0].m_x, 4, sizeof (dBigVector), dFloat64(0.0f));

			dAssert(convexMesh.GetCount());
			convexMesh.CalculateNormals(dFloat32(30.0f * dDegreeToRad));
			for (dInt32 i = 0; i < convexMesh.m_points.m_vertex.m_count; i++) {
				convexMesh.m_points.m_layers[i] = layers;
			}
			MergeFaces(&convexMesh);
		}
		EndBuild(dFloat64(1.0e-8f), false);
		fclose(file);
	}
}



ndMeshEffect* ndMeshEffect::CreateTetrahedraIsoSurface() const
{
/*
ndMeshEffect xxxx  (GetAllocator());
xxxx.BeginBuild();

xxxx.BeginBuildFace ();
xxxx.AddPoint (0.0, 0.0, -1.0);
xxxx.AddLayer (0);

xxxx.AddPoint (1.0, 0.0, 0.0);
xxxx.AddLayer (0);

xxxx.AddPoint (0.0, 0.0, 1.0);
xxxx.AddLayer (0);
xxxx.EndBuildFace ();

xxxx.BeginBuildFace ();
xxxx.AddPoint (0.0, 0.0, -1.0);
xxxx.AddLayer (1);

xxxx.AddPoint (0.0, 0.0, 1.0);
xxxx.AddLayer (1);

xxxx.AddPoint (-1.0, 0.0, 0.0);
xxxx.AddLayer (1);

xxxx.EndBuildFace ();
xxxx.EndBuild(dFloat64(1.0e-8f), false);
*/


	dgTetraIsoSufaceStuffing tetraIsoStuffing (this, dFloat64(0.125f));

	ndMeshEffect* delaunayPartition = nullptr;
	if (tetraIsoStuffing.m_tetraCount) {
		dgMemoryAllocator* const allocator = GetAllocator();
		delaunayPartition = new (allocator) ndMeshEffect (allocator);
		delaunayPartition->BeginBuild();
		dInt32 layer = 0;
		dBigVector pointArray[4];
		for (dInt32 j = 0; j < tetraIsoStuffing.m_tetraCount; j ++) {
			dgTetraIsoSufaceStuffing::dgTetrahedra& tetra = tetraIsoStuffing.m_tetraList[j];
			for (dInt32 i = 0; i < 4; i ++) {
				dInt32 index = tetra[i];
				pointArray[i] = tetraIsoStuffing.m_points[index];
			}
			ndMeshEffect convexMesh(allocator, &pointArray[0].m_x, 4, sizeof (dBigVector), dFloat64(0.0f));
			//dAssert (convexMesh.GetCount());
			//convexMesh.CalculateNormals(dFloat32 (30.0f * dgDEG2RAD));
			for (dInt32 i = 0; i < convexMesh.m_points.m_vertex.m_count; i++) {
				convexMesh.m_points.m_layers[i] = layer;
			}
			delaunayPartition->MergeFaces(&convexMesh);
			layer++;
		}
		delaunayPartition->EndBuild(dFloat64(1.0e-8f), false);
	}

	return delaunayPartition;
}

void ndMeshEffect::CreateTetrahedraLinearBlendSkinWeightsChannel (const ndMeshEffect* const tetrahedraMesh)
{
dAssert(0);
/*
	dgRayTrataAABBAccelerator accelerator (tetrahedraMesh);
	m_points.m_weights.Clear();
	m_points.m_weights.Reserve(m_points.m_vertex.m_count);

	dBigVector padding (dFloat64(1.0f / 32.0f));
	for (dInt32 i = 0; i < m_points.m_weights.m_count; i ++) {
		dBigVector p (m_points.m_vertex[i]);
		dBigVector p0 (p - padding);
		dBigVector p1 (p + padding);
		dList<dMeshBVH::dgMeshBVHNode*> overlapNodes (GetAllocator());
		accelerator.GetOverlapNodes (overlapNodes, p0, p1);
		dAssert (overlapNodes.GetCount());

		bool weightFound = false;
		for (dList<dMeshBVH::dgMeshBVHNode*>::dListNode* node = overlapNodes.GetFirst(); node; node = node->GetNext()) {
			dEdge* const edge = node->GetInfo()->m_face;

			dInt32 i0 = edge->m_incidentVertex;
			dInt32 i1 = edge->m_next->m_incidentVertex;
			dInt32 i2 = edge->m_prev->m_incidentVertex;
			dInt32 i3 = edge->m_twin->m_prev->m_incidentVertex;
			dBigVector q0 (tetrahedraMesh->m_points.m_vertex[i0]);
			dBigVector q1 (tetrahedraMesh->m_points.m_vertex[i1]);
			dBigVector q2 (tetrahedraMesh->m_points.m_vertex[i2]);
			dBigVector q3 (tetrahedraMesh->m_points.m_vertex[i3]);

			const dBigVector e10(q1 - q0);
			const dBigVector e20(q2 - q0);
			const dBigVector e30(q3 - q0);

			dAssert (e10.DotProduct(e10).GetScalar() > dFloat32 (0.0f));
			const dFloat64 d0 = sqrt(e10.DotProduct(e10).GetScalar());
			const dFloat64 invd0 = dFloat64(1.0f) / d0;
			const dFloat64 l10 = e20.DotProduct(e10).GetScalar() * invd0;
			const dFloat64 l20 = e30.DotProduct(e10).GetScalar() * invd0;

			dAssert ((e20.DotProduct(e20).GetScalar() - l10 * l10) > dFloat32 (0.0f));
			const dFloat64 desc11 = e20.DotProduct(e20).GetScalar() - l10 * l10;

			const dFloat64 d1 = sqrt(desc11);
			const dFloat64 invd1 = dFloat64(1.0f) / d1;
			const dFloat64 l21 = (e30.DotProduct(e20).GetScalar() - l20 * l10) * invd1;
			dAssert (e30.DotProduct(e30).GetScalar() - l20 * l20 - l21 * l21 > dFloat32 (0.0f));
			const dFloat64 desc22 = e30.DotProduct(e30).GetScalar() - l20 * l20 - l21 * l21;

			dBigVector p0Point(p - q0);
			const dFloat64 d2 = sqrt(desc22);
			const dFloat64 invd2 = dFloat64(1.0f) / d2;

			const dFloat64 b0 = e10.DotProduct(p0Point).GetScalar();
			const dFloat64 b1 = e20.DotProduct(p0Point).GetScalar();
			const dFloat64 b2 = e30.DotProduct(p0Point).GetScalar();

			dFloat64 u1 = b0 * invd0;
			dFloat64 u2 = (b1 - l10 * u1) * invd1;
			dFloat64 u3 = (b2 - l20 * u1 - l21 * u2) * invd2;

			u3 = u3 * invd2;
			u2 = (u2 - l21 * u3) * invd1;
			u1 = (u1 - l10 * u2 - l20 * u3) * invd0;
			if ((u1 >= dFloat64(0.0f)) && (u2 >= dFloat64(0.0f)) && (u3 >= dFloat64(0.0f)) && ((u1 + u2 + u3) <= dFloat64(1.0f))) {
				dBigVector r0 (q0 + e10.Scale(u1) + e20.Scale(u2) + e30.Scale(u3));

				dFloat64 u0 = dFloat64 (1.0f) - u1 - u2 - u3;
				dBigVector r1 (q0.Scale (u0) + q1.Scale (u1) + q2.Scale (u2) + q3.Scale (u3));
				dgWeights& weighSet = m_points.m_weights[i];

				weighSet.m_controlIndex[0] = i0;
				weighSet.m_weightBlends[0] = dFloat32(u0);

				weighSet.m_controlIndex[1] = i1;
				weighSet.m_weightBlends[1] = dFloat32(u1);

				weighSet.m_controlIndex[2] = i2;
				weighSet.m_weightBlends[2] = dFloat32(u2);

				weighSet.m_controlIndex[3] = i3;
				weighSet.m_weightBlends[3] = dFloat32(u3);

				weightFound = true;
				break;
			}
		}
		dAssert (weightFound);
		if (!weightFound) {
			dTrace (("%d %f %f %f\n", i, p.m_x, p.m_y, p.m_z));
		}

		overlapNodes.RemoveAll();
	}
*/
}
#endif

ndMeshEffect* ndMeshEffect::CreateVoronoiConvexDecomposition(dInt32 pointCount, dVector* const pointCloud, dInt32 interiorMaterialIndex, const dMatrix& textureProjectionMatrix)
{
	dStack<dBigVector> buffer(pointCount + 32);
	dBigVector* const pool = &buffer[0];
	dInt32 count = 0;
	dFloat64 quantizeFactor = dFloat64(16.0f);
	dFloat64 invQuantizeFactor = dFloat64(1.0f) / quantizeFactor;
	
	dBigVector pMin(dFloat32(1.0e10f), dFloat32(1.0e10f), dFloat32(1.0e10f), dFloat32(0.0f));
	dBigVector pMax(dFloat32(-1.0e10f), dFloat32(-1.0e10f), dFloat32(-1.0e10f), dFloat32(0.0f));
	for (dInt32 i = 0; i < pointCount; i++)
	{
		dFloat64 x = pointCloud[i].m_x;
		dFloat64 y = pointCloud[i].m_y;
		dFloat64 z = pointCloud[i].m_z;
		x = floor(x * quantizeFactor) * invQuantizeFactor;
		y = floor(y * quantizeFactor) * invQuantizeFactor;
		z = floor(z * quantizeFactor) * invQuantizeFactor;
		dBigVector p(x, y, z, dFloat64(0.0f));
		pMin = dBigVector(dMin(x, pMin.m_x), dMin(y, pMin.m_y), dMin(z, pMin.m_z), dFloat64(0.0f));
		pMax = dBigVector(dMax(x, pMax.m_x), dMax(y, pMax.m_y), dMax(z, pMax.m_z), dFloat64(0.0f));
		pool[count] = p;
		count++;
	}

	dBigVector meshMin;
	dBigVector meshMax;
	CalculateAABB(meshMin, meshMax);

	pMin = pMin.GetMin(meshMin);
	pMax = pMax.GetMax(meshMax);

	// add the bounding box as a barrier
	pool[count + 0] = dBigVector(pMin.m_x, pMin.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 1] = dBigVector(pMax.m_x, pMin.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 2] = dBigVector(pMin.m_x, pMax.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 3] = dBigVector(pMax.m_x, pMax.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 4] = dBigVector(pMin.m_x, pMin.m_y, pMax.m_z, dFloat64(0.0f));
	pool[count + 5] = dBigVector(pMax.m_x, pMin.m_y, pMax.m_z, dFloat64(0.0f));
	pool[count + 6] = dBigVector(pMin.m_x, pMax.m_y, pMax.m_z, dFloat64(0.0f));
	pool[count + 7] = dBigVector(pMax.m_x, pMax.m_y, pMax.m_z, dFloat64(0.0f));
	count += 8;
	
	dStack<dInt32> indexList(count);
	count = dVertexListToIndexList(&pool[0].m_x, sizeof(dBigVector), 3, count, &indexList[0], dFloat64(5.0e-2f));
	dAssert(count >= 8);
	
	dFloat64 maxSize = dMax(pMax.m_x - pMin.m_x, pMax.m_y - pMin.m_y, pMax.m_z - pMin.m_z);
	pMin -= dBigVector(maxSize, maxSize, maxSize, dFloat64(0.0f));
	pMax += dBigVector(maxSize, maxSize, maxSize, dFloat64(0.0f));

	// add the a guard zone, so that we do no have to clip
	dInt32 guardVertexKey = count;
	pool[count + 0] = dBigVector(pMin.m_x, pMin.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 1] = dBigVector(pMax.m_x, pMin.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 2] = dBigVector(pMin.m_x, pMax.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 3] = dBigVector(pMax.m_x, pMax.m_y, pMin.m_z, dFloat64(0.0f));
	pool[count + 4] = dBigVector(pMin.m_x, pMin.m_y, pMax.m_z, dFloat64(0.0f));
	pool[count + 5] = dBigVector(pMax.m_x, pMin.m_y, pMax.m_z, dFloat64(0.0f));
	pool[count + 6] = dBigVector(pMin.m_x, pMax.m_y, pMax.m_z, dFloat64(0.0f));
	pool[count + 7] = dBigVector(pMax.m_x, pMax.m_y, pMax.m_z, dFloat64(0.0f));
	count += 8;
	
	dDelaunayTetrahedralization delaunayTetrahedras(&pool[0].m_x, count, sizeof(dBigVector), dFloat32(0.0f));
	delaunayTetrahedras.RemoveUpperHull();
	
	//	delaunayTetrahedras.Save("xxx0.txt");
	dInt32 tetraCount = delaunayTetrahedras.GetCount();
	dStack<dBigVector> voronoiPoints(tetraCount + 32);
	dStack<dDelaunayTetrahedralization::dListNode*> tetradrumNode(tetraCount);
	dTree<dList<dInt32>, dInt32> delaunayNodes;
	
	dInt32 index = 0;
	const dConvexHull4dVector* const convexHulPoints = delaunayTetrahedras.GetHullVertexArray();
	for (dDelaunayTetrahedralization::dListNode* node = delaunayTetrahedras.GetFirst(); node; node = node->GetNext()) 
	{
		dConvexHull4dTetraherum& tetra = node->GetInfo();
		voronoiPoints[index] = tetra.CircumSphereCenter(convexHulPoints);
		tetradrumNode[index] = node;
	
		for (dInt32 i = 0; i < 4; i++) 
		{
			dTree<dList<dInt32>, dInt32>::dTreeNode* header = delaunayNodes.Find(tetra.m_faces[0].m_index[i]);
			if (!header) 
			{
				dList<dInt32> list;
				header = delaunayNodes.Insert(list, tetra.m_faces[0].m_index[i]);
			}
			header->GetInfo().Append(index);
		}
		index++;
	}
	
	const dFloat32 normalAngleInRadians = dFloat32(30.0f * dDegreeToRad);
	ndMeshEffect* const voronoiPartition = new ndMeshEffect;
	voronoiPartition->BeginBuild();
	dInt32 layer = 0;
	dTree<dList<dInt32>, dInt32>::Iterator iter(delaunayNodes);
	for (iter.Begin(); iter; iter++) 
	{
		dTree<dList<dInt32>, dInt32>::dTreeNode* const nodeNode = iter.GetNode();
		const dList<dInt32>& list = nodeNode->GetInfo();
		dInt32 key = nodeNode->GetKey();
	
		if (key < guardVertexKey) 
		{
			dBigVector pointArray[512];
			dInt32 indexArray[512];
	
			dInt32 count1 = 0;
			for (dList<dInt32>::dListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) 
			{
				dInt32 i = ptr->GetInfo();
				pointArray[count1] = voronoiPoints[i];
				count1++;
				dAssert(count1 < dInt32(sizeof(pointArray) / sizeof(pointArray[0])));
			}
	
			count1 = dVertexListToIndexList(&pointArray[0].m_x, sizeof(dBigVector), 3, count1, &indexArray[0], dFloat64(1.0e-3f));
			if (count1 >= 4) 
			{
				ndMeshEffect convexMesh(&pointArray[0].m_x, count1, sizeof(dBigVector), dFloat64(0.0f));
				if (convexMesh.GetCount()) 
				{
					convexMesh.CalculateNormals(normalAngleInRadians);
					convexMesh.UniformBoxMapping(interiorMaterialIndex, textureProjectionMatrix);
					for (dInt32 i = 0; i < convexMesh.m_points.m_vertex.GetCount(); i++) 
					{
						convexMesh.m_points.m_layers[i] = layer;
					}
					voronoiPartition->MergeFaces(&convexMesh);
					layer++;
				}
			}
		}
	}
	voronoiPartition->EndBuild(dFloat64(1.0e-8f), false);
	//voronoiPartition->SaveOFF("xxx0.off");
	return voronoiPartition;
}
