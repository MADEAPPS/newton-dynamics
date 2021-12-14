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

#ifndef __ND_POLYHEDRA_H__
#define __ND_POLYHEDRA_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndList.h"
#include "ndTree.h"
#include "ndHeap.h"
#include "ndHeap.h"
#include "ndDebug.h"
#include "ndClassAlloc.h"

class ndEdge;
class ndMatrix;
class ndBigPlane;
class ndBigVector;
class ndPolyhedra;
class ndVertexCollapseVertexMetric;

typedef ndInt64 ndEdgeKey;

class ndEdge
{
	public:
	ndEdge ();
	ndEdge (ndInt32 vertex, ndInt32 face, ndUnsigned64 userdata = 0);
	~ndEdge ();

	ndInt32 m_incidentVertex;
	ndInt32 m_incidentFace;
	ndUnsigned64 m_userData;
	ndEdge* m_next;
	ndEdge* m_prev;
	ndEdge* m_twin;
	ndInt32 m_mark;
} D_GCC_NEWTON_ALIGN_32 ;

class ndPolyhedra: public ndTree <ndEdge, ndEdgeKey>
{
	public:
	class ndPairKey
	{
		public:
		ndPairKey()
		{
		}

		ndPairKey(ndInt64 key)
			:m_key(ndUnsigned64(key))
		{
		}

		ndPairKey(ndInt32 keyHigh, ndInt32 keyLow)
			:m_keyLow(ndUnsigned32 (keyLow))
			,m_keyHigh(ndUnsigned32 (keyHigh))
		{
		}

		ndInt64 GetVal() const
		{
			return ndInt64(m_key);
		}

		ndInt32 GetLowKey() const
		{
			return ndInt32(m_keyLow);
		}

		ndInt32 GetHighKey() const
		{
			return ndInt32(m_keyHigh);
		}

		bool operator<(const ndPairKey& key) const
		{
			return m_key < key.m_key;
		}

		bool operator>(const ndPairKey& key) const
		{
			return m_key > key.m_key;
		}


		private:
		union 
		{
			ndUnsigned64 m_key;
			struct 
			{
				ndUnsigned32 m_keyLow;
				ndUnsigned32 m_keyHigh;
			};
		};
	};

	ndPolyhedra ();
	D_CORE_API ndPolyhedra (const ndPolyhedra &polyhedra);
	D_CORE_API virtual ~ndPolyhedra();

	virtual bool ReportProgress(ndFloat32) const { return true;}

	virtual void BeginFace();
	ndEdge* AddFace (ndInt32 v0, ndInt32 v1, ndInt32 v2);
	ndEdge* AddFace (ndInt32 count, const ndInt32* const index);
	D_CORE_API ndEdge* AddFace (ndInt32 count, const ndInt32* const index, const ndInt64* const userdata);
	D_CORE_API virtual bool EndFace ();
	D_CORE_API virtual void DeleteFace(ndEdge* const edge);

	D_CORE_API ndInt32 GetFaceCount() const;
	ndInt32 GetEdgeCount() const;
	ndInt32 GetLastVertexIndex() const;

	ndInt32 IncLRU() const;
	ndInt32 GetLRU() const;
	void SetLRU(ndInt32 lru) const;

	ndEdge* FindEdge (ndInt32 v0, ndInt32 v1) const;
	ndNode* FindEdgeNode (ndInt32 v0, ndInt32 v1) const;

	D_CORE_API ndEdge* AddHalfEdge (ndInt32 v0, ndInt32 v1);
	D_CORE_API void DeleteEdge (ndEdge* const edge);
	void DeleteEdge (ndInt32 v0, ndInt32 v1);

	D_CORE_API ndEdge* ConnectVertex (ndEdge* const e0, ndEdge* const e1);
	
	D_CORE_API bool FlipEdge (ndEdge* const edge);
	D_CORE_API ndEdge* SpliteEdge (ndInt32 newIndex, ndEdge* const edge);
	D_CORE_API ndBigVector FaceNormal (const ndEdge* const face, const ndFloat64* const vertex, ndInt32 strideInBytes) const;

	D_CORE_API void SavePLY(const char* const fileName, const ndFloat64* const vertex, ndInt32 strideInBytes) const;

	void BeginConectedSurface() const;
	D_CORE_API bool GetConectedSurface (ndPolyhedra &polyhedra) const;
	void EndConectedSurface() const;

	D_CORE_API ndMatrix CalculateSphere(ndBigVector& size, const ndFloat64* const vertex, ndInt32 strideInBytes) const;
	
	D_CORE_API void ChangeEdgeIncidentVertex (ndEdge* const edge, ndInt32 newIndex);
	D_CORE_API void DeleteDegenerateFaces (const ndFloat64* const pool, ndInt32 dstStrideInBytes, ndFloat64 minArea);

	D_CORE_API bool Optimize (const ndFloat64* const pool, ndInt32 strideInBytes, ndFloat64 tol, ndInt32 maxFaceCount = 1<<28);
	D_CORE_API void Triangulate (const ndFloat64* const vertex, ndInt32 strideInBytes, ndPolyhedra* const leftOversOut);
	D_CORE_API void ConvexPartition (const ndFloat64* const vertex, ndInt32 strideInBytes, ndPolyhedra* const leftOversOut);
	D_CORE_API bool IsFaceConvex(ndEdge* const face, const ndFloat64* const pool, ndInt32 strideInBytes) const;

	protected:
	D_CORE_API ndEdge* CollapseEdge(ndEdge* const edge);
	D_CORE_API bool PolygonizeFace(ndEdge* const face, const ndFloat64* const pool, ndInt32 stride);
	D_CORE_API bool TriangulateFace(ndEdge* const face, const ndFloat64* const pool, ndInt32 stride);

	private:
	void RefineTriangulation (const ndFloat64* const vertex, ndInt32 stride);
	void RefineTriangulation (const ndFloat64* const vertex, ndInt32 stride, const ndBigVector& normal, ndInt32 perimeterCount, ndEdge** const perimeter);
	void OptimizeTriangulation (const ndFloat64* const vertex, ndInt32 strideInBytes);
	void RemoveInteriorEdges (ndPolyhedra& polyhedraOut, const ndFloat64* const vertex, ndInt32 strideInBytes);
	void MarkAdjacentCoplanarFaces (ndPolyhedra& polyhedraOut, ndEdge* const face, const ndFloat64* const pool, ndInt32 strideInBytes);
	ndEdge* FindEarTip (ndEdge* const face, const ndFloat64* const pool, ndInt32 stride, ndDownHeap<ndEdge*, ndFloat64>& heap, const ndBigVector &normal) const;
	ndEdge* TriangulateFace (ndEdge* const face, const ndFloat64* const pool, ndInt32 stride, ndDownHeap<ndEdge*, ndFloat64>& heap, ndBigVector* const faceNormalOut);
		
	void RemoveHalfEdge (ndEdge* const edge);
	ndEdge* OptimizeCollapseEdge (ndEdge* const edge);
	bool IsOkToCollapse (const ndBigVector* const pool, ndEdge* const edge) const;
	ndFloat64 EdgePenalty (const ndBigVector* const pool, ndEdge* const edge, ndFloat64 dist) const;
	ndBigPlane EdgePlane (ndInt32 i0, ndInt32 i1, ndInt32 i2, const ndBigVector* const pool) const;
	void CalculateAllMetrics (ndVertexCollapseVertexMetric* const table, const ndBigVector* const pool) const;
	void CalculateVertexMetrics (ndVertexCollapseVertexMetric* const table, const ndBigVector* const pool, ndEdge* const edge) const;
	ndEdge* BestEdgePolygonizeFace(const ndBigVector& normal, ndEdge* const edge, const ndFloat64* const pool, ndInt32 stride, const ndBigVector& point) const;

	static ndInt32 GetInteriorDiagonals (ndPolyhedra& polyhedra, ndEdge** const diagonals, ndInt32 maxCount);
	static ndBigPlane UnboundedLoopPlane (ndInt32 i0, ndInt32 i1, ndInt32 i2, const ndBigVector* const pool);
	static void RemoveOuterColinearEdges(ndPolyhedra& flatFace, const ndFloat64* const vertex, ndInt32 stride);
	static void RemoveInteriorColinearEdges(ndPolyhedra& flatFace, const ndFloat64* const vertex, ndInt32 stride);
	static bool IsEssensialDiagonal (ndEdge* const diagonal, const ndBigVector& normal, const ndFloat64* const pool,  ndInt32 stride);
	static bool IsEssensialPointDiagonal (ndEdge* const diagonal, const ndBigVector& normal, const ndFloat64* const pool, ndInt32 stride);
	
	mutable ndInt32 m_baseMark;
	mutable ndInt32 m_edgeMark;
	mutable ndInt32 m_faceSecuence;
	friend class dPolyhedraDescriptor;
};

inline ndEdge::ndEdge ()
{
}

inline ndEdge::ndEdge (ndInt32 vertex, ndInt32 face, ndUnsigned64 userdata)
	:m_incidentVertex(vertex)
	,m_incidentFace(face)
	,m_userData(userdata)
	,m_next(nullptr)
	,m_prev(nullptr)
	,m_twin(nullptr)
	,m_mark(0)
{
}

inline ndEdge::~ndEdge ()
{
}

inline void ndPolyhedra::BeginFace ()
{
}

inline ndEdge* ndPolyhedra::AddFace (ndInt32 count, const ndInt32* const index) 
{
	return AddFace (count, index, nullptr);
}

inline ndEdge* ndPolyhedra::AddFace (ndInt32 v0, ndInt32 v1, ndInt32 v2)
{
	ndInt32 vertex[3];

	vertex [0] = v0;
	vertex [1] = v1;
	vertex [2] = v2;
	return AddFace (3, vertex, nullptr);
}

inline ndInt32 ndPolyhedra::GetEdgeCount() const
{
#ifdef _DEBUG
	ndInt32 edgeCount = 0;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) 
	{
		edgeCount ++;
	}
	dAssert (edgeCount == GetCount());;
#endif
	return GetCount();
}

inline ndPolyhedra::ndPolyhedra()
	:ndTree <ndEdge, ndInt64>()
	,m_baseMark(0)
	,m_edgeMark(0)
	,m_faceSecuence(0)
{
}

inline ndInt32 ndPolyhedra::GetLastVertexIndex() const
{
	ndInt32 maxVertexIndex = -1;
	Iterator iter(*this);
	for (iter.Begin(); iter; iter ++) 
	{
		const ndEdge* const edge = &(*iter);
		if (edge->m_incidentVertex > maxVertexIndex) 
		{
			maxVertexIndex = edge->m_incidentVertex;
		}
	}
	return maxVertexIndex + 1;
}

inline ndInt32 ndPolyhedra::IncLRU() const
{	
	m_edgeMark ++;
	dAssert (m_edgeMark < 0x7fffffff);
	return m_edgeMark;
}

inline ndInt32 ndPolyhedra::GetLRU() const
{
	return m_edgeMark;
}

inline void ndPolyhedra::SetLRU(ndInt32 lru) const
{
	if (lru > m_edgeMark) 
	{
		m_edgeMark = lru;
	}
}

inline void ndPolyhedra::BeginConectedSurface() const
{
	m_baseMark = IncLRU();
}

inline void ndPolyhedra::EndConectedSurface() const
{
}

inline ndPolyhedra::ndNode* ndPolyhedra::FindEdgeNode (ndInt32 i0, ndInt32 i1) const
{
	ndPairKey key (i0, i1);
	return Find (key.GetVal());
}

inline ndEdge *ndPolyhedra::FindEdge (ndInt32 i0, ndInt32 i1) const
{
	ndNode* const node = FindEdgeNode (i0, i1);
	return node ? &node->GetInfo() : nullptr;
}

inline void ndPolyhedra::DeleteEdge (ndInt32 v0, ndInt32 v1)
{
	ndPairKey pairKey (v0, v1);
	ndNode* const node = Find(pairKey.GetVal());
	ndEdge* const edge = node ? &node->GetInfo() : nullptr;
	if (!edge) 
	{
		return;
	}
	DeleteEdge (edge);
}

#endif

